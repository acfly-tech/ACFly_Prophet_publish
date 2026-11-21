/*
 * 通用 MAVLink 路由实现 (静态单例方式).
 *
 * 功能:
 *   把挂在任意 CommuPort 上的外设 (云台/相机/伴随计算机等 MAVLink 设备)
 *   与挂在其它的 CommuPort 上的地面站自动打通, 无需为每种外设写驱动.
 *   外设发来的报文按需转发到其它链路; GCS 发给外设的命令按据此学到的
 *   路由表回送到外设所在的链路. 客户把外设插在任意 UART 上即插即用.
 *
 * 本模块仅操作 CommuPort (get_CommuPort / get_CommuPortByPortId).
 */

#include "Commulink.hpp"
#include "MavlinkCMDProcess.hpp" // CmdMsg
#include "MavlinkRCProcess.hpp"
#include "mavlink.h"
#include <string.h>

/* ===================== 内部常量与类型 ===================== */

// 路由表最大条目数. 单机外设场景足够。
#define MAVLINK_ROUTING_MAX_ROUTES 20

// 路由表条目: 记录某个 (sysid, compid) 是在哪条 CommuPort 上被看到的.
struct MAVLink_Route
{
    uint8_t sysid;   // 对端的 system id
    uint8_t compid;  // 对端的 component id
    uint8_t channel; // 收到该对端时所在的 CommuPort 端口索引 (CommuPorts 下标)
    uint8_t mavtype; // 若由 HEARTBEAT 学到, 记录心跳里的 type 字段, 否则 0
};

/* ===================== 单例状态 ===================== */

// 全局静态路由表. 单例实现.
static struct
{
    // 路由条目数组, 线性表 (条目少, 不必用散列).
    MAVLink_Route routes[MAVLINK_ROUTING_MAX_ROUTES];
    // 已用条目数.
    uint8_t num_routes;
} s_routing;

// 路由功能总开关. 由 Lk_CommuCfg 的 CommuCfg_mavlinkRoute_BIT 控制,
// 在 Commulink 任务启动时读取参数后设置. 默认关闭, 未使能时所有路由
// 接口直接返回失败, 不影响 FC 原有处理逻辑.
static bool s_routing_enabled = false;

// GCS 口标记位图: bit n 置 1 表示第 n 个 CommuPort 上检测到过 GCS 心跳.
// 独立于路由使能开关和路由学习过滤, 供集群从机->GCS 转发时过滤目标口
// (只发给接 GCS 的口, 避免把从机消息误发到外设口).
static uint32_t s_gcs_port_mask = 0;

void mavlink_routing_set_enabled(bool ena)
{
    s_routing_enabled = ena;
}

// 判断某条 CommuPort 是否接的是地面站 (GCS).
// 依据: 该口曾经收到过 type == MAV_TYPE_GCS 的心跳.
bool mavlink_routing_is_gcs_port(uint8_t chan)
{
    if (chan < 32)
        return (s_gcs_port_mask & (uint32_t) (1UL << chan)) != 0;
    return false;
}

/* ===================== 工具函数 ===================== */

// 从 payload 中按消息元信息提取 target_system / target_component.
// 任何带目标字段的消息都能自动识别, 不需要为每种 msgid 写 switch. 无该字段时返回 -1.
static void routing_get_targets(const mavlink_message_t *msg, int16_t &tgt_sys, int16_t &tgt_comp)
{
    tgt_sys = -1;
    tgt_comp = -1;
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msg->msgid);
    if (entry == 0)
        return;
    if (entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)
    {
        // target_system 是 uint8, 直接从 payload 偏移处读取.
        tgt_sys = ((const uint8_t *) msg->payload64)[entry->target_system_ofs];
    }
    if (entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)
    {
        tgt_comp = ((const uint8_t *) msg->payload64)[entry->target_component_ofs];
    }
}

// 仅 HEARTBEAT 时调用: 登记或刷新一条路由.
// 不学习:
//   - 广播系统 id (sysid==0, MAV_COMP_ID_ALL 的系统级含义, 不对应具体系统).
//   - 广播组件 id (compid==0): 任何系统都不应把 0 作为自己的真实组件身份,
//     这是 MAV_COMP_ID_ALL, 没有具体组件可路由, 学进表只会产生"幽灵"条目.
//   - 飞控自身 (sysid==FC 且 compid==FC): 我们知道自己是谁, 不用学.
static void routing_learn_from_heartbeat(uint8_t in_channel, const mavlink_message_t *msg)
{
    if (msg->sysid == 0)
        return;
    if (msg->compid == 0)
        return;
    if (msg->sysid == get_CommulinkSysId() && msg->compid == get_CommulinkCompId())
        return;

    // 已存在则刷新 mavtype (可能之前学到时不是心跳, 没拿到 type).
    for (uint8_t i = 0; i < s_routing.num_routes; ++i)
    {
        if (s_routing.routes[i].sysid == msg->sysid && s_routing.routes[i].compid == msg->compid && s_routing.routes[i].channel == in_channel)
        {
            if (s_routing.routes[i].mavtype == 0)
                s_routing.routes[i].mavtype = mavlink_msg_heartbeat_get_type(msg);
            return;
        }
    }

    // 新增条目.
    if (s_routing.num_routes < MAVLINK_ROUTING_MAX_ROUTES)
    {
        s_routing.routes[s_routing.num_routes].sysid = msg->sysid;
        s_routing.routes[s_routing.num_routes].compid = msg->compid;
        s_routing.routes[s_routing.num_routes].channel = in_channel;
        s_routing.routes[s_routing.num_routes].mavtype = mavlink_msg_heartbeat_get_type(msg);
        ++s_routing.num_routes;
    }
}

// 把一条已解包的消息通过指定 CommuPort 发出.
// 内部包好通道锁 (mavlink_lock_chan / mavlink_unlock_chan),
// 与 Commulink 任务里其它发送路径用法一致. 静默失败返回 false.
static bool routing_send_to_channel(uint8_t chan, const mavlink_message_t *msg)
{
    const Port *port = get_CommuPort(chan);
    if (port == 0 || port->write == 0 || port->lock == 0 || port->unlock == 0)
        return false;
    if (mavlink_lock_chan(chan, 0.01))
    {
        mavlink_send_msg(port, msg, 0, 0.01);
        mavlink_unlock_chan(chan);
        return true;
    }
    return false;
}

// 计算屏蔽位掩码: 哪些 CommuPort 不应接收本次转发.
// 屏蔽规则:
//   1) 源链路本身 (永远不发回源链路);
//   2) 路由表里学过 "源 sysid" 的其它链路 (这些链路上 GCS/对端已能直接看到源,
//      再转发就是重复, 典型场景: 串口1 和串口2 都接 GCS).
// 返回值 bit n 置 1 表示屏蔽第 n 个 CommuPort.
// 用 uint32_t 位宽, 足以覆盖所有可能的 CommuPort (最多 32 条链路).
static uint32_t routing_blocked_mask(uint8_t in_channel, uint8_t src_sysid)
{
    uint32_t mask = 0;
    // 永远不发回源链路.
    if (in_channel < 32)
        mask |= (uint32_t) (1UL << in_channel);
    // 屏蔽源 sysid 学过的所有链路 (sysid 粒度, 搜出该源心跳所在的所有链路,
    // 能覆盖 "两个 GCS 同 sysid 接两口" 这种回环场景).
    for (uint8_t i = 0; i < s_routing.num_routes; ++i)
    {
        if (s_routing.routes[i].sysid == src_sysid)
        {
            if (s_routing.routes[i].channel < 32)
                mask |= (uint32_t) (1UL << s_routing.routes[i].channel);
        }
    }
    return mask;
}

/* ===================== HEARTBEAT 特判 ===================== */

// 心跳无 target 字段, 不能走通用转发逻辑, 单独处理.
// 心跳必须转发, 否则 GCS 永远识别不到外设在线 (云台 UI 起不来等).
// 转发策略: 广播到除屏蔽链路以外的所有可用 CommuPort.
static void routing_handle_heartbeat(uint8_t in_channel, const mavlink_message_t *msg)
{
    uint32_t blocked = routing_blocked_mask(in_channel, msg->sysid);
    for (uint8_t c = 0; c < MAVLINK_COMM_NUM_BUFFERS; ++c)
    {
        if (blocked & (1UL << c))
            continue;
        routing_send_to_channel(c, msg);
    }
}

/* ===================== 对外接口 ===================== */

// 收到一条解包成功的消息时调用.
//   in_channel : 收到该消息的 CommuPort 索引.
//   返回 true  : 该消息已被路由模块当作"给外设/GCS 透传"消费掉, 调用方应跳过
//               FC 本地处理 (避免 FC 抢答 COMMAND_ACK 或误执行).
//   返回 false : 该消息未被路由消费, 调用方按原逻辑 (Mavlink_RC_Process 等) 处理.
//
// 设计原则:
//   - FC 显式己方的消息 (target 命中本机) -> 本地处理, 不消费.
//   - target 指向路由表里已知的外设 -> 转发到该外设所在链路, 消费.
//   - target 是广播/无 target 字段    -> 不消费, 交 FC 处理.
bool mavlink_routing_handle_message(uint8_t in_channel, const mavlink_message_t *msg)
{
    // 自身环回 (串口接错线) 直接忽略, 不处理也不转发.
    if (msg->sysid == get_CommulinkSysId() && msg->compid == get_CommulinkCompId())
        return false;

    // 更新 GCS 口标记: 检测到 GCS 心跳 (type == MAV_TYPE_GCS) 就标记该口.
    // 独立于路由使能开关和学习过滤 (QGC 心跳 compid 可能是 0, 不能靠路由表识别),
    // 供集群从机->GCS 转发时判断目标口.
    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT && mavlink_msg_heartbeat_get_type(msg) == MAV_TYPE_GCS)
    {
        if (in_channel < 32)
            s_gcs_port_mask |= (uint32_t) (1UL << in_channel);
    }

    // 未使能路由功能 (Lk_CommuCfg 未置 CommuCfg_mavlinkRoute_BIT): 直接放行,
    // 交给 FC 原有逻辑处理.
    if (!s_routing_enabled)
        return false;

    // 源组件为广播 (compid==0, MAV_COMP_ID_ALL): 这是"无具体身份"的异常报文,
    // 没有真实的源组件可言. 约定: 此类消息既不学习也不转发 (但仍允许 FC 本地处理,
    // 例如地面站发的 target_component=0 广播命令仍由 Mavlink_RC_Process 执行).
    bool src_is_broadcast_component = (msg->compid == 0);

    // 1) 路由学习: 只在 HEARTBEAT 时登记, 降低 CPU 占用.
    //    源 compid==0 / sysid==0 在 routing_learn_from_heartbeat 内部已统一过滤.
    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
        routing_learn_from_heartbeat(in_channel, msg);

    // 2) HEARTBEAT 走专门的广播转发, 不消费 (心跳同时会进 FC 的 Mavlink_RC_Process,
    //    用来刷新 GCS_is_MP / 协议版本, 这部分逻辑不能丢).
    //    源 compid==0 的心跳不转发.
    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        if (!src_is_broadcast_component)
            routing_handle_heartbeat(in_channel, msg);
        return false;
    }

    // 3) 提取目标. 无 target 字段的消息 tgt_* 保持 -1, 按 "无目标" 处理.
    int16_t tgt_sys = -1, tgt_comp = -1;
    routing_get_targets(msg, tgt_sys, tgt_comp);

    uint8_t self_sys = get_CommulinkSysId();
    uint8_t self_comp = get_CommulinkCompId();

    bool broadcast_system = (tgt_sys == 0 || tgt_sys == -1);
    bool broadcast_component = (tgt_comp == 0 || tgt_comp == -1);
    // 是否明确指向飞控自身: target_system 命中本机 且 (compid 为广播或等于本机).
    bool match_self = (!broadcast_system && tgt_sys == self_sys) && (broadcast_component || tgt_comp == self_comp);

    // 明确指向本机的非广播命令/参数: 交给 FC 处理, 不消费.
    if (match_self)
        return false;

    // 源 compid==0 的消息: 不转发, 交 FC 处理 (语义见函数开头注释).
    if (src_is_broadcast_component)
        return false;

    // 4) 转发到路由表里匹配的链路.
    //    匹配条件:
    //      - 广播系统 (含无 target 字段): 转发到所有未屏蔽链路.
    //      - 非广播: 转发到 target_system/target_component 命中的链路.
    bool forwarded = false;
    uint32_t blocked = routing_blocked_mask(in_channel, msg->sysid);

    bool sent_to_chan[MAVLINK_COMM_NUM_BUFFERS];
    memset(sent_to_chan, 0, sizeof(sent_to_chan));

    for (uint8_t i = 0; i < s_routing.num_routes; ++i)
    {
        uint8_t chan = s_routing.routes[i].channel;
        if (chan >= MAVLINK_COMM_NUM_BUFFERS)
            continue;
        if (blocked & (1UL << chan))
            continue;
        if (sent_to_chan[chan])
            continue;

        bool should_forward = false;
        if (broadcast_system)
        {
            // 广播系统: 转发到所有 (未屏蔽) 学过的链路.
            should_forward = true;
        }
        else if (s_routing.routes[i].sysid == (uint8_t) tgt_sys && (broadcast_component || s_routing.routes[i].compid == (uint8_t) tgt_comp))
        {
            // 精确命中目标组件所在链路.
            should_forward = true;
        }

        if (should_forward)
        {
            routing_send_to_channel(chan, msg);
            sent_to_chan[chan] = true;
            forwarded = true;
        }
    }

    // 5) 消费策略:
    //    - 命中外设的精确转发 (非广播) -> 消费, 防止 FC 抢答/误执行.
    //    - 广播 -> 不消费, 交 FC 处理.
    //    - 没匹配到任何链路 -> 不消费, 交 FC 处理.
    if (forwarded && !broadcast_system)
        return true;
    return false;
}

/* ===================== 命令下发到免驱相机/云台 ===================== */

// 判定某条路由是否是"相机/云台类"组件, 用于 SendCmdMsgToAllCamera 选择目标.
// 判定方式 (满足任一即认为是):
//   - 心跳 type 为 CAM-GIMBAL 系列 (CAMERA / GIMBAL);
//   - compid 落在 MAVLink 标准相机/云台组件范围内 (GIMBAL2..GIMBAL6 等).
static bool route_is_camera_or_gimbal(const MAVLink_Route &r)
{
    if (r.compid >= 100 && r.compid <= 105) // MAV_COMP_ID_CAMERA1 - MAV_COMP_ID_CAMERA6
        return true;
    if (r.compid == MAV_COMP_ID_GIMBAL)
        return true;
    if (r.compid >= MAV_COMP_ID_GIMBAL2 && r.compid <= MAV_COMP_ID_GIMBAL6)
        return true;
    return false;
}

// 把 CmdMsg 打包为 COMMAND_LONG 并通过指定 CommuPort 发出.
// 仅做发送, 不做目标匹配判定 (调用方负责). 成功返回 true.
static bool send_cmdmsg_as_command_long(uint8_t chan, const CmdMsg *msg, uint8_t target_sys, uint8_t target_comp)
{
    const Port *port = get_CommuPort(chan);
    if (port == 0 || port->write == 0 || port->lock == 0 || port->unlock == 0)
        return false;
    mavlink_message_t out;
    mavlink_msg_command_long_pack_chan(get_CommulinkSysId(), // 飞控作为命令源
                                       get_CommulinkCompId(), chan, &out,
                                       target_sys,  // 目标系统: 路由表里该组件的 sysid
                                       target_comp, // 目标组件: 路由表里该组件的 compid
                                       msg->cmd,    // MAV_CMD_xxx
                                       0,           // confirmation
                                       msg->params[0], msg->params[1], msg->params[2], msg->params[3], msg->params[4], msg->params[5], msg->params[6]);
    if (mavlink_lock_chan(chan, 0.01) == false)
        return false;
    mavlink_send_msg(port, &out, 0, 0.01);
    mavlink_unlock_chan(chan);
    return true;
}

bool mavlink_routing_send_command_to_gimbals(const CmdMsg *msg, bool onlyCameraLike)
{
    // 未使能路由功能: 不往免驱云台发命令.
    if (!s_routing_enabled)
        return false;
    if (msg == 0)
        return false;
    bool sent = false;
    bool sent_to_chan[MAVLINK_COMM_NUM_BUFFERS];
    memset(sent_to_chan, 0, sizeof(sent_to_chan));

    for (uint8_t i = 0; i < s_routing.num_routes; ++i)
    {
        const MAVLink_Route &r = s_routing.routes[i];
        if (onlyCameraLike && !route_is_camera_or_gimbal(r))
            continue;
        if (r.channel >= MAVLINK_COMM_NUM_BUFFERS)
            continue;
        if (sent_to_chan[r.channel])
            continue;
        // 同一链路只发一次, 避免一次拍照命令催生多条重复报文.
        if (send_cmdmsg_as_command_long(r.channel, msg, r.sysid, r.compid))
        {
            sent_to_chan[r.channel] = true;
            sent = true;
        }
    }
    return sent;
}
