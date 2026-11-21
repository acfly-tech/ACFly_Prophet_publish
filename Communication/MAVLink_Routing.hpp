#pragma once

#include "mavlink.h"

/*
 * 通用 MAVLink 路由接口 (静态单例方式).
 *
 * 详细说明见 MAVLink_Routing.cpp 顶部注释. 只操作 CommuPort.
 *
 * 调用方 (Commulink 任务) 在每条消息解包成功后调用本接口:
 *   in_channel: 收到该消息的 CommuPort 索引.
 *
 * 返回 true : 该消息已被路由模块当作"给外设/GCS 透传"消费掉, 调用方应跳过
 *             FC 本地处理 (避免 FC 抢答 COMMAND_ACK 或误执行外设命令).
 * 返回 false: 该消息未被路由消费, 调用方按原逻辑 (Mavlink_RC_Process / RID 等) 处理.
 */
bool mavlink_routing_handle_message(uint8_t in_channel, const mavlink_message_t *msg);

/*
 * 设置路由功能使能状态.
 * 由 Commulink 任务在启动时根据 Lk_CommuCfg 的 CommuCfg_mavlinkRoute_BIT 调用.
 * 未使能时所有路由接口直接返回失败, 不影响 FC 原有处理逻辑.
 */
void mavlink_routing_set_enabled(bool ena);

/*
 * 判断某条 CommuPort 是否接的是地面站 (GCS).
 * 依据: 该口曾经收到过 type == MAV_TYPE_GCS 的心跳.
 * 供集群从机->GCS 转发时过滤目标口 (只发给接 GCS 的口).
 * 该判定独立于路由使能开关, 始终可用.
 */
bool mavlink_routing_is_gcs_port(uint8_t chan);

struct CmdMsg;
/*
 * 把一条相机/云台命令 (CmdMsg) 打包成 COMMAND_LONG, 发给路由表里所有
 * "免驱直连的 MAVLink 相机/云台" 所在的链路.
 *
 * 用途: 航线中触发拍照时, 除原"本地注册驱动队列"投递外, 还需让未注册本地
 * 驱动、由通用路由转发心跳引入的 MAVLink 云台也能收到该命令.
 *
 *   msg          : 待发送的命令 (取 msg.cmd 作为 COMMAND_LONG.command,
 *                  msg.params[0..6] 作为 param1..7).
 *   onlyCameraLike: true = 仅发给心跳 type 为 CAMERA/GIMBAL 的组件;
 *                   false = 发给路由表里所有组件.
 * 返回 true : 至少有一条命令被成功打包发出.
 * 返回 false: 路由表里无匹配组件, 或全部发送失败.
 */
bool mavlink_routing_send_command_to_gimbals(const CmdMsg *msg, bool onlyCameraLike);
