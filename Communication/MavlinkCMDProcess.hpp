#pragma once

#include "Basic.hpp"
#include "mavlink.h"

#define CAMERA_FEEDBACK_QUEUE_ID 60000
#define MAX_CMD_QUEUE 20
struct message_queue_struct
{
    uint16_t componentID;
    QueueHandle_t message_queue;
};

struct CmdMsg
{
    uint8_t port_id;
    uint16_t target_system;
    uint16_t target_component;
    uint32_t cmd;
    uint8_t cmdSourceType;
    bool need_ack;
    double params[16];
};

/**
 * @brief 获取云台相机的CompID (MAVLINK协议)
 * @note  协议: https://docs.qq.com/doc/DT1RpdHhxdFpFZm1W
 * @param compID 返回的可用的相机ID，范围[MAV_COMP_ID_GIMBAL2(171) - MAV_COMP_ID_GIMBAL6(176)], 地面站会判断此范围的相机为MAVLINK协议相机，显示完整的控制界面
 * @param name ID 相机名称(可填空)
 * @return true 成功; false 资源已满
 */
bool registerMavlinkGimbalCompID(uint16_t &compID, SName name);

/* 注册进程消息队列
        queueID: 消息队列唯一ID
        message_queue_size: 消息队列中最大可容纳的消息数
        返回值：
                true: 注册成功
                false: 注册失败
*/
bool TaskQueueRegister(uint16_t queueID, uint16_t message_queue_size);

/* 发送消息到对应的queueID的消息队列
        msg: 要发送的消息
        queueID: 消息队列唯一ID
        TIMEOUT: 发送等待时间
        返回值：
                true: 发送成功
                false: 发送失败
*/
bool SendCmdMsgToTask(CmdMsg msg, uint16_t queueID, double TIMEOUT);

// 发送消息到首个数字相机的消息队列
bool SendCmdMsgToSingleCamera(CmdMsg msg, double TIMEOUT);

// 发送消息到所有数字相机的消息队列
bool SendCmdMsgToAllCamera(CmdMsg msg, double TIMEOUT);

/* 从指定queueID消息队列获取消息
        msg: 返回的消息
        queueID: 消息队列唯一ID
        TIMEOUT: 接收等待时间
        返回值：
                true: 接收到消息
                false: 无接收到消息
*/
bool ReceiveCmdMsgFromTask(CmdMsg *msg, uint16_t queueID, double TIMEOUT);

/**
 * @brief 激活/取消激活无人机
 * @note  对应参数ID_regStatus, [0: 未激活(生产模式,允许飞行)],  [1: 未激活(不允许飞行)],  [2: 已激活(允许飞行)],  [3: 已注销激活(不允许飞行)]
 * @param port_index
 * @param msg
 * @return void
 */
void Cmd21921_MAV_CMD_ACTIVE_VEHICLE_REQUEST(uint8_t port_index, const mavlink_message_t *msg);

// 发送版本信息
void send_AutoPilot_Version(uint8_t port_index);
// Mavlink命令处理函数表
extern void (*const Mavlink_CMD_Process[])(uint8_t port_index, const mavlink_message_t *msg_rd);
// Mavlink命令处理函数个数
extern const uint16_t Mavlink_CMD_Process_Count;
