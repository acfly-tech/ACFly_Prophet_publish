#pragma once

#include "mavlink.h"

enum class MRP_RESULT
{
    // 已处理
    Processed,
    // 已处理 但需要转发
    ProcessedNeedForwarding,
    // 未处理
    Unprocessed,
};

// Mavlink消息处理函数表
extern MRP_RESULT (*const Mavlink_RC_Process[])(uint8_t Port_index, const mavlink_message_t *msg_sd);
// Mavlink消息处理函数个数
extern const uint16_t Mavlink_RC_Process_Count;

// 参数更新协议任务
void paramProtocolTask(bool slowMode);
// ftp协议任务
void ftpProtocolTask();

// 解锁授权(禁飞区围栏使能时使用)
// 发送MAV_CMD_ARM_AUTHORIZATION_REQUEST解锁授权请求给地面站
// timeout: 期望的授权有效期(秒)
void sendArmAuthorizationRequest(double timeout);
// 等待地面站返回解锁授权结果(COMMAND_ACK)
// timeout: 等待超时时间(秒), 超时未收到回复返回false
// 返回true表示地面站允许解锁/起飞
bool waitArmAuthorizationResult(double timeout);