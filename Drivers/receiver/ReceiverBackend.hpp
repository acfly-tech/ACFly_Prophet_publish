#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"
#include <stdbool.h>

/**
 * @brief 注册接收机
 * @param name 接收机名称
 * @param rc_type 接收机类型
 * @param prio 初始优先级
 * @param initialCfg 预校准信息
 */
bool ReceiverRegister(SName name, RC_CLASS rc_type, uint8_t prio = 0, const RC_CONFIG *initialCfg = 0);

/**
 * @brief 更新接收机数据
 * @param name 接收机名称
 * @param raw_data 接收机原始通道数据
 * @param channel_count 接收机通道数
 */
bool ReceiverUpdate(SName name, bool connected, float raw_data[16], uint8_t channel_count, double TIMEOUT);