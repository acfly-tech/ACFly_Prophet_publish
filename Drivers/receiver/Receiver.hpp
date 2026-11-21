#pragma once

#include "Basic.hpp"
#include <stdbool.h>

// 接收机最大原始通道数
#define RC_MAX_RAW_CHANNELS 16
// 接收机最大校准通道数
#define RC_MAX_CHANNELS 8

// 接收机配置定义
struct RC_CONFIG
{
    uint8_t reflections[RC_MAX_CHANNELS];
    float minRcs[RC_MAX_CHANNELS];
    float scales[RC_MAX_CHANNELS];
} __AC_PACKED;

// 接收机通用配置
struct RC_COMMON_CONFIG
{
    // 配置选项开关
    // bit0: 根据优先接收机类型选择接收机
    // bit1: 摇杆动作进行同优先级切换
    uint32_t configs[2];
} __AC_PACKED;
// bit0: 根据优先接收机类型选择接收机
#define RC_COMMON_CONFIG_SW_PRIO (1 << 0)
// bit1: 摇杆动作进行同优先级切换
#define RC_COMMON_CONFIG_SW_ACT (1 << 1)
// bit8: 根据优先接收机类型选择远程接收机
#define RMRC_COMMON_CONFIG_SW_PRIO (1 << 8)

// 单体接收机配置
struct RC_IND_CONFIG
{
    // 优先级
    uint8_t prio[8];
    // 切换接收机通道
    uint8_t sw_chan[8];
    // 需要切换到的接收机序号
    uint8_t sw_rcInd[8];
} __AC_PACKED;

// 接收机种类
enum class RC_CLASS : uint8_t
{
    MainPhysicalRc = 0,
    SubPhysicalRc,
    MainRemoteRc,
    SubRemoteRc,

    END,
};

// 接收机定义
struct Receiver
{
    // 是否已连接
    bool connected;
    // 是否可用
    bool available;
    // 可用的通道数目
    uint8_t available_channels;
    // 原始数据可用的通道数目
    uint8_t raw_available_channels;
    // 上次更新时间
    TIME last_update_time;
    // 更新时间间隔
    double update_time;

    // 摇杆类型
    RC_CLASS rc_type;
    // 摇杆id
    uint8_t id;
    // 已预校准(不能校准)
    bool preCalibrated;

    // 原始数据
    float raw_data[RC_MAX_RAW_CHANNELS];
    // 校准后的数据
    float data[RC_MAX_CHANNELS];
};
/**
 * @brief 获取当前可用接收机
 * @param rc 获取的接收机
 * @param name 获取到的接收机名称
 * @param TIMEOUT 超时时间
 */
bool getReceiver(Receiver *rc, SName *name = 0, double TIMEOUT = -1);

/**
 * @brief 获取当前可用远程接收机（仅通过优先级切换）
 * @param rc 获取的接收机
 * @param name 获取到的接收机名称
 * @param TIMEOUT 超时时间
 */
bool getRemoteReceiver(Receiver *rc, SName *name = 0, double TIMEOUT = -1);

/**
 * @brief 获取指定接收机
 * @param name 接收机名称
 * @param rc 获取的接收机
 * @param TIMEOUT 超时时间
 */
bool getReceiver(SName name, Receiver *rc, double TIMEOUT = -1);