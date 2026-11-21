#pragma once

#include "Basic.hpp"

#define MAX_FORWARDDIST_SENSORS 3

// 前向距离参数
struct ForwardDistInf
{
    // 向内控制Kp
    float inwardKp[2];
    // 向外控制Kp
    float outwardKp[2];
    // 控制误差限幅(cm)
    float ctrlErrLimit[2];
    // 最大距离(cm)
    float maxDist[2];
} __ALIGNED_PACKED;

struct forwardDistSensor
{
    // id
    uint8_t id;

    // 对墙距离(cm)
    double distance;
    // 数据更新时间
    TIME updateTime;
    // 传感器是否可用
    bool available;

    // 航向角(rad)
    double yaw;
    // 航向角是否可用
    bool yaw_available;
};

/*
    注册前向距离传感器
      id: 返回的传感器id指针
    返回值：
      非0:添加成功 返回key
      0:添加失败（无空位或内存不足）
*/
uint32_t forwardDistSensor_register(uint8_t *id, double TIMEOUT = -1);

/*
    取消注册前向距离传感器
      id: 传感器id
      key: 传感器密钥

    返回值：
    true:移除成功
    false:移除失败
*/
bool forwardDistSensor_unRegister(uint8_t id, uint32_t key, double TIMEOUT = -1);

/*
    获取最小前向距离
      res_distance: 返回的最小距离(cm)

    返回值：
    true:成功
    false:失败（无可用传感器）
*/
bool forwardDistSensor_getMinDistance(double *res_distance, double TIMEOUT = -1);

/*
    获取第一个可用的航向角
      res_yaw: 返回的航向角(rad)

    返回值：
    true:成功
    false:失败（无可用航向角）
*/
bool forwardDistSensor_getYaw(double *res_yaw, double TIMEOUT = -1);

/*
    更新前向距离传感器
      id: 传感器id
      key: 传感器密钥
      distance: 对墙距离(cm)
      yaw: 航向角(rad)
      available: 传感器是否可用
      yaw_available: 航向角是否可用

    返回值：
    true:成功
    false:失败
*/
bool forwardDistSensor_update(uint8_t id, uint32_t key, double distance, double yaw, bool available, bool yaw_available, double TIMEOUT = -1);

/*
    设置前向距离传感器不可用
      id: 传感器id
      key: 传感器密钥

    返回值：
    true:成功
    false:失败
*/
bool forwardDistSensor_setUnavailable(uint8_t id, uint32_t key, double TIMEOUT = -1);