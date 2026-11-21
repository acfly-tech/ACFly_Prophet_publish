#pragma once

#include "Basic.hpp"
#include "vector3.hpp"

#define MAX_PRECLAND_SENSORS 3

// 精准降落信息
struct PrecLandSensorInf
{
    // 延时(s)
    float delay[2];
    /*降落偏移坐标系
        0-前左上 通过yaw信息计算前向方向
        1-东北天
*/
    uint32_t ofsType[2];
    // 降落偏移(cm)
    float ofs_x[2];
    float ofs_y[2];
    float ofs_z[2];
    // 传感器在机体的偏移(cm)
    float pofs_x[2];
    float pofs_y[2];
    float pofs_z[2];
} __ALIGNED_PACKED;
struct PrecLandInf
{
    // 控制参数
    float ctrlKp[2];
    float ctrlKi[2];

    // 最大静态允许误差
    float maxStaticErrRaw[2];
    float maxStaticErrCtrl[2];
    // 最大动态允许误差
    float maxDynamicErrRaw[2];
    float maxDynamicErrCtrl[2];

    // 安装倾角补偿
    float pitch_offset[2];
    float roll_offset[2];

    // 传感器参数
    PrecLandSensorInf sensorInfs[MAX_PRECLAND_SENSORS];
} __ALIGNED_PACKED;

struct precLandSensor
{
    // id
    uint8_t id;

    // 精准降落位置(东北天)
    vector3<double> pos;
    // 延迟时间(s)
    double delay;
    // 数据更新时间
    TIME updateTime;
    // 传感器是否可用
    bool available;
    // z轴数据是否可用
    bool z_available;

    // 航向角(rad)
    double yaw;
    // 航向角是否可用
    bool yaw_available;
};

/*
    注册精准降落传感器
      id: 传感器id
    返回值：
      非0:添加成功 返回key
      0:添加失败（已有传感器或内存不足）
*/
uint32_t precLandSensor_register(uint8_t id, double TIMEOUT = -1);
/*
    取消注册精准降落传感器
      id: 传感器id
      key: 传感器密钥

    返回值：
    true:移除成功
    false:移除失败
*/
bool precLandSensor_unRegister(uint8_t id, uint32_t key, double TIMEOUT = -1);

/*
    获取当前精准降落传感器
      res_sensor: 返回的传感器

    返回值：
    true:成功
    false:失败
*/
bool precLandSensor_get(precLandSensor *res_sensor, double TIMEOUT = -1);

/*
    获取指定精准降落传感器
      id: 传感器id
      res_sensor: 返回的传感器

    返回值：
    true:成功
    false:失败
*/
bool precLandSensor_get(uint8_t id, precLandSensor *res_sensor, double TIMEOUT = -1);

/*
    更新精准降落传感器
      id: 传感器id
      key: 传感器密钥
      pos: 降落点相对于当前位置的东北天距离(cm)
      yaw: 航向角(rad 东0 北pi/2)
      available: 传感器是否可用
      z_available: 传感器z轴是否可用
      yaw_available: 传感器yaw是否可用

    返回值：
    true:成功
    false:失败
*/
bool precLandSensor_updateENU(uint8_t id, uint32_t key, vector3<double> pos, double yaw, bool available, bool z_available, bool yaw_available, double TIMEOUT = -1);