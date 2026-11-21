#pragma once

#include "basic.h"
#include <stdint.h>

// SLAM 相关配置参数组
//  sensor_datatype: msg102/msg331 注册的传感器数据维度
//                   0 - XYZ（三维位置）
//                   1 - XY （仅水平位置）
//  slam_frame: local 控制指令(msg84)坐标系重映射开关
//              0 - 禁用，LOCAL_NED/ENU 走普通 ENU 控制
//              1 - 启用，LOCAL_NED/ENU 重映射为 VISION_NED/ENU 走 SLAM 控制
//              （适配 MAV_FRAME_VISION 已弃用、需用 local 指令控制室内 SLAM 飞行的场景）
struct SlamCfg
{
    uint32_t sensor_datatype[2]; // 0=XYZ 1=XY
    uint32_t slam_frame[2];      // 0=off 1=on
} __ALIGNED_PACKED;
