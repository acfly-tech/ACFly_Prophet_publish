#pragma once

#include "Basic.h"
#include "FreeRTOS.h"
#include "quaternion.hpp"

/*参数*/
// 水平校准四元数
struct AirframeCalibQuat
{
    float q0, q1, q2, q3;
} __ALIGNED_PACKED;

// 解算参数
struct MS_OFFSET
{
    float rol_offset[2];   // slot 0:  MS_OffsetRol  安装滚转偏移 (deg)
    float pit_offset[2];   // slot 1:  MS_OffsetPit  安装俯仰偏移 (deg)
    float acc_td4p[2];     // slot 2:  MS_AccTD4P    加速度/陀螺TD4滤波参数
    float mag_td4p[2];     // slot 3:  MS_MagTD4P    磁力计TD4滤波参数
    float acc_corr_p[2];   // slot 4:  MS_AccCorrP   加速度姿态修正P
    float acc_corr_i[2];   // slot 5:  MS_AccCorrI   加速度姿态修正I
    float acc_err_max[2];  // slot 6:  MS_AccErrMax  加速度修正最大误差
    float gyr_err_max[2];  // slot 7:  MS_GyrErrMax  陀螺修正最大误差
    float xy_pos_fil_p[2]; // slot 8:  MS_XYPosFilP  XY位置误差TD4滤波参数
    float z_pos_fil_p[2];  // slot 9:  MS_ZPosFilP   Z位置误差TD4滤波参数
    float xy_vel_fil_p[2]; // slot 10: MS_XYVelFilP  XY速度误差TD4滤波参数
    float z_vel_fil_p[2];  // slot 11: MS_ZVelFilP   Z速度误差TD4滤波参数
    float z_s_kp[2];       // slot 12: MS_ZSKp       Z轴位置修正Kp系数
    float z_v_kp[2];       // slot 13: MS_ZVKp       Z轴速度修正Kp系数
    float xy_s_kp[2];      // slot 14: MS_XYSKp      XY位置修正Kp系数
    float xy_v_kp[2];      // slot 15: MS_XYVKp      XY速度修正Kp系数
} __ALIGNED_PACKED;
const MS_OFFSET *get_msCfg();
const Quaternion *get_msOffsetQuat();
/*参数*/

/*IMU更新时间定义*/
extern double IMU_Accelerometer_UpdateT[];
extern double IMU_Accelerometer_UpdateFreq[];
void set_IMU_Accelerometer_UpdateFreq(uint8_t ind, double Freq);

extern double IMU_Gyroscope_UpdateT[];
extern double IMU_Gyroscope_UpdateFreq[];
void set_IMU_Gyroscope_UpdateFreq(uint8_t ind, double Freq);
/*IMU更新时间定义*/

/*IMU（陀螺加速度）更新通知*/
// 陀螺更新通知
bool MS_Notify_IMUGyroUpdate(uint8_t ind);
bool MS_Notify_IMUGyroUpdateFromISR(uint8_t ind, BaseType_t *pxHigherPriorityTaskWoken);

// 加速度更新通知
bool MS_Notify_IMUAccelUpdate(uint8_t ind);
bool MS_Notify_IMUAccelUpdateFromISR(uint8_t ind, BaseType_t *pxHigherPriorityTaskWoken);
/*IMU（陀螺加速度）更新通知*/

void init_MS_Main();