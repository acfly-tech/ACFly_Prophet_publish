#include "MS_Main.hpp"
#include "HardWareConfig.hpp"

#include "Basic.hpp"
#include "ControlSystem.hpp"
#include "MS_Attitude.hpp"
#include "MS_Position.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "Sensors.hpp"
#include "StorageSystem.hpp"

/*解算参数*/
MS_OFFSET msCfg;
Quaternion msOffsetQuat(1, 0, 0, 0);
const MS_OFFSET *get_msCfg()
{
    return &msCfg;
}
const Quaternion *get_msOffsetQuat()
{
    return &msOffsetQuat;
}
/*解算参数*/

/*IMU更新时间定义*/
double IMU_Accelerometer_UpdateT[IMU_Sensors_Count];
double IMU_Accelerometer_UpdateFreq[IMU_Sensors_Count];
void set_IMU_Accelerometer_UpdateFreq(uint8_t ind, double Freq)
{
    IMU_Accelerometer_UpdateT[ind] = 1.0 / Freq;
    IMU_Accelerometer_UpdateFreq[ind] = Freq;
}

double IMU_Gyroscope_UpdateT[IMU_Sensors_Count];
double IMU_Gyroscope_UpdateFreq[IMU_Sensors_Count];
void set_IMU_Gyroscope_UpdateFreq(uint8_t ind, double Freq)
{
    IMU_Gyroscope_UpdateT[ind] = 1.0 / Freq;
    IMU_Gyroscope_UpdateFreq[ind] = Freq;
}
/*IMU更新时间定义*/

/*IMU（陀螺加速度）更新通知*/
static TaskHandle_t MS_Main_GyroTaskHandle = 0;
static TaskHandle_t MS_Main_TaskHandle = 0;

// 陀螺更新通知
bool MS_Notify_IMUGyroUpdate(uint8_t ind)
{
    if (ind >= IMU_Sensors_Count)
        return false;
    if (MS_Main_GyroTaskHandle == 0)
        return false;
    xTaskNotify(MS_Main_GyroTaskHandle, 1 << ind, eSetBits);
    return true;
}
bool MS_Notify_IMUGyroUpdateFromISR(uint8_t ind, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (ind >= IMU_Sensors_Count)
        return false;
    if (MS_Main_GyroTaskHandle == 0)
        return false;
    xTaskNotifyFromISR(MS_Main_GyroTaskHandle, 1 << ind, eSetBits, pxHigherPriorityTaskWoken);
    return true;
}

// 加速度更新通知
bool MS_Notify_IMUAccelUpdate(uint8_t ind)
{
    if (ind >= IMU_Sensors_Count)
        return false;
    if (MS_Main_TaskHandle == 0)
        return false;
    xTaskNotify(MS_Main_TaskHandle, 1 << ind, eSetBits);
    return true;
}
bool MS_Notify_IMUAccelUpdateFromISR(uint8_t ind, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (ind >= IMU_Sensors_Count)
        return false;
    if (MS_Main_TaskHandle == 0)
        return false;
    xTaskNotifyFromISR(MS_Main_TaskHandle, 1 << ind, eSetBits, pxHigherPriorityTaskWoken);
    return true;
}
/*IMU（陀螺加速度）更新通知*/

void MS_Main_Gyro(void *pvParameters)
{
    // 等待初始化完成
    while (getInitializationCompleted() == false)
        os_delay(0.1);

    // 读取解算参数
    ReadParamGroup("MS_OFFSET", (uint64_t *) &msCfg, 0, -1);
    msOffsetQuat = Quaternion(1, 0, 0, 0);
    vector3<double> offsetAng(degree2rad(-msCfg.rol_offset[0]), degree2rad(-msCfg.pit_offset[0]), 0);
    msOffsetQuat.rotate_delta_angle(offsetAng);

    while (1)
    {
        // 等待陀螺数据
        uint32_t notify_value;
        xTaskNotifyWait(pdFALSE, pdTRUE, &notify_value, portMAX_DELAY);
        for (uint8_t i = 0; i < IMU_Sensors_Count; ++i)
        {
            if (notify_value & (1 << i))
            {
                // 陀螺数据更新
                // 处理陀螺数据
                IMU_Sensor sensor;
                if (GetGyroscope(i, &sensor, 0.01))
                {
                    MS_Attitude_GyroIntegral(i, sensor);
                }
            }
        }
    }
}

void MS_Main(void *pvParameters)
{
    // 等待初始化完成
    while (getInitializationCompleted() == false)
        os_delay(0.1);

    uint8_t SDLog_Att_counter = 0;
    uint8_t SDLog_LocalNed_counter = 0;
    while (1)
    {
        // 等待加速度数据
        uint32_t notify_value;
        xTaskNotifyWait(pdFALSE, pdTRUE, &notify_value, portMAX_DELAY);
        for (uint8_t i = 0; i < IMU_Sensors_Count; ++i)
        {
            if (notify_value & (1 << i))
            {
                // 加速度数据更新
                // 处理加速度数据
                IMU_Sensor sensor;
                if (GetAccelerometer(i, &sensor, 0.01))
                {
                    MS_Attitude(i, sensor);
                    MS_Position(i, sensor);
                }

                /*更新记录数据Log*/
                bool unLocked;
                is_Attitude_Control_Enabled(&unLocked);
                if (unLocked && i == 0)
                {
                    uint64_t log;

                    if (ReadParam("SDLog_Att", 0, 0, (uint64_t *) &log, 0) == PR_OK)
                    { // 记录姿态
                        log &= 0xffffffff;
                        if (log)
                        {
                            if (++SDLog_Att_counter >= log)
                            {
                                SDLog_Att_counter = 0;
                                SDLog_Msg_AttitudeQuaternion();
                            }
                        }
                    }

                    if (ReadParam("SDLog_LocalNed", 0, 0, (uint64_t *) &log, 0) == PR_OK)
                    { // 记录位置速度
                        log &= 0xffffffff;
                        if (log)
                        {
                            if (++SDLog_LocalNed_counter >= log)
                            {
                                SDLog_LocalNed_counter = 0;
                                SDLog_Msg_LocalPositionNed();
                            }
                        }
                    }
                }
                /*更新记录数据Log*/
            }
        }
    }
}

#define MS_Gyro_StackSize 800
Static_DTCMBuf __attribute__((aligned(8))) StackType_t MS_Gyro_Stack[MS_Gyro_StackSize];
Static_DTCMBuf StaticTask_t MS_Gyro_TaskBuffer;

#define MS_Main_StackSize 3200
Static_DTCMBuf __attribute__((aligned(8))) StackType_t MS_Main_Stack[MS_Main_StackSize];
Static_DTCMBuf StaticTask_t MS_Main_TaskBuffer;

void init_MS_Main()
{
    init_MS_Attitude();
    init_MS_Position();

    /*参数注册*/
    // 水平校准四元数参数
    AirframeCalibQuat initial_AirframeCalibQuat = {1, 0, 0, 0};
    MAV_PARAM_TYPE AirframeCalib_param_types[] = {MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64};
    ParamGroupRegister("AirframeCalib", 1, 2, AirframeCalib_param_types, 0, (uint64_t *) &initial_AirframeCalibQuat);

    // 注册码参数
    MAV_PARAM_TYPE WGA_param_types[] = {MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64};
    ParamGroupRegister("WGA_Code", 2, 2, WGA_param_types, 0, 0);

    msCfg.rol_offset[0] = 0;
    msCfg.pit_offset[0] = 0;
    msCfg.acc_td4p[0] = 0.5f;
    msCfg.mag_td4p[0] = 0.5f;
    msCfg.acc_corr_p[0] = 2.0f;
    msCfg.acc_corr_i[0] = 0.01f;
    msCfg.acc_err_max[0] = 0.45f;
    msCfg.gyr_err_max[0] = 0.1f;
    msCfg.xy_pos_fil_p[0] = 7.0f;
    msCfg.z_pos_fil_p[0] = 7.0f;
    msCfg.xy_vel_fil_p[0] = 7.0f;
    msCfg.z_vel_fil_p[0] = 7.0f;
    msCfg.z_s_kp[0] = 0.5f;
    msCfg.z_v_kp[0] = 0.5f;
    msCfg.xy_s_kp[0] = 1.5f;
    msCfg.xy_v_kp[0] = 3.0f;
    MAV_PARAM_TYPE msOffset_param_types[] = {
        MAV_PARAM_TYPE_REAL32, // rol_offset
        MAV_PARAM_TYPE_REAL32, // pit_offset
        MAV_PARAM_TYPE_REAL32, // acc_td4p
        MAV_PARAM_TYPE_REAL32, // mag_td4p
        MAV_PARAM_TYPE_REAL32, // acc_corr_p
        MAV_PARAM_TYPE_REAL32, // acc_corr_i
        MAV_PARAM_TYPE_REAL32, // acc_err_max
        MAV_PARAM_TYPE_REAL32, // gyr_err_max
        MAV_PARAM_TYPE_REAL32, // xy_pos_fil_p
        MAV_PARAM_TYPE_REAL32, // z_pos_fil_p
        MAV_PARAM_TYPE_REAL32, // xy_vel_fil_p
        MAV_PARAM_TYPE_REAL32, // z_vel_fil_p
        MAV_PARAM_TYPE_REAL32, // z_s_kp
        MAV_PARAM_TYPE_REAL32, // z_v_kp
        MAV_PARAM_TYPE_REAL32, // xy_s_kp
        MAV_PARAM_TYPE_REAL32, // xy_v_kp
    };
    SName msOffset_param_names[] = {
        "MS_OffsetRol", "MS_OffsetPit", "MS_AccTD4P",   "MS_MagTD4P",  "MS_AccCorrP", "MS_AccCorrI", "MS_AccErrMax", "MS_GyrErrMax",
        "MS_XYPosFilP", "MS_ZPosFilP",  "MS_XYVelFilP", "MS_ZVelFilP", "MS_ZSKp",     "MS_ZVKp",     "MS_XYSKp",     "MS_XYVKp",
    };
    InitialParametersInit((uint64_t *) &msCfg, msOffset_param_types, msOffset_param_names, sizeof(msCfg) / 8);
    ParamGroupRegister("MS_OFFSET", 1, sizeof(msCfg) / 8, msOffset_param_types, msOffset_param_names, (uint64_t *) &msCfg);
    /*参数注册*/

    MS_Main_GyroTaskHandle = xTaskCreateStatic(MS_Main_Gyro, "MS_Main_Gyro", MS_Gyro_StackSize, NULL, SysPriority_MeasurementSystem_Integral, MS_Gyro_Stack, &MS_Gyro_TaskBuffer);
    MS_Main_TaskHandle = xTaskCreateStatic(MS_Main, "MS_Main", MS_Main_StackSize, NULL, SysPriority_MeasurementSystem, MS_Main_Stack, &MS_Main_TaskBuffer);
}