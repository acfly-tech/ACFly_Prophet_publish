#include "drv_Alt1_NRA24_C.hpp"
#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "SensorsBackend.hpp"
#include "drv_CAN.hpp"
#include "task.h"

// Baud:500k

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
};

static void CAN_NRA24_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    // 获取避障参数
    const AvoidanceCfg *avCfg = getAvCfg();

    /*状态机*/
    uint8_t targets_count = {0};
    uint8_t current_target = {0};
#define resetState                                                                                                                                                                                     \
    {                                                                                                                                                                                                  \
        targets_count = 0;                                                                                                                                                                             \
        current_target = 0;                                                                                                                                                                            \
    }

#define maxTargetsCount 64
    float vertical_distance[maxTargetsCount];
    /*状态机*/

    while (1)
    {
        CanPacket mail;
        if (driver_info.mail_box->receiveMail(&mail, 2))
        {
            if (mail.Identifier == 0x70b)
            {
                targets_count = mail.data[0];
                if (targets_count > maxTargetsCount)
                    targets_count = maxTargetsCount;
                current_target = 0;
                if (targets_count == 0)
                {
                    PositionSensorSetInavailable(default_mmWaveRadarAH_sensor_index, driver_info.sensor_key);
                }
            }
            else if (mail.Identifier == 0x70c)
            {
                if (current_target < targets_count)
                {
                    uint8_t target_id = mail.data[0];
                    uint8_t sector_id = (mail.data[6] >> 3) & 0x03;

                    vertical_distance[current_target] = (mail.data[2] << 8) + (mail.data[3]);
                    ++current_target;
                }
                if (current_target >= targets_count)
                { // 目标收集完毕
                    float minDistance = -1;
                    if (current_target > 0)
                    {
                        for (uint8_t i = 0; i < current_target; ++i)
                        {
                            if (minDistance < 0 || vertical_distance[i] < minDistance)
                            {
                                minDistance = vertical_distance[i];
                            }
                        }

                        if (minDistance >= 0)
                        { // 目标有效
                            vector3<double> position;
                            position.z = minDistance;
                            // 获取倾角
                            Quaternion quat;
                            get_Airframe_quat(&quat);
                            double lean_cosin = quat.get_lean_angle_cosin();
                            // 更新
                            position.z *= lean_cosin;
                            PositionSensorUpdatePosition(default_mmWaveRadarAH_sensor_index, driver_info.sensor_key, position, true);
                        }
                    }
                    resetState
                }
            }
        }
        else
        {
            PositionSensorSetInavailable(default_mmWaveRadarAH_sensor_index, driver_info.sensor_key);
        }
    }
}

static bool CAN_NRA24_DriverInit()
{
    return true;
}
static bool CAN_NRA24_DriverRun(CanMailBox *mailBox)
{
    DriverInfo *driver_info = new DriverInfo;
    CanId Ids[2];
    Ids[0].Identifier = 0x70b;
    Ids[0].IdType = 0;
    Ids[1].Identifier = 0x70c;
    Ids[1].IdType = 0;
    mailBox->init(5, Ids, 2);

    // 注册传感器
    uint32_t sensor_key =
        PositionSensorRegister(default_mmWaveRadarAH_sensor_index, "NRA24", Position_Sensor_Type_RangePositioning, Position_Sensor_DataType_s_z, Position_Sensor_frame_ENU, 0.03, // 延时
                               0,                                                                                                                                                 // xy信任度
                               0                                                                                                                                                  // z信任度
        );
    if (sensor_key == 0)
        return false;

    driver_info->mail_box = mailBox;
    driver_info->sensor_key = sensor_key;
    xTaskCreate(CAN_NRA24_Server, "CAN_NRA24", 1500, (void *) driver_info, SysPriority_UserTask, NULL);
    return true;
}

void init_drv_Alt1_NRA24_C()
{
    CanFunc_Register(80, CAN_NRA24_DriverInit, CAN_NRA24_DriverRun);
}