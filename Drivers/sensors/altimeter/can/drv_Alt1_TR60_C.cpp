#include "drv_Alt1_TR60_C.hpp"
#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "SensorsBackend.hpp"
#include "drv_CAN.hpp"
#include "task.h"

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
};

static void Alt1_TR60_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    // 状态信息
    uint8_t current_packet = 0;
    uint8_t packets_count = 0;

    while (1)
    {
        CanPacket mail;
        if (driver_info.mail_box->receiveMail(&mail, 2))
        {
            if (mail.Identifier == 0x501)
            { // 目标信息包
                struct target_info_packet
                {
                    // 分区个数
                    uint8_t sectors_count;
                    // 数据包数量
                    uint8_t packets_count;
                    // 计数
                    uint8_t seq;
                    // 状态 0-正常 1-异常
                    uint8_t flag;
                    // 仿地高度(0.01m 0.5m-34m)
                    uint16_t GroundHeight;
                    // 工作类型0-5
                    uint16_t work_type;
                } __AC_PACKED;
                target_info_packet *packet = (target_info_packet *) &mail.data;
                // 更新位置
                vector3<double> position;
                position.z = packet->GroundHeight;
                // 获取倾角
                Quaternion quat;
                get_Airframe_quat(&quat);
                double lean_cosin = quat.get_lean_angle_cosin();
                // 更新
                position.z *= lean_cosin;
                PositionSensorUpdatePosition(default_mmWaveRadarAH_sensor_index, driver_info.sensor_key, position, true);
            }
            else if (mail.Identifier == 0x502)
            { // 数据包
                struct target_data_packet
                {
                    // 分区id
                    uint8_t sectors_id;
                    // 目标反射截面积 dBM2
                    uint8_t RCS;
                    // 距离(0.01m 0.5m-34m)
                    uint16_t distance;
                    // 角度(0.01° -100-100)
                    uint16_t angle;
                    // 保留
                    uint16_t rsv;
                } __AC_PACKED;
                target_data_packet *packet = (target_data_packet *) &mail.data;
            }
        }
        else
        {
            PositionSensorSetInavailable(default_mmWaveRadarAH_sensor_index, driver_info.sensor_key);
        }
    }
}

static bool Alt1_TR60_DriverInit()
{
    return true;
}
static bool Alt1_TR60_DriverRun(CanMailBox *mailBox)
{
    CanId Ids[2];
    Ids[0].Identifier = 0x501;
    Ids[0].IdType = 0;
    Ids[1].Identifier = 0x502;
    Ids[1].IdType = 0;
    mailBox->init(5, Ids, 2);
    // 注册传感器
    uint32_t sensor_key =
        PositionSensorRegister(default_mmWaveRadarAH_sensor_index, "TR60", Position_Sensor_Type_RangePositioning, Position_Sensor_DataType_s_z, Position_Sensor_frame_ENU, 0.05, // 延时
                               0,                                                                                                                                                // xy信任度
                               0                                                                                                                                                 // z信任度
        );
    if (sensor_key == 0)
        return false;
    DriverInfo *driver_info = new DriverInfo;
    driver_info->mail_box = mailBox;
    driver_info->sensor_key = sensor_key;
    xTaskCreate(Alt1_TR60_Server, "Alt1_TR60", 800, (void *) driver_info, SysPriority_UserTask, NULL);
    return true;
}

void init_drv_Alt1_TR60_C()
{
    CanFunc_Register(68, Alt1_TR60_DriverInit, Alt1_TR60_DriverRun);
}