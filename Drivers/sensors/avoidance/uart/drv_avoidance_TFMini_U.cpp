#include "drv_avoidance_TFMini_U.hpp"

#include "Avoidance.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "SensorsBackend.hpp"
#include "basic.hpp"
#include "task.h"

#define SensorInd 2

struct DriverInfo
{
    uint32_t param;
    Port port;
    uint32_t sensor_key;
};

struct _TfMini
{
    uint16_t Dist;     // Dist距离（30-1200cm）
    uint16_t Strength; // Strength信号强度（20-2000可信）
    uint8_t Mode;      // Mode测距档位
    uint8_t Rsv;       // 预留
} __attribute__((packed));
static const unsigned char packet_ID[2] = {0x59, 0x59};

static void avoidance_TFMini_U_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    /*状态机*/
    _TfMini SensorD;
    unsigned char rc_counter = 0;
    unsigned char sum = 0;
    /*状态机*/

    while (1)
    {
        uint8_t rdata;
        if (driver_info.port.read(&rdata, 1, 2, 0.5))
        {
            if (rc_counter == 0)
                sum = 0;
            if (rc_counter < 2)
            {
                // 接收包头
                if (rdata != packet_ID[rc_counter])
                    rc_counter = 0;
                else
                {
                    sum += rdata;
                    ++rc_counter;
                }
            }
            else if (rc_counter < 8)
            { // 接收数据
                ((unsigned char *) &SensorD)[rc_counter - 2] = rdata;
                sum += (unsigned char) rdata;
                ++rc_counter;
            }
            else
            { // 校验
                if (sum == rdata)
                { // 校验成功
                    if (SensorD.Strength > 30 && SensorD.Dist > 1 && SensorD.Dist < 5000)
                    {
                        vector2<double> dis;
                        dis.x = SensorD.Dist;
                        set_AvTargetXYStraightLine_RelativeFlu(driver_info.sensor_key, dis);
                    }
                    else
                        set_AvTargetInavailable(driver_info.sensor_key);
                }
                rc_counter = 0;
            }
        }
        else
        { // 一段时间无数据失能
            set_AvTargetInavailable(driver_info.sensor_key);
        }
    }
}

static bool avoidance_TFMini_U_DriverInit(Port port, uint32_t param)
{
    // 波特率115200
    port.SetBaudRate(115200, 2, 2);
    // 注册传感器
    int8_t sensor_key = registere_AvTarget();
    if (sensor_key < 0)
        return false;
    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    driver_info->sensor_key = sensor_key;
    xTaskCreate(avoidance_TFMini_U_Server, "avoidance_TFMini_U", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_avoidance_TFMini_U()
{
    PortFunc_Register(82, avoidance_TFMini_U_DriverInit);
}