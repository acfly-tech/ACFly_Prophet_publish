#include "drv_frDist_TonRainU.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "SensorsBackend.hpp"
#include "forwardDist.hpp"
#include "task.h"

struct DriverInfo
{
    uint32_t param;
    Port port;
};

static void TonRainU_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    uint8_t rc_counter = 0;
    uint16_t uRange = 0;

    // 注册避障目标
    int32_t avTarget_key = registere_AvTarget();

    // 注册前向距离传感器
    uint8_t frDist_id = 0;
    uint32_t frDist_key = forwardDistSensor_register(&frDist_id, -1);

    while (1)
    {
        uint8_t rdata;
        if (driver_info.port.read(&rdata, 1, 2, 0.8))
        {
            if (rc_counter == 0)
            {
                if (rdata == 0x24)
                    rc_counter++;
                else
                    rc_counter = 0;
            }
            else if (rc_counter == 1)
            {
                if (rdata == 0x3a)
                    rc_counter++;
                else
                    rc_counter = 0;
            }
            else if (rc_counter == 2)
            {
                rc_counter++;
                uRange = (uint16_t) rdata << 8;
            }
            else if (rc_counter == 3)
            {
                rc_counter++;
                uRange |= rdata;
            }
            else if (rc_counter == 4)
            {
                if (rdata == 0x0d)
                    rc_counter++;
                else
                    rc_counter = 0;
            }
            else if (rc_counter == 5)
            {
                if (rdata == 0x0a)
                {
                    rc_counter = 0;

                    if (uRange < 990)
                    {
                        // 更新避障目标
                        vector2<double> dis;
                        dis.x = uRange;
                        dis.y = 0;
                        set_AvTargetXYStraightLine_RelativeFlu(avTarget_key, dis);

                        // 更新前向距离传感器
                        if (frDist_key != 0)
                        {
                            forwardDistSensor_update(frDist_id, frDist_key, uRange, 0, true, false, 0.01);
                        }
                    }
                    else
                    {
                        set_AvTargetInavailable(avTarget_key);

                        // 更新前向距离传感器为不可用
                        if (frDist_key != 0)
                        {
                            forwardDistSensor_update(frDist_id, frDist_key, -1, 0, false, false, 0.01);
                        }
                    }
                }
                else
                {
                    rc_counter = 0;
                }
            }
        }
        else
        {
            set_AvTargetInavailable(avTarget_key);

            // 超时，更新前向距离传感器为不可用
            if (frDist_key != 0)
            {
                forwardDistSensor_update(frDist_id, frDist_key, -1, 0, false, false, 0.01);
            }
        }
    }
}

static bool TonRainU_DriverInit(Port port, uint32_t param)
{
    // 波特率115200
    port.SetBaudRate(115200, 2, 2);

    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(TonRainU_Server, "TonRainU", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_frDist_TonRainU()
{
    PortFunc_Register(60, TonRainU_DriverInit);
}
