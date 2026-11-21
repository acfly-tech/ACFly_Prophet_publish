#include "drv_msp_DJI_FPV.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "StorageSystem.hpp"
#include "task.h"

struct DriverInfo
{
    uint32_t param;
    Port port;
};

static void MSP_DJI_FPV_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    while (1)
    {
        os_delay(0.1);
    }
}

static bool MSP_DJI_FPV_DriverInit(Port port, uint32_t param)
{
    port.SetBaudRate(115200, 2, 2);

    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(MSP_DJI_FPV_Server, "MSP_DJI_FPV", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_msp_DJI_FPV()
{
    PortFunc_Register(172, MSP_DJI_FPV_DriverInit);
}