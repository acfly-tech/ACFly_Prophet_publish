#include "drv_SDMMC.hpp"
#include "Basic.hpp"
#include "fatfs.h"

#include "semphr.h"
#include "stream_buffer.h"

SD_HandleTypeDef hsd1;
// SD卡读写访问互斥锁
static SemaphoreHandle_t SDSemphr;

extern "C" uint8_t Lock_SD(double TIMEOUT)
{
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive(SDSemphr, TIMEOUT_Ticks))
        return 1;
    return 0;
}

extern "C" void UnLock_SD(void)
{
    xSemaphoreGiveRecursive(SDSemphr);
}

/* 卷级互斥锁：独立信号量，仅用于 f_mount 串行化 */
static SemaphoreHandle_t SDVolumeSemphr;

uint8_t Lock_SDVolume(double TIMEOUT)
{
    if (SDVolumeSemphr == 0)
        return 0;
    TickType_t TIMEOUT_Ticks;
    if (TIMEOUT >= 0)
        TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
    else
        TIMEOUT_Ticks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive(SDVolumeSemphr, TIMEOUT_Ticks))
        return 1;
    return 0;
}

void UnLock_SDVolume(void)
{
    if (SDVolumeSemphr != 0)
        xSemaphoreGiveRecursive(SDVolumeSemphr);
}

static void MX_SDMMC1_SD_Init(void)
{
    hsd1.Instance = SDMMC1;
    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd1.Init.ClockDiv = 4;
}

void init_drv_SDMMC()
{
    uint8_t retSD;

    MX_SDMMC1_SD_Init();
    HAL_SD_DeInit(&hsd1);
    os_delay(0.01);

    // 开启SD卡检测引脚(PA15)
    // RCC->AHB4ENR |= (1 << 0);
    // set_register(GPIOA->MODER, 0b00, 30, 2);
    // GPIOA->OTYPER |= (1 << 15);
    // set_register(GPIOA->PUPDR, 1, 30, 2);

    SDSemphr = xSemaphoreCreateRecursiveMutex();
    SDVolumeSemphr = xSemaphoreCreateRecursiveMutex();

    if (BSP_SD_Init() != MSD_OK)
        while (1)
            ;

    SD_initialize(DEV_SD);
    retSD = FATFS_LinkDriverEx(&SD_Driver, 0, DEV_SD_INT);
    retSD = FATFS_LinkDriverEx(&SD_Driver, 0, DEV_SD_USR);
    retSD = FATFS_LinkDriverEx(&SD_Driver, 0, DEV_SD_EXT);

    //	SDPath[0] = DEV_SD_EXT + '0';
    //    SDPath[1] = ':';
    //    SDPath[2] = '/';
    //    SDPath[3] = 0;
}

extern "C" void SDMMC1_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd1);
}
