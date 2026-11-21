#include "drv_Main.hpp"

#include "Commulink.hpp"
#include "HardWareConfig.hpp"
#include "SensorsBackend.hpp"
#include "dma.hpp"
#include "drv_ADC.hpp"
#include "drv_CRC.hpp"
#include "drv_Flash.hpp"
#include "drv_LED.hpp"
#include "drv_PWMOut.hpp"
#include "drv_SDMMC.hpp"
#include "drv_USB.hpp"
#include "drv_Uart.hpp"
#include "drv_can.hpp"
#include "drv_imu.hpp"
#include "drv_spi.hpp"

// i2c底层驱动
#include "drv_i2c.hpp"

#include "Parameters.hpp"
#include "StorageSystem.hpp"

// gps
#include "drv_GPS.hpp"

// 接收机
#include "drv_receiver.hpp"

// 光流
#include "drv_OpticalFlow.hpp"

// 定高
#include "drv_Altimeter.hpp"

// 精准降落
#include "drv_PrecLand.hpp"

// UWB
#include "drv_UWB.hpp"

// 前向距离
#include "drv_ForwardDist.hpp"

// 避障
#include "drv_Avoidance.hpp"

// extIO
#include "drv_extIO.hpp"

// 云台
#include "drv_Gimbal.hpp"

// LED灯
#include "drv_extLed.hpp"

// 二次开发接口
#include "drv_SDI.hpp"

// 磁力计
#include "drv_Compass.hpp"

// RID
#include "drv_RemoteID.hpp"

// 功能
#include "drv_Functions.hpp"

// OSD
#include "drv_OSD.hpp"

void init_drv_Main()
{
    // DMA初始化
    init_DMA();

    // CRC初始化
    os_delay(0.1);
    init_drv_CRC();

    // 内部存储介质初始化
    init_drv_Flash();
    // 存储外设驱动
    init_drv_SDMMC();
    init_InternalStorage();
    // 参数表初始化
    init_Parameters();

    // 执行 [fixed_pins] 段的 GPIO 固定输出 (从 InternalStorage 读取)
    HW_ApplyFixedPins();

    // 执行 [exti_pins] 段的 GPIO 外部中断配置 (从 InternalStorage 读取, 须在USB初始化之前)
    HW_ApplyExtiPins();

    // LED初始化 (在FuncMap已写入后执行，第一次开机也能正确配置)
    init_drv_LED();

    // SD存储初始化
    init_SDStorage();

    // 读取硬件版本号 (由INI [hardware] 段配置)
    extern uint16_t HW_VERSION;
    extern uint16_t HW_Revision;
    HW_GetVersion(&HW_VERSION, &HW_Revision);

    // ADC初始化
    init_drv_ADC();

    // 传感器接口初始化
    init_Sensors();

    // 初始化PWM输出
    init_drv_PWMOut();

    // 通信端口初始化
    init_drv_USB();
    init_drv_uart();
    // SPI
    init_drv_spi();
    // i2c
    init_drv_i2c();

    // 开启通信线程
    init_Commulink();

    // 内部传感器驱动
    if (HTL_isEnabled() == false)
        init_drv_imu();

    // 初始化磁罗盘
    if (HTL_isEnabled() == false)
        init_drv_Compass();

    // 初始化接收机
    init_drv_receiver();

    // 初始化gps
    init_drv_GPS();

    // 初始化光流
    init_drv_OpticalFlow();

    // 初始化高度计
    init_drv_Altimeter();

    // 初始化uwb
    init_drv_UWB();

    // 初始化精准降落
    init_drv_PrecLand();

    // 初始化前向距离
    init_drv_ForwardDist();

    // 初始化避障
    init_drv_Avoidance();

    // 初始化云台
    init_drv_Gimbal();

    // 初始化extio
    init_drv_ExtIO();

    // 初始化LED灯
    init_drv_ExtLed();

    // 初始化功能
    init_drv_Functions();

    // 初始化rid
    init_drv_Remoteid();

    // 初始化OSD
    init_drv_OSD();

    // 二次开发接口
    //  init_drv_SDI();
}