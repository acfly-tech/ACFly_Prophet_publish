#pragma once

#include "Basic.hpp"
#ifndef HWCONFIG_READONLY
#include "Parameters.hpp" // 仅主固件需引入 MAV_PARAM_TYPE/SName (BootLoader 不会编译 InitialParametersInit)
#endif
#include "ff.h"

/*
    硬件配置系统 (HardWareConfig)
    =============================

    功能概述:
      从SD卡读取INI配置文件 → 解析为二进制配置表 → 存入InternalStorage
      运行时各驱动通过 HW_ReadConfigEntry() 查询配置，动态初始化引脚和外设

    二进制格式:
      每条配置记录 = HW_ConfigEntry (4 × uint16_t = 8字节):
        [0] device      - 硬件设备编号 (HW_Device)
        [1] config_item - 配置项编号   (HW_ConfigItem)
        [2] value_high  - 高字值 (如GPIO端口号, DMA控制器号, Timer号, ADC通道号等)
        [3] value_low   - 低字值 (如引脚号, DMA流号, Timer通道号, AF值等)

    存储位置: InternalStorage Group="HardWare", Name="FuncMap"

    中断回调系统:
      DMA中断:   通过 DMA_CallBackFunc_Register()  注册 (见 dma.hpp)
      USART中断: 通过 USART_CallBackFunc_Register() 注册 (见下方)
      所有ISR handler统一定义在各自的回调系统中，驱动只需注册回调函数

    INI文件格式示例:
      [uart1]           ; section名 -> HW_Device
      tx_pin = PA9      ; key=value -> HW_ConfigItem + 解析值
      tx_af = 7
      tx_dma = DMA1_Stream3
      [sbus]            ; SBUS接收机配置
      uart = uart2      ; 使用uart2硬件
      rx_pin = PD6
*/

// 硬件设备编号 (INI section名 -> 设备号)
// 各驱动通过此编号 + HW_ConfigItem 查询自己的配置
enum HW_Device : uint16_t
{
    // 通用UART (1~8)
    HW_DEV_UART1 = 1,
    HW_DEV_UART2 = 2,
    HW_DEV_UART3 = 3,
    HW_DEV_UART4 = 4,
    HW_DEV_UART5 = 5,
    HW_DEV_UART6 = 6,
    HW_DEV_UART7 = 7,
    HW_DEV_UART8 = 8,
    // CAN总线
    HW_DEV_CAN1 = 10,
    HW_DEV_CAN2 = 11,
    // I2C总线
    HW_DEV_I2C1 = 20,
    HW_DEV_I2C2 = 21,
    HW_DEV_I2C3 = 22,
    HW_DEV_I2C4 = 23,
    // SPI总线
    HW_DEV_SPI1 = 30,
    HW_DEV_SPI2 = 31,
    HW_DEV_SPI3 = 32,
    HW_DEV_SPI4 = 33,
    HW_DEV_SPI5 = 34,
    HW_DEV_SPI6 = 35,
    // PWM输出通道 (1~16)
    HW_DEV_PWM1 = 40,
    HW_DEV_PWM2 = 41,
    HW_DEV_PWM3 = 42,
    HW_DEV_PWM4 = 43,
    HW_DEV_PWM5 = 44,
    HW_DEV_PWM6 = 45,
    HW_DEV_PWM7 = 46,
    HW_DEV_PWM8 = 47,
    HW_DEV_PWM9 = 48,
    HW_DEV_PWM10 = 49,
    HW_DEV_PWM11 = 50,
    HW_DEV_PWM12 = 51,
    HW_DEV_PWM13 = 52,
    HW_DEV_PWM14 = 53,
    HW_DEV_PWM15 = 54,
    HW_DEV_PWM16 = 55,
    // PWM总体设置
    HW_DEV_PWM = 74,
    // 输入触发捕获 (相机热靴等)
    HW_DEV_INPUT_TRIGGER = 76,
    // PPM 接收机输入
    HW_DEV_PPM = 77,
    // ADC
    HW_DEV_ADC = 75,
    // SDMMC
    // HW_DEV_SDMMC1 = 70,
    // 功能模块 (使用其他外设的上层驱动)
    HW_DEV_SBUS = 80,                 // SBUS接收机, 通过 HW_CFG_UART_REF 指定使用哪个UART
    HW_DEV_I2CINT = 81,               // 内部I2C功能, 通过 HW_CFG_I2C_REF 指定使用哪个I2C
    HW_DEV_I2C_COMPASS_INTERNAL = 82, // 内部I2C罗盘配置 (axis)
    HW_DEV_I2C_COMPASS_EXTERNAL = 83, // 外部I2C罗盘配置 (axis)

    // LED / 蜂鸣器
    HW_DEV_LED = 90,

    // imu
    HW_DEV_IMU_BMI088 = 100,
    HW_DEV_IMU_ICM20689 = 101,
    HW_DEV_IMU_ICM40609 = 102,
    HW_DEV_IMU_AK8975 = 103, // AK8975 SPI 磁力计

    // 超声波 (Trig/Echo 电平式, 用于[ultrasonic1]段)
    // HW_CFG_TIMER (15) 复用: value_high=定时器号, value_low=回波通道号(1~2)
    // PIN1=触发引脚 PIN2=回波引脚 AF2=回波AF PULL2=回波上下拉 OTYPE2=回波输出类型
    // HW_CFG_PARAM1 (28) 复用: value_low=高度传感器序号(1~4, 默认1)
    HW_DEV_ULTRASONIC1 = 105,

    // 气压计
    HW_DEV_IMUBARO_SPL06 = 170,
    // OSD
    HW_DEV_OSD_ANALOG = 180, // 模拟OSD (AT7456E)
                             // ESC
    HW_DEV_ESC_FPV = 190,    // FPV ESC (LPUART1 单线半双工)

    // 硬件元信息 (版本号等)
    HW_DEV_HARDWARE = 200,
    // [fixed_pins] 段: 固定 GPIO 输出
    //   config_item = (port << 4) | pin  (port 0~7, pin 0~15)
    //   value_low   = 输出电平 (0=低, 1=高)
    HW_DEV_FIXED_PIN = 201,
    // [exti_pins] 段: GPIO 外部中断输入
    //   config_item = (port << 4) | pin  (port 0~7, pin 0~15)
    //   value_high  = 触发沿 (HW_ExtiEdge: 0=rising, 1=falling, 2=both)
    //   value_low   = 上拉/下拉 (HW_Pull)
    HW_DEV_EXTI_PIN = 202,
};

// 配置项编号 (同一编号在不同HW_Device下含义不同)
//
//  编号  | UART       | I2C        | SPI        | PWM/ADC    | SBUS
//  ------+------------+------------+------------+------------+-----------
//  PIN1  | TX引脚     | SCL引脚    | SCK引脚    | 输出引脚   | -
//  PIN2  | RX引脚     | SDA引脚    | MISO引脚   | -          | RX引脚
//  PIN3  | -          | -          | MOSI引脚   | -          | -
//  AF1   | TX复用     | SCL复用    | SCK复用    | 复用功能   | -
//  AF2   | RX复用     | SDA复用    | MISO复用   | -          | RX复用
//  AF3   | -          | -          | MOSI复用   | -          | -
//  DMA1  | TX DMA     | TX DMA     | TX DMA     | 通用DMA    | -
//  DMA2  | RX DMA     | RX DMA     | RX DMA     | -          | -
enum HW_ConfigItem : uint16_t
{
    HW_CFG_PIN1 = 1,                // UART:TX_PIN  I2C:SCL_PIN  SPI:SCK_PIN   PWM/ADC:PIN    SENSOR:CS_PIN/GYRO_CS_PIN
    HW_CFG_PIN2 = 2,                // UART:RX_PIN  I2C:SDA_PIN  SPI:MISO_PIN  SENSOR:ACCEL_CS_PIN
    HW_CFG_PIN3 = 3,                // SPI:MOSI_PIN
    HW_CFG_AF1 = 4,                 // UART:TX_AF   I2C:SCL_AF   SPI:SCK_AF    PWM:AF
    HW_CFG_AF2 = 5,                 // UART:RX_AF   I2C:SDA_AF   SPI:MISO_AF
    HW_CFG_AF3 = 6,                 // SPI:MOSI_AF
    HW_CFG_PULL1 = 7,               // UART:TX_PULL I2C:SCL_PULL PWM:PULL
    HW_CFG_PULL2 = 8,               // UART:RX_PULL I2C:SDA_PULL
    HW_CFG_OTYPE1 = 9,              // UART:TX_OTYPE I2C:SCL_OTYPE PWM:OTYPE
    HW_CFG_OTYPE2 = 10,             // UART:RX_OTYPE I2C:SDA_OTYPE
    HW_CFG_SPEED1 = 11,             // UART:TX_SPEED I2C:SCL_SPEED PWM:SPEED
    HW_CFG_SPEED2 = 12,             // UART:RX_SPEED I2C:SDA_SPEED
    HW_CFG_DMA1 = 13,               // TX_DMA / 通用DMA
    HW_CFG_DMA2 = 14,               // RX_DMA
    HW_CFG_TIMER = 15,              // PWM: timer+channel
    HW_CFG_TYPE = 16,               // PWM: pwm/dshot类型
    HW_CFG_ENABLED = 17,            // 外设使能
    HW_CFG_COUNT = 18,              // PWM总通道数
    HW_CFG_CH_PIN = 19,             // ADC: channel pin, value_high=(ch<<8)|port, value_low=pin
    HW_CFG_TR_SWAP = 20,            // UART: TX/RX引脚交换 (CR2 SWAP bit)
    HW_CFG_UART_REF = 21,           // 引用的UART编号 (如SBUS指定用哪个UART, value_low=uart_num)
    HW_CFG_SPI_REF = 22,            // 引用的SPI编号 (如某功能模块使用哪个SPI, value_low=spi_num)
    HW_CFG_I2C_REF = 23,            // 引用的I2C编号 (如某功能模块使用哪个I2C, value_low=i2c_num)
    HW_CFG_CAN_REF = 24,            // 引用的CAN编号 (如某功能模块使用哪个CAN, value_low=can_num)
    HW_CFG_AXIS = 25,               // 轴向编号 (如陀螺仪/加速度计的X/Y/Z轴,value_high=bit0~3表示Z, value_low=bit0~3表示X bit4~7表示Y bit8~11表示Z)
    HW_CFG_PRIO = 26,               // 优先级 (数值越小优先级越高, 仅供驱动初始化时排序使用)
    HW_CFG_FUNC = 27,               // 功能号
    HW_CFG_PARAM1 = 28,             // 参数1    HW_CFG_PRESCALE = 43, // 定时器预分频 (value_low = 定时器PSC对应的模数，0=不额外分频)
    HW_CFG_FILTER = 44,             // 输入捕获滤波 (value_low = ICxF字段 0~15)    // ADC配置 (用于[adc]段)
    HW_CFG_ADC_RESOLUTION = 29,     // ADC分辨率 (value_low: 8/10/12/14/16)
    HW_CFG_ADC_OVERSAMPLING = 30,   // 过采样倍率 (value_high:value_low = uint32)
    HW_CFG_ADC_BAT1_VOLT = 31,      // 电池1电压通道 (value_high=adc_num, value_low=channel)
    HW_CFG_ADC_BAT1_VOLT_PIN = 32,  // 电池1电压引脚 (value_high=port, value_low=pin)
    HW_CFG_ADC_BAT1_VOLT_SMPT = 33, // 电池1电压采样时间 (value_low=0~7)
    HW_CFG_ADC_BAT2_VOLT = 34,      // 电池2电压通道
    HW_CFG_ADC_BAT2_VOLT_PIN = 35,  // 电池2电压引脚
    HW_CFG_ADC_BAT2_VOLT_SMPT = 36, // 电池2电压采样时间
    HW_CFG_ADC_BAT1_CUR = 37,       // 电池1电流通道
    HW_CFG_ADC_BAT1_CUR_PIN = 38,   // 电池1电流引脚
    HW_CFG_ADC_BAT1_CUR_SMPT = 39,  // 电池1电流采样时间
    HW_CFG_ADC_BAT2_CUR = 40,       // 电池2电流通道
    HW_CFG_ADC_BAT2_CUR_PIN = 41,   // 电池2电流引脚
    HW_CFG_ADC_BAT2_CUR_SMPT = 42,  // 电池2电流采样时间
    // 罗盘类型轴向 (用于i2c_compass_internal / i2c_compass_external 段)
    HW_CFG_QMC5883P = 100,
    HW_CFG_QMC5883 = 101,
    HW_CFG_IST8310 = 102,
    HW_CFG_FXOS8700 = 103,
    HW_CFG_QMC6983 = 104,
    HW_CFG_HMC5983 = 105,
    HW_CFG_HMC5883 = 106,
    HW_CFG_MAG3110 = 107,
    HW_CFG_RM3100 = 108,

    // LED / 蜂鸣器配置 (用于[led]段)
    // HW_CFG_TIMER (15) 复用: value_high=timer_num, value_low=0
    HW_CFG_LED_BUZZER_PIN = 200, // 蜂鸣器引脚 (value_high=port, value_low=pin)
    HW_CFG_LED_BUZZER_AF = 201,  // 蜂鸣器 AF
    HW_CFG_LED_BUZZER_CH = 202,  // 蜂鸣器定时器通道 (1~4)
    HW_CFG_LED_R_PIN = 203,      // 红色 LED 引脚
    HW_CFG_LED_R_AF = 204,
    HW_CFG_LED_R_CH = 205,
    HW_CFG_LED_R_INV = 206, // 低电平有效 (1=inverted)
    HW_CFG_LED_G_PIN = 207,
    HW_CFG_LED_G_AF = 208,
    HW_CFG_LED_G_CH = 209,
    HW_CFG_LED_G_INV = 210,
    HW_CFG_LED_B_PIN = 211,
    HW_CFG_LED_B_AF = 212,
    HW_CFG_LED_B_CH = 213,
    HW_CFG_LED_B_INV = 214,
    HW_CFG_LED_BUZZER_INV = 216, // 蜂鸣器极性反转 (1=inverted, 默认true)
    // i2cInt 扩展选项
    HW_CFG_I2C_EXT = 215, // ext=true 时同时向外部总线注册驱动 (value_low: 0=false, 1=true)

    // 硬件版本 (用于[hardware]段)
    HW_CFG_HW_MAIN_VER = 300, // 硬件主版本号 (A9=2, K9=5, F9=6)
    HW_CFG_HW_SUB_VER = 301,  // 硬件从版本号 (K9_2025V1=1, K9_2026V1=2, ...)
};

// GPIO端口编号
enum HW_Port : uint16_t
{
    HW_PORT_PA = 0,
    HW_PORT_PB = 1,
    HW_PORT_PC = 2,
    HW_PORT_PD = 3,
    HW_PORT_PE = 4,
    HW_PORT_PF = 5,
    HW_PORT_PG = 6,
    HW_PORT_PH = 7,
};

// GPIO上拉/下拉
enum HW_Pull : uint16_t
{
    HW_PULL_NONE = 0,
    HW_PULL_UP = 1,
    HW_PULL_DOWN = 2,
};

// GPIO输出类型
enum HW_OType : uint16_t
{
    HW_OTYPE_PUSH_PULL = 0,
    HW_OTYPE_OPEN_DRAIN = 1,
};

// GPIO速度
enum HW_Speed : uint16_t
{
    HW_SPEED_LOW = 0,
    HW_SPEED_MEDIUM = 1,
    HW_SPEED_HIGH = 2,
    HW_SPEED_VERY_HIGH = 3,
};

// EXTI触发沿
enum HW_ExtiEdge : uint16_t
{
    HW_EXTI_RISING = 0,
    HW_EXTI_FALLING = 1,
    HW_EXTI_BOTH = 2,
};

// PWM输出类型
enum HW_PwmType : uint16_t
{
    HW_PWM_TYPE_PWM = 0,
    HW_PWM_TYPE_DSHOT150 = 1,
    HW_PWM_TYPE_DSHOT300 = 2,
    HW_PWM_TYPE_DSHOT600 = 3,
};

// 单条配置记录
struct HW_ConfigEntry
{
    uint16_t device;
    uint16_t config_item;
    uint16_t value_high;
    uint16_t value_low;
};

// 参数初始值记录 (存储在 InternalStorage Group="HardWare", Name="InitParams")
// 由 HardWareInit 从 ini [parameters] 段解析写入，由 InitialParametersInit 读取使用
// 每条 28 字节：名称 16 字节 + 默认值 4 字节 + 最小值 4 字节 + 最大值 4 字节
struct HW_ParamInitEntry
{
    char name[16];    // 参数名（与 SName 二进制兼容，不足 16 字节右填 0）
    float defaultVal; // 出厂默认值
    float minVal;     // 最小值（当前版本存储但暂不使用）
    float maxVal;     // 最大值（当前版本存储但暂不使用）
};

// 从SD卡读取INI配置并写入InternalStorage
// 需要在SD卡usr_fs已挂载后调用
void HardWareInit(FIL *iniFile, FIL *logFile);

// 从InternalStorage中读取指定设备的指定配置项
// 返回true表示找到，entry中填入结果
bool HW_ReadConfigEntry(uint16_t device, uint16_t config_item, HW_ConfigEntry *entry);
void HW_FreeConfigCache(); // 所有驱动初始化完成后调用，释放FuncMap内存缓存

// 执行[fixed_pins]段指定的GPIO固定输出
//   从InternalStorage的FuncMap中读取device==HW_DEV_FIXED_PIN的条目,
//   逐条配置为GPIO输出并设定电平。
//   无需SD卡/INI文件, 只需InternalStorage已有数据。
//   在init_drv_hardware()早期阶段调用, 先于其他驱动初始化。
void HW_ApplyFixedPins();

// 执行[exti_pins]段指定的GPIO外部中断配置
//   从InternalStorage的FuncMap中读取device==HW_DEV_EXTI_PIN的条目,
//   配置GPIO输入+上下拉、SYSCFG EXTICR映射、EXTI边沿触发与NVIC使能。
//   无需SD卡/INI文件, 只需InternalStorage已有数据。
void HW_ApplyExtiPins();

// 注册EXTI引脚中断回调 (在ISR上下文执行, 中断优先级5, 与PPM/输入捕获一致)
//   port: HW_PORT_PA ~ HW_PORT_PH, pin: 0~15
//   未注册的引脚触发中断时只清标志不执行任何动作
void HW_EXTI_RegisterCallback(uint16_t port, uint16_t pin, void (*cb)(void));

// 读取INI中[hardware]段的主/从版本号, 未配置则填0
// 需在HardWareInit完成后调用
bool HW_GetVersion(uint16_t *main_ver, uint16_t *sub_ver);

// 从 InternalStorage 读取 ini 的 [parameters] 段，按名称匹配并覆盖初始参数数组
// 在调用 ParamGroupRegister 之前调用，使 ini 配置的值作为该组参数的出厂默认值
// initial_params: 已按编译默认值初始化的 uint64_t 参数数组
// param_types:    各参数的 MAV_PARAM_TYPE 类型数组
// param_names:    各参数的 SName 名称数组（与 ParamGroupRegister 传入的同一数组）
// count:          数组元素个数
// 注意: BootLoader 定义 HWCONFIG_READONLY 后不舍编译此函数（它只读 FuncMap，不涉及参数系统）
#ifndef HWCONFIG_READONLY
void InitialParametersInit(uint64_t initial_params[], const MAV_PARAM_TYPE param_types[], const SName param_names[], uint32_t count);
#endif

// 根据定时器编号获取 TIM_TypeDef 指针 (1~17, 不存在则返回 nullptr)
TIM_TypeDef *HW_GetTimerPtr(uint16_t timer_num);

// 使能指定定时器的 RCC 时钟
void HW_EnableTimerClock(uint16_t timer_num);

// ==== 硬件通用查找表 (STM32H743) ====
// 这些表将编号映射到寄存器地址，供各驱动在运行时根据INI配置动态初始化使用

// GPIO 端口表: HW_PORT_PA(0)~HW_PORT_PH(7) -> GPIO_TypeDef*
extern GPIO_TypeDef *const HW_GpioTable[8];

// RCC 使能信息
struct HW_RccEnableInfo
{
    volatile uint32_t *reg;
    uint32_t bit;
};