#include "HardWareConfig.hpp"

#ifndef HWCONFIG_READONLY
#include "Parameters.hpp" // 仅主固件需要 (InitialParametersInit 的实现用到 MAV_PARAM_TYPE)
#endif
#include "StorageSystem.hpp"
#include "TimeBase.h"
#include "drv_CRC.hpp"
#include "fatfs.h"
#include "stm32h743xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==== 硬件通用查找表定义 ====
GPIO_TypeDef *const HW_GpioTable[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};

// 最大配置条目数
// INI行缓冲区大小
#define HW_LINE_BUF_SIZE 128

// 挂载路径
static const char hw_usr_mountPath[4] = {DEV_SD_USR + '0', ':', '/', 0};
static const char hw_log_mountPath[4] = {DEV_SD_EXT + '0', ':', '/', 0};

// 硬件配置 CRC 算法配置（与 Parameters.cpp 保持一致，可在编译期覆盖）
#ifndef HARDWARE_CRC_CONFIG
#define HARDWARE_CRC_CONFIG cg_CRC_CONFIG_CRC32
#endif

/* ========== CRC 工具 ==========
   文件存储格式：[原始 payload][uint32_t crc]
   crc 覆盖 payload 全部字节（不含自身 4 字节）。*/
static bool HW_CalcCRC(const uint8_t *data, uint32_t length, uint32_t *res)
{
    if (length > 0xFFFF)
    {
        *res = 0;
        return false;
    }
    return CRC_Calc(HARDWARE_CRC_CONFIG, data, (uint16_t) length, res, (uint16_t) length);
}

// CRC 校验重试最大次数
#define HW_CRC_MAX_RETRY 3

// ---- 辅助函数 ----

static const char *skipSpaces(const char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    return s;
}

static void trimRight(char *s)
{
    int len = strlen(s);
    while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' || s[len - 1] == '\r' || s[len - 1] == '\n'))
        s[--len] = 0;
}

static int parsePort(char c)
{
    if (c >= 'A' && c <= 'H')
        return (int) (c - 'A');
    if (c >= 'a' && c <= 'h')
        return (int) (c - 'a');
    return -1;
}

static bool parsePin(const char *val, uint16_t *port, uint16_t *pin)
{
    val = skipSpaces(val);
    if ((val[0] != 'P' && val[0] != 'p') || val[1] == 0 || val[2] == 0)
        return false;
    int p = parsePort(val[1]);
    if (p < 0)
        return false;
    int pinNum = atoi(&val[2]);
    if (pinNum < 0 || pinNum > 15)
        return false;
    *port = (uint16_t) p;
    *pin = (uint16_t) pinNum;
    return true;
}

static uint16_t parsePull(const char *val)
{
    val = skipSpaces(val);
    if (strncmp(val, "pull_up", 7) == 0)
        return HW_PULL_UP;
    if (strncmp(val, "pull_down", 9) == 0)
        return HW_PULL_DOWN;
    return HW_PULL_NONE;
}

static uint16_t parseOType(const char *val)
{
    val = skipSpaces(val);
    if (strncmp(val, "open_drain", 10) == 0)
        return HW_OTYPE_OPEN_DRAIN;
    return HW_OTYPE_PUSH_PULL;
}

static uint16_t parseSpeed(const char *val)
{
    val = skipSpaces(val);
    if (strncmp(val, "very_high", 9) == 0)
        return HW_SPEED_VERY_HIGH;
    if (strncmp(val, "high", 4) == 0)
        return HW_SPEED_HIGH;
    if (strncmp(val, "medium", 6) == 0)
        return HW_SPEED_MEDIUM;
    return HW_SPEED_LOW;
}

// 解析 [exti_pins] 段的值: "rising|falling|both[, none|pull_up|pull_down]"
static bool parseExtiSpec(const char *val, uint16_t *edge, uint16_t *pull)
{
    const char *v = skipSpaces(val);
    uint16_t e;
    if (strncmp(v, "rising", 6) == 0)
        e = HW_EXTI_RISING;
    else if (strncmp(v, "falling", 7) == 0)
        e = HW_EXTI_FALLING;
    else if (strncmp(v, "both", 4) == 0)
        e = HW_EXTI_BOTH;
    else
        return false;
    *edge = e;
    const char *comma = strchr(v, ',');
    if (!comma)
    {
        *pull = HW_PULL_NONE; // 上下拉可省略, 默认none
        return true;
    }
    *pull = parsePull(comma + 1);
    return true;
}

static bool parseDMA(const char *val, uint16_t *dma_num, uint16_t *stream_num)
{
    val = skipSpaces(val);
    // BDMA_ChannelX → dma_num=3, stream_num=X (I2C4 等 D3 外设使用 BDMA)
    if (strncmp(val, "BDMA", 4) == 0 || strncmp(val, "bdma", 4) == 0)
    {
        const char *p = strstr(val, "Channel");
        if (!p)
            return false;
        *dma_num = 3; // 约定: 3 = BDMA
        *stream_num = (uint16_t) atoi(p + 7);
        return (*stream_num <= 7);
    }
    if (strncmp(val, "DMA", 3) != 0)
        return false;
    *dma_num = (uint16_t) atoi(&val[3]);
    const char *p = strstr(val, "Stream");
    if (!p)
        return false;
    *stream_num = (uint16_t) atoi(p + 6);
    return true;
}

static bool parseTimer(const char *val, uint16_t *timer_num, uint16_t *ch_num)
{
    val = skipSpaces(val);
    if (strncmp(val, "TIM", 3) != 0)
        return false;
    *timer_num = (uint16_t) atoi(&val[3]);
    const char *p = strstr(val, "CH");
    if (!p)
        return false;
    *ch_num = (uint16_t) atoi(p + 2);
    return true;
}

static uint16_t parsePwmType(const char *val)
{
    val = skipSpaces(val);
    if (strncmp(val, "dshot600", 8) == 0)
        return HW_PWM_TYPE_DSHOT600;
    if (strncmp(val, "dshot300", 8) == 0)
        return HW_PWM_TYPE_DSHOT300;
    if (strncmp(val, "dshot150", 8) == 0)
        return HW_PWM_TYPE_DSHOT150;
    return HW_PWM_TYPE_PWM;
}

static uint16_t parseSectionDevice(const char *section)
{
    if (strcmp(section, "uart1") == 0)
        return HW_DEV_UART1;
    if (strcmp(section, "uart2") == 0)
        return HW_DEV_UART2;
    if (strcmp(section, "uart3") == 0)
        return HW_DEV_UART3;
    if (strcmp(section, "uart4") == 0)
        return HW_DEV_UART4;
    if (strcmp(section, "uart5") == 0)
        return HW_DEV_UART5;
    if (strcmp(section, "uart6") == 0)
        return HW_DEV_UART6;
    if (strcmp(section, "uart7") == 0)
        return HW_DEV_UART7;
    if (strcmp(section, "uart8") == 0)
        return HW_DEV_UART8;
    if (strcmp(section, "can1") == 0)
        return HW_DEV_CAN1;
    if (strcmp(section, "can2") == 0)
        return HW_DEV_CAN2;
    if (strcmp(section, "i2c1") == 0)
        return HW_DEV_I2C1;
    if (strcmp(section, "i2c2") == 0)
        return HW_DEV_I2C2;
    if (strcmp(section, "i2c3") == 0)
        return HW_DEV_I2C3;
    if (strcmp(section, "i2c4") == 0)
        return HW_DEV_I2C4;
    if (strcmp(section, "spi1") == 0)
        return HW_DEV_SPI1;
    if (strcmp(section, "spi2") == 0)
        return HW_DEV_SPI2;
    if (strcmp(section, "spi3") == 0)
        return HW_DEV_SPI3;
    if (strcmp(section, "spi4") == 0)
        return HW_DEV_SPI4;
    if (strcmp(section, "spi5") == 0)
        return HW_DEV_SPI5;
    if (strcmp(section, "spi6") == 0)
        return HW_DEV_SPI6;
    if (strcmp(section, "adc") == 0)
        return HW_DEV_ADC;
    if (strcmp(section, "sbus") == 0)
        return HW_DEV_SBUS;
    if (strcmp(section, "i2cint") == 0)
        return HW_DEV_I2CINT;
    if (strcmp(section, "i2c_compass_internal") == 0)
        return HW_DEV_I2C_COMPASS_INTERNAL;
    if (strcmp(section, "i2c_compass_external") == 0)
        return HW_DEV_I2C_COMPASS_EXTERNAL;
    if (strncmp(section, "pwm", 3) == 0)
    {
        if (section[3] == 0)
            return HW_DEV_PWM; // 默认pwm段映射到PWM1
        if (section[3] >= '0' && section[3] <= '9')
        {
            int n = atoi(&section[3]);
            if (n >= 1 && n <= 16)
                return (uint16_t) (HW_DEV_PWM1 + n - 1);
        }
    }
    if (strcmp(section, "imu_bmi088") == 0)
        return HW_DEV_IMU_BMI088;
    if (strcmp(section, "imu_icm20689") == 0)
        return HW_DEV_IMU_ICM20689;
    if (strcmp(section, "imu_icm40609") == 0)
        return HW_DEV_IMU_ICM40609;
    if (strcmp(section, "imu_ak8975") == 0)
        return HW_DEV_IMU_AK8975;
    if (strcmp(section, "imubaro_spl06") == 0)
        return HW_DEV_IMUBARO_SPL06;
    if (strcmp(section, "osd_analog") == 0)
        return HW_DEV_OSD_ANALOG;
    if (strcmp(section, "led") == 0)
        return HW_DEV_LED;
    if (strcmp(section, "input_trigger") == 0)
        return HW_DEV_INPUT_TRIGGER;
    if (strcmp(section, "ultrasonic1") == 0)
        return HW_DEV_ULTRASONIC1;
    if (strcmp(section, "ppm") == 0)
        return HW_DEV_PPM;
    if (strcmp(section, "esc_fpv") == 0)
        return HW_DEV_ESC_FPV;
    if (strcmp(section, "hardware") == 0)
        return HW_DEV_HARDWARE;
    if (strcmp(section, "parameters") == 0)
        return (uint16_t) 0xFFFF; // 特殊标记：参数初始值段，跳过硬件条目解析
    if (strcmp(section, "fixed_pins") == 0)
        return (uint16_t) 0xFFFE; // 特殊标记：固定引脚配置段，直接配置GPIO
    if (strcmp(section, "exti_pins") == 0)
        return (uint16_t) 0xFFFD; // 特殊标记：EXTI外部中断引脚配置段
    return 0;
}

static bool addEntry(HW_ConfigEntry *entries, uint16_t *count, uint16_t maxCount, uint16_t device, uint16_t config_item, uint16_t value_high, uint16_t value_low)
{
    if (entries)
    {
        if (*count >= maxCount)
            return false;
        entries[*count].device = device;
        entries[*count].config_item = config_item;
        entries[*count].value_high = value_high;
        entries[*count].value_low = value_low;
    }
    (*count)++;
    return true;
}

// key名直接映射到 HW_ConfigItem
struct KeyMap
{
    const char *name;
    HW_ConfigItem item;
};

static const KeyMap keyTable[] = {
    // PIN类
    {"tx_pin", HW_CFG_PIN1},
    {"rx_pin", HW_CFG_PIN2},
    {"scl_pin", HW_CFG_PIN1},
    {"sda_pin", HW_CFG_PIN2},
    {"sck_pin", HW_CFG_PIN1},
    {"miso_pin", HW_CFG_PIN2},
    {"mosi_pin", HW_CFG_PIN3},
    {"pin", HW_CFG_PIN1},
    {"cs_pin", HW_CFG_PIN1},
    {"gyro_cs_pin", HW_CFG_PIN1},
    {"accel_cs_pin", HW_CFG_PIN2},
    // AF类
    {"tx_af", HW_CFG_AF1},
    {"rx_af", HW_CFG_AF2},
    {"scl_af", HW_CFG_AF1},
    {"sda_af", HW_CFG_AF2},
    {"sck_af", HW_CFG_AF1},
    {"miso_af", HW_CFG_AF2},
    {"mosi_af", HW_CFG_AF3},
    {"af", HW_CFG_AF1},
    // PULL类
    {"tx_pull", HW_CFG_PULL1},
    {"rx_pull", HW_CFG_PULL2},
    {"scl_pull", HW_CFG_PULL1},
    {"sda_pull", HW_CFG_PULL2},
    {"pull", HW_CFG_PULL1},
    // OTYPE类
    {"tx_otype", HW_CFG_OTYPE1},
    {"rx_otype", HW_CFG_OTYPE2},
    {"scl_otype", HW_CFG_OTYPE1},
    {"sda_otype", HW_CFG_OTYPE2},
    {"otype", HW_CFG_OTYPE1},
    // SPEED类
    {"tx_speed", HW_CFG_SPEED1},
    {"rx_speed", HW_CFG_SPEED2},
    {"scl_speed", HW_CFG_SPEED1},
    {"sda_speed", HW_CFG_SPEED2},
    {"speed", HW_CFG_SPEED1},
    // DMA类
    {"tx_dma", HW_CFG_DMA1},
    {"rx_dma", HW_CFG_DMA2},
    {"dma", HW_CFG_DMA1},
    // 其他
    {"timer", HW_CFG_TIMER},
    {"type", HW_CFG_TYPE},
    {"enabled", HW_CFG_ENABLED},
    {"count", HW_CFG_COUNT},
    {"tr_swap", HW_CFG_TR_SWAP},
    {"uart", HW_CFG_UART_REF},
    {"spi", HW_CFG_SPI_REF},
    {"i2c", HW_CFG_I2C_REF},
    {"can", HW_CFG_CAN_REF},
    {"axis", HW_CFG_AXIS},
    {"prio", HW_CFG_PRIO},
    {"param1", HW_CFG_PARAM1},
    {"prescale", HW_CFG_PARAM1},
    {"filter", HW_CFG_FILTER},
    // 罗盘类型轴向
    {"axis_qmc5883p", HW_CFG_QMC5883P},
    {"axis_qmc5883", HW_CFG_QMC5883},
    {"axis_ist8310", HW_CFG_IST8310},
    {"axis_fxos8700", HW_CFG_FXOS8700},
    {"axis_qmc6983", HW_CFG_QMC6983},
    {"axis_hmc5983", HW_CFG_HMC5983},
    {"axis_hmc5883", HW_CFG_HMC5883},
    {"axis_mag3110", HW_CFG_MAG3110},
    // 罗盘类型轴向 (新格式: 直接用罗盘名, 如 QMC5883P = x, y, z | addr = 0x0D)
    {"qmc5883p", HW_CFG_QMC5883P},
    {"qmc5883", HW_CFG_QMC5883},
    {"ist8310", HW_CFG_IST8310},
    {"fxos8700", HW_CFG_FXOS8700},
    {"qmc6983", HW_CFG_QMC6983},
    {"hmc5983", HW_CFG_HMC5983},
    {"hmc5883", HW_CFG_HMC5883},
    {"mag3110", HW_CFG_MAG3110},
    {"rm3100", HW_CFG_RM3100},
    // ADC配置
    {"resolution", HW_CFG_ADC_RESOLUTION},
    {"oversampling", HW_CFG_ADC_OVERSAMPLING},
    {"bat1_volt", HW_CFG_ADC_BAT1_VOLT},
    {"bat1_volt_pin", HW_CFG_ADC_BAT1_VOLT_PIN},
    {"bat1_volt_sample_time", HW_CFG_ADC_BAT1_VOLT_SMPT},
    {"bat2_volt", HW_CFG_ADC_BAT2_VOLT},
    {"bat2_volt_pin", HW_CFG_ADC_BAT2_VOLT_PIN},
    {"bat2_volt_sample_time", HW_CFG_ADC_BAT2_VOLT_SMPT},
    {"bat1_current", HW_CFG_ADC_BAT1_CUR},
    {"bat1_current_pin", HW_CFG_ADC_BAT1_CUR_PIN},
    {"bat1_current_sample_time", HW_CFG_ADC_BAT1_CUR_SMPT},
    {"bat2_current", HW_CFG_ADC_BAT2_CUR},
    {"bat2_current_pin", HW_CFG_ADC_BAT2_CUR_PIN},
    {"bat2_current_sample_time", HW_CFG_ADC_BAT2_CUR_SMPT},
    // LED / 蜂鸣器配置
    {"buzzer_pin", HW_CFG_LED_BUZZER_PIN},
    {"buzzer_af", HW_CFG_LED_BUZZER_AF},
    {"buzzer_channel", HW_CFG_LED_BUZZER_CH},
    {"buzzer_inverted", HW_CFG_LED_BUZZER_INV},
    {"red_pin", HW_CFG_LED_R_PIN},
    {"red_af", HW_CFG_LED_R_AF},
    {"red_channel", HW_CFG_LED_R_CH},
    {"red_inverted", HW_CFG_LED_R_INV},
    {"green_pin", HW_CFG_LED_G_PIN},
    {"green_af", HW_CFG_LED_G_AF},
    {"green_channel", HW_CFG_LED_G_CH},
    {"green_inverted", HW_CFG_LED_G_INV},
    {"blue_pin", HW_CFG_LED_B_PIN},
    {"blue_af", HW_CFG_LED_B_AF},
    {"blue_channel", HW_CFG_LED_B_CH},
    {"blue_inverted", HW_CFG_LED_B_INV},
    // 超声波 (Trig/Echo)
    {"trig_pin", HW_CFG_PIN1},
    {"echo_pin", HW_CFG_PIN2},
    {"echo_af", HW_CFG_AF2},
    {"echo_pull", HW_CFG_PULL2},
    {"echo_otype", HW_CFG_OTYPE2},
    // i2cInt 扩展选项
    {"ext", HW_CFG_I2C_EXT},
    // 硬件版本
    {"main_version", HW_CFG_HW_MAIN_VER},
    {"sub_version", HW_CFG_HW_SUB_VER},
};
static const int keyTableSize = sizeof(keyTable) / sizeof(keyTable[0]);

static HW_ConfigItem lookupKey(const char *key)
{
    // ADC channel pin特殊处理: ch*_pin
    if (strncmp(key, "ch", 2) == 0 && strstr(key, "_pin"))
        return HW_CFG_CH_PIN;
    for (int i = 0; i < keyTableSize; i++)
    {
        if (strcmp(key, keyTable[i].name) == 0)
            return keyTable[i].item;
    }
    return (HW_ConfigItem) 0;
}

// 解析一行 key=value 并添加配置记录
// 返回 false 表示遇到无法识别的key
static bool parseKeyValue(const char *key, const char *val, uint16_t device, HW_ConfigEntry *entries, uint16_t *count, uint16_t maxCount)
{
    uint16_t port, pin, v1, v2;
    HW_ConfigItem item = lookupKey(key);

    switch (item)
    {
        // --- PIN类 ---
        case HW_CFG_PIN1:
        case HW_CFG_PIN2:
        case HW_CFG_PIN3:
            if (!parsePin(val, &port, &pin))
                return false;
            addEntry(entries, count, maxCount, device, item, port, pin);
            break;

        // --- AF类 ---
        case HW_CFG_AF1:
        case HW_CFG_AF2:
        case HW_CFG_AF3:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        // --- PULL类 ---
        case HW_CFG_PULL1:
        case HW_CFG_PULL2:
            addEntry(entries, count, maxCount, device, item, 0, parsePull(val));
            break;

        // --- OTYPE类 ---
        case HW_CFG_OTYPE1:
        case HW_CFG_OTYPE2:
            addEntry(entries, count, maxCount, device, item, 0, parseOType(val));
            break;

        // --- SPEED类 ---
        case HW_CFG_SPEED1:
        case HW_CFG_SPEED2:
            addEntry(entries, count, maxCount, device, item, 0, parseSpeed(val));
            break;

        // --- DMA类 ---
        case HW_CFG_DMA1:
        case HW_CFG_DMA2:
            if (!parseDMA(val, &v1, &v2))
                return false;
            addEntry(entries, count, maxCount, device, item, v1, v2);
            break;

        // --- Timer ---
        case HW_CFG_TIMER:
            if (device == HW_DEV_LED)
            {
                // [led] 段的 timer 只有定时器号，无通道号，格式: "TIM2"
                const char *v = skipSpaces(val);
                if (strncmp(v, "TIM", 3) != 0 && strncmp(v, "tim", 3) != 0)
                    return false;
                uint16_t tnum = (uint16_t) atoi(v + 3);
                if (tnum == 0)
                    return false;
                addEntry(entries, count, maxCount, device, item, tnum, 0);
            }
            else
            {
                if (!parseTimer(val, &v1, &v2))
                    return false;
                addEntry(entries, count, maxCount, device, item, v1, v2);
            }
            break;

        // --- PWM type ---
        case HW_CFG_TYPE:
            addEntry(entries, count, maxCount, device, item, 0, parsePwmType(val));
            break;

        // --- enabled ---
        case HW_CFG_ENABLED:
        {
            const char *v = skipSpaces(val);
            addEntry(entries, count, maxCount, device, item, 0, (strncmp(v, "true", 4) == 0) ? 1 : 0);
            break;
        }

        // --- count ---
        case HW_CFG_COUNT:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        // --- ADC channel pin: ch10_pin = PC0 ---
        case HW_CFG_CH_PIN:
        {
            uint16_t ch = (uint16_t) atoi(&key[2]);
            if (!parsePin(val, &port, &pin))
                return false;
            addEntry(entries, count, maxCount, device, HW_CFG_CH_PIN, (ch << 8) | port, pin);
            break;
        }

        // --- UART TR_SWAP ---
        case HW_CFG_TR_SWAP:
        {
            const char *v = skipSpaces(val);
            addEntry(entries, count, maxCount, device, item, 0, (strncmp(v, "true", 4) == 0 || strncmp(v, "1", 1) == 0) ? 1 : 0);
            break;
        }

        // --- UART引用: "uart2" -> value_low=2 ---
        case HW_CFG_UART_REF:
        {
            const char *v = skipSpaces(val);
            if (strncmp(v, "uart", 4) != 0)
                return false;
            uint16_t unum = (uint16_t) atoi(v + 4);
            if (unum < 1 || unum > 8)
                return false;
            addEntry(entries, count, maxCount, device, item, 0, unum);
            break;
        }
        // --- SPI引用: "spi1" -> value_low=1 ---
        case HW_CFG_SPI_REF:
        {
            const char *v = skipSpaces(val);
            if (strncmp(v, "spi", 3) != 0)
                return false;
            uint16_t snum = (uint16_t) atoi(v + 3);
            if (snum < 1 || snum > 6)
                return false;
            addEntry(entries, count, maxCount, device, item, 0, snum);
            break;
        }
        // --- I2C引用: "i2c1" -> value_low=1 ---
        case HW_CFG_I2C_REF:
        {
            const char *v = skipSpaces(val);
            if (strncmp(v, "i2c", 3) != 0)
                return false;
            uint16_t inum = (uint16_t) atoi(v + 3);
            if (inum < 1 || inum > 4)
                return false;
            addEntry(entries, count, maxCount, device, item, 0, inum);
            break;
        }
        // --- CAN引用: "can1" -> value_low=1 ---
        case HW_CFG_CAN_REF:
        {
            const char *v = skipSpaces(val);
            if (strncmp(v, "can", 3) != 0)
                return false;
            uint16_t cnum = (uint16_t) atoi(v + 3);
            if (cnum < 1 || cnum > 1)
                return false;
            addEntry(entries, count, maxCount, device, item, 0, cnum);
            break;
        }

        // --- i2cInt ext 标志 ---
        case HW_CFG_I2C_EXT:
        {
            const char *v = skipSpaces(val);
            addEntry(entries, count, maxCount, device, item, 0, (strncmp(v, "true", 4) == 0 || strncmp(v, "1", 1) == 0) ? 1 : 0);
            break;
        }

        // --- 优先级 ---
        case HW_CFG_PRIO:
        {
            const char *v = skipSpaces(val);
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(v));
            break;
        }

        // 参数1
        case HW_CFG_PARAM1:
        {
            uint32_t content = (uint32_t) atoi(val);
            addEntry(entries, count, maxCount, device, item, (uint16_t) (content >> 16), (uint16_t) content);
            break;
        }

        // --- 预分频 / 滤波 ---
        case HW_CFG_FILTER:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        // --- 轴向: "axis = y, -x, z" / "QMC5883P = x, y, z" / "QMC5883P = x, y, z | addr = 0x0D" ---
        // value_high: I2C地址覆写 (0=使用协议默认地址)
        // value_low: bit0~3=X轴映射, bit4~7=Y轴映射, bit8~11=Z轴映射
        // 每个nibble: 1=+sensorX, 2=+sensorY, 3=+sensorZ, 0x9=-sensorX, 0xA=-sensorY, 0xB=-sensorZ
        case HW_CFG_QMC5883P:
        case HW_CFG_QMC5883:
        case HW_CFG_IST8310:
        case HW_CFG_FXOS8700:
        case HW_CFG_QMC6983:
        case HW_CFG_HMC5983:
        case HW_CFG_HMC5883:
        case HW_CFG_MAG3110:
        case HW_CFG_RM3100:
        case HW_CFG_AXIS:
        {
            // 解析三个逗号分隔的轴向: "y, -x, z"
            const char *p = skipSpaces(val);
            uint16_t axes[3] = {0, 0, 0};
            for (int ai = 0; ai < 3; ai++)
            {
                p = skipSpaces(p);
                bool neg = false;
                if (*p == '-')
                {
                    neg = true;
                    p++;
                }
                else if (*p == '+')
                {
                    p++;
                }
                uint8_t sAxis = 0;
                if (*p == 'x' || *p == 'X')
                    sAxis = 1;
                else if (*p == 'y' || *p == 'Y')
                    sAxis = 2;
                else if (*p == 'z' || *p == 'Z')
                    sAxis = 3;
                else
                    return false;
                if (neg)
                    sAxis |= 0x8;
                axes[ai] = sAxis;
                p++;
                // 跳过逗号
                while (*p == ' ' || *p == '\t' || *p == ',')
                    p++;
            }
            // 可选: 解析 "| addr = 0xXX"
            uint16_t addr = 0;
            p = skipSpaces(p);
            if (*p == '|')
            {
                p++;
                p = skipSpaces(p);
                if (strncmp(p, "addr", 4) == 0)
                {
                    p += 4;
                    p = skipSpaces(p);
                    if (*p == '=')
                    {
                        p++;
                        p = skipSpaces(p);
                        addr = (uint16_t) strtol(p, nullptr, 0);
                    }
                }
            }
            addEntry(entries, count, maxCount, device, item, addr, (uint16_t) ((axes[2] << 8) | (axes[1] << 4) | axes[0]));
            break;
        }

        // --- ADC通道引用: "adc3_ch10" -> value_high=adc_num, value_low=channel ---
        case HW_CFG_ADC_BAT1_VOLT:
        case HW_CFG_ADC_BAT2_VOLT:
        case HW_CFG_ADC_BAT1_CUR:
        case HW_CFG_ADC_BAT2_CUR:
        {
            const char *v = skipSpaces(val);
            if (strncmp(v, "adc", 3) != 0)
                return false;
            uint8_t adc_num = (uint8_t) atoi(v + 3);
            const char *ch = strstr(v, "_ch");
            if (!ch)
                return false;
            uint8_t channel = (uint8_t) atoi(ch + 3);
            addEntry(entries, count, maxCount, device, item, adc_num, channel);
            break;
        }

        // --- ADC通道引脚 ---
        case HW_CFG_ADC_BAT1_VOLT_PIN:
        case HW_CFG_ADC_BAT2_VOLT_PIN:
        case HW_CFG_ADC_BAT1_CUR_PIN:
        case HW_CFG_ADC_BAT2_CUR_PIN:
            if (!parsePin(val, &port, &pin))
                return false;
            addEntry(entries, count, maxCount, device, item, port, pin);
            break;

        // --- ADC分辨率/采样时间 ---
        case HW_CFG_ADC_RESOLUTION:
        case HW_CFG_ADC_BAT1_VOLT_SMPT:
        case HW_CFG_ADC_BAT2_VOLT_SMPT:
        case HW_CFG_ADC_BAT1_CUR_SMPT:
        case HW_CFG_ADC_BAT2_CUR_SMPT:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        // --- ADC过采样 ---
        case HW_CFG_ADC_OVERSAMPLING:
        {
            uint32_t content = (uint32_t) atoi(val);
            addEntry(entries, count, maxCount, device, item, (uint16_t) (content >> 16), (uint16_t) content);
            break;
        }

        // --- LED 引脚 ---
        case HW_CFG_LED_BUZZER_PIN:
        case HW_CFG_LED_R_PIN:
        case HW_CFG_LED_G_PIN:
        case HW_CFG_LED_B_PIN:
            if (!parsePin(val, &port, &pin))
                return false;
            addEntry(entries, count, maxCount, device, item, port, pin);
            break;

        // --- LED AF ---
        case HW_CFG_LED_BUZZER_AF:
        case HW_CFG_LED_R_AF:
        case HW_CFG_LED_G_AF:
        case HW_CFG_LED_B_AF:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        // --- LED 通道号 ---
        case HW_CFG_LED_BUZZER_CH:
        case HW_CFG_LED_R_CH:
        case HW_CFG_LED_G_CH:
        case HW_CFG_LED_B_CH:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        // --- LED 极性反转 ---
        case HW_CFG_LED_BUZZER_INV:
        case HW_CFG_LED_R_INV:
        case HW_CFG_LED_G_INV:
        case HW_CFG_LED_B_INV:
        {
            const char *v = skipSpaces(val);
            addEntry(entries, count, maxCount, device, item, 0, (strncmp(v, "true", 4) == 0 || strncmp(v, "1", 1) == 0) ? 1 : 0);
            break;
        }

        // --- 硬件版本号 ---
        case HW_CFG_HW_MAIN_VER:
        case HW_CFG_HW_SUB_VER:
            addEntry(entries, count, maxCount, device, item, 0, (uint16_t) atoi(val));
            break;

        default:
            return false;
    }
    return true;
}

bool HW_GetVersion(uint16_t *main_ver, uint16_t *sub_ver)
{
    HW_ConfigEntry entry;
    *main_ver = 0;
    *sub_ver = 0;
    bool ok = false;
    if (HW_ReadConfigEntry(HW_DEV_HARDWARE, HW_CFG_HW_MAIN_VER, &entry))
    {
        *main_ver = entry.value_low;
        ok = true;
    }
    if (HW_ReadConfigEntry(HW_DEV_HARDWARE, HW_CFG_HW_SUB_VER, &entry))
    {
        *sub_ver = entry.value_low;
        ok = true;
    }
    return ok;
}

// 从文件中逐行读取
static int readLine(FIL *fp, char *buf, int bufSize)
{
    int i = 0;
    UINT br;
    char c;
    while (i < bufSize - 1)
    {
        FRESULT res = f_read(fp, &c, 1, &br);
        if (res != FR_OK || br == 0)
            break;
        if (c == '\n')
            break;
        buf[i++] = c;
    }
    buf[i] = 0;
    return i;
}

// 跳过各种可能的 BOM 头，并返回实际内容起始位置
static const char *SkipBOM(const char *line)
{
    if (line == NULL)
        return NULL;

    const unsigned char *p = (const unsigned char *) line;

    // UTF-8 BOM: EF BB BF
    if (p[0] == 0xEF && p[1] == 0xBB && p[2] == 0xBF)
        return (const char *) (p + 3);

    // UTF-16 LE BOM: FF FE
    if (p[0] == 0xFF && p[1] == 0xFE)
        return (const char *) (p + 2);

    // UTF-16 BE BOM: FE FF
    if (p[0] == 0xFE && p[1] == 0xFF)
        return (const char *) (p + 2);

    // UTF-32 LE BOM: FF FE 00 00
    if (p[0] == 0xFF && p[1] == 0xFE && p[2] == 0x00 && p[3] == 0x00)
        return (const char *) (p + 4);

    // UTF-32 BE BOM: 00 00 FE FF
    if (p[0] == 0x00 && p[1] == 0x00 && p[2] == 0xFE && p[3] == 0xFF)
        return (const char *) (p + 4);

    // 没有 BOM，直接返回原指针
    return line;
}

// 解析INI行提取section/key/value的通用函数
// 返回: 0=空行/注释, 1=section(写入secBuf), 2=key=value(写入keyBuf,valBuf), -1=格式错误
static int parseLine(const char *lineBuf, char *secBuf, int secBufSize, char *keyBuf, int keyBufSize, char *valBuf, int valBufSize)
{
    char trimBuf[HW_LINE_BUF_SIZE];
    strncpy(trimBuf, lineBuf, HW_LINE_BUF_SIZE - 1);
    trimBuf[HW_LINE_BUF_SIZE - 1] = 0;
    trimRight(trimBuf);
    const char *line = skipSpaces(trimBuf);

    // 跳过各种可能的 BOM 头
    line = SkipBOM(line);

    if (line[0] == 0 || line[0] == ';' || line[0] == '#')
        return 0;

    if (line[0] == '[')
    {
        const char *end = strchr(line, ']');
        if (!end)
            return -1;
        int len = end - line - 1;
        if (len <= 0 || len >= secBufSize)
            return -1;
        memcpy(secBuf, line + 1, len);
        secBuf[len] = 0;
        for (int i = 0; i < len; i++)
        {
            if (secBuf[i] >= 'A' && secBuf[i] <= 'Z')
                secBuf[i] += 32;
        }
        return 1;
    }

    const char *eq = strchr(line, '=');
    if (!eq)
        return -1;

    int keyLen = eq - line;
    while (keyLen > 0 && (line[keyLen - 1] == ' ' || line[keyLen - 1] == '\t'))
        keyLen--;
    if (keyLen <= 0 || keyLen >= keyBufSize)
        return -1;
    memcpy(keyBuf, line, keyLen);
    keyBuf[keyLen] = 0;
    for (int i = 0; i < keyLen; i++)
    {
        if (keyBuf[i] >= 'A' && keyBuf[i] <= 'Z')
            keyBuf[i] += 32;
    }
    const char *valStart = skipSpaces(eq + 1);
    strncpy(valBuf, valStart, valBufSize - 1);
    valBuf[valBufSize - 1] = 0;
    trimRight(valBuf);
    return 2;
}

// 前向声明：带 CRC 校验的存储读写辅助（实现在文件后半段，此处供 HardWareInit 使用）
// out_crc: 非空时返回本次计算出的 CRC32（用于日志记录，避免调用者重复计算）
static SS_RESULT HW_SaveWithCRC(const char *group, const char *name, const void *payload, uint32_t payload_len, uint32_t *out_crc, double TIMEOUT);

void HardWareInit(FIL *iniFile, FIL *logFile)
{
    // 构造ACFlySetup路径
    char path[80];
    path[0] = 0;
    strcat(path, hw_usr_mountPath);
    strcat(path, "ACFlySetup");

    FILINFO finfo;
    FRESULT fres = f_stat(path, &finfo);
    if (fres != FR_OK)
        return;

    // 查找 *.ini 文件
    DIR dir;
    fres = f_findfirst(&dir, &finfo, path, "*.ini");
    if (fres != FR_OK || finfo.fname[0] == 0)
    {
        f_closedir(&dir);
        return;
    }
    f_closedir(&dir);

    char filepath[120];
    filepath[0] = 0;
    strcat(filepath, path);
    strcat(filepath, "/");
    strcat(filepath, finfo.fname);

    // 打开日志文件
    char logTxt[150];
    bool logOpened = false;
    {
        char log_filename[40];
        log_filename[0] = 0;
        strcat(log_filename, hw_log_mountPath);
        strcat(log_filename, "ACFly/sysLog.txt");
        fres = f_open(logFile, log_filename, FA_OPEN_APPEND | FA_WRITE);
        if (fres == FR_OK)
        {
            logOpened = true;
            RTC_TimeStruct RTC_Time = Get_RTC_Time();
            sprintf(logTxt, "\r\nHardWare Config Loader\r\n%d/%02d/%02d-%02d:%02d:%02d\r\n", RTC_Time.Year, RTC_Time.Month, RTC_Time.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
            f_puts(logTxt, logFile);
            sprintf(logTxt, "\tConfig file: %s\r\n", finfo.fname);
            f_puts(logTxt, logFile);
        }
    }

    // 打开INI文件
    fres = f_open(iniFile, filepath, FA_OPEN_EXISTING | FA_READ);
    if (fres != FR_OK)
    {
        if (logOpened)
        {
            f_puts("\tFailed to open config file.\r\n", logFile);
            f_close(logFile);
        }
        f_unlink(filepath);
        return;
    }

    // --- 第一遍：预读验证并统计条目数 ---
    bool parseOk = true;
    int lineNum = 0;
    char lineBuf[HW_LINE_BUF_SIZE];
    char secBuf[32];
    char keyBuf[32];
    char valBuf[HW_LINE_BUF_SIZE];
    uint16_t currentDevice = 0;
    uint16_t totalEntryCount = 0;
    uint32_t paramEntryCount = 0; // [parameters] 段条目数，用于分配二进制数组

    while (true)
    {
        if (readLine(iniFile, lineBuf, HW_LINE_BUF_SIZE) == 0 && f_eof(iniFile))
            break;
        lineNum++;
        int pres = parseLine(lineBuf, secBuf, sizeof(secBuf), keyBuf, sizeof(keyBuf), valBuf, sizeof(valBuf));
        if (pres == 0)
        {
            if (f_eof(iniFile))
                break;
            continue;
        }
        if (pres < 0)
        {
            if (logOpened)
            {
                sprintf(logTxt, "\tLine %d: syntax error.\r\n", lineNum);
                f_puts(logTxt, logFile);
            }
            parseOk = false;
            break;
        }
        if (pres == 1)
        {
            // section
            uint16_t dev = parseSectionDevice(secBuf);
            if (dev == 0)
            {
                if (logOpened)
                {
                    sprintf(logTxt, "\tLine %d: unknown section [%s].\r\n", lineNum, secBuf);
                    f_puts(logTxt, logFile);
                }
                parseOk = false;
                break;
            }
            currentDevice = dev;
        }
        else if (pres == 2)
        {
            if (currentDevice == 0)
            {
                if (logOpened)
                {
                    sprintf(logTxt, "\tLine %d: key before any section.\r\n", lineNum);
                    f_puts(logTxt, logFile);
                }
                parseOk = false;
                break;
            }
            if (currentDevice == 0xFFFF)
            {
                // [parameters] 段：不解析为硬件条目，只统计条目数用于分配二进制数组
                paramEntryCount++;
            }
            else if (currentDevice == 0xFFFE)
            {
                // [fixed_pins] 段: 每行 key=引脚名 value=电平, 统计后存入 FuncMap
                uint16_t port, pin;
                if (parsePin(keyBuf, &port, &pin) && port < 8 && pin <= 15)
                    totalEntryCount++;
            }
            else if (currentDevice == 0xFFFD)
            {
                // [exti_pins] 段: 每行 key=引脚名 value="沿, 上下拉", 统计后存入 FuncMap
                uint16_t port, pin, edge, pull;
                if (parsePin(keyBuf, &port, &pin) && port < 8 && pin <= 15 && parseExtiSpec(valBuf, &edge, &pull))
                    totalEntryCount++;
            }
            else
            {
                // 验证key是否可识别，同时统计条目数 (entries传0不写入)
                uint16_t tmpCount = 0;
                if (!parseKeyValue(keyBuf, valBuf, currentDevice, 0, &tmpCount, 0))
                {
                    if (logOpened)
                    {
                        sprintf(logTxt, "\tLine %d: unknown key \"%s\" or invalid value.\r\n", lineNum, keyBuf);
                        f_puts(logTxt, logFile);
                    }
                    parseOk = false;
                    break;
                }
                totalEntryCount += tmpCount;
            }
        }

        if (f_eof(iniFile))
            break;
    }

    if (!parseOk)
    {
        if (logOpened)
        {
            f_puts("\tConfig parse failed, aborted.\r\n", logFile);
            f_close(logFile);
        }
        f_close(iniFile);
        f_unlink(filepath);
        return;
    }

    // --- 第二遍：实际解析并写入 ---
    f_lseek(iniFile, 0);

    HW_ConfigEntry *entries = nullptr;
    if (totalEntryCount > 0)
        entries = (HW_ConfigEntry *) pvPortMalloc(totalEntryCount * sizeof(HW_ConfigEntry));
    if (totalEntryCount > 0 && !entries)
    {
        if (logOpened)
        {
            f_puts("\tMemory alloc failed.\r\n", logFile);
            f_close(logFile);
        }
        f_close(iniFile);
        f_unlink(filepath);
        return;
    }

    // 为 [parameters] 段分配二进制条目数组
    HW_ParamInitEntry *paramEntries = nullptr;
    uint32_t paramEntryLen = 0;
    if (paramEntryCount > 0)
        paramEntries = (HW_ParamInitEntry *) pvPortMalloc(paramEntryCount * sizeof(HW_ParamInitEntry));

    uint16_t entryCount = 0;
    currentDevice = 0;

    while (true)
    {
        if (readLine(iniFile, lineBuf, HW_LINE_BUF_SIZE) == 0 && f_eof(iniFile))
            break;
        int pres = parseLine(lineBuf, secBuf, sizeof(secBuf), keyBuf, sizeof(keyBuf), valBuf, sizeof(valBuf));
        if (pres == 0)
        {
            if (f_eof(iniFile))
                break;
            continue;
        }
        if (pres == 1)
        {
            currentDevice = parseSectionDevice(secBuf);
        }
        else if (pres == 2 && currentDevice != 0)
        {
            if (currentDevice == 0xFFFF)
            {
                // [parameters] 段：解析 "name = default[, min[, max]]" 并填入二进制条目
                // parseLine 对 key 做了小写化，参数名区分大小写，需重新从 lineBuf 提取原始大小写 key
                if (paramEntries && paramEntryLen < paramEntryCount)
                {
                    const char *rawLine = skipSpaces(lineBuf);
                    const char *eqPos = strchr(rawLine, '=');
                    if (eqPos)
                    {
                        int rk = (int) (eqPos - rawLine);
                        while (rk > 0 && (rawLine[rk - 1] == ' ' || rawLine[rk - 1] == '\t'))
                            rk--;
                        if (rk > 0 && rk <= 16)
                        {
                            HW_ParamInitEntry &e = paramEntries[paramEntryLen++];
                            memset(e.name, 0, 16);
                            memcpy(e.name, rawLine, rk);
                            // valBuf 格式: "default[, min[, max]]"
                            e.defaultVal = (float) atof(valBuf);
                            e.minVal = 0.0f;
                            e.maxVal = 0.0f;
                            const char *comma1 = strchr(valBuf, ',');
                            if (comma1)
                            {
                                e.minVal = (float) atof(comma1 + 1);
                                const char *comma2 = strchr(comma1 + 1, ',');
                                if (comma2)
                                    e.maxVal = (float) atof(comma2 + 1);
                            }
                        }
                    }
                }
            }
            else if (currentDevice == 0xFFFE)
            {
                // [fixed_pins] 段: 存入 FuncMap, 稍后由 HW_ApplyFixedPins() 统一执行
                uint16_t port, pin;
                if (parsePin(keyBuf, &port, &pin) && port < 8 && pin <= 15)
                {
                    uint16_t state = (uint16_t) atoi(valBuf);
                    addEntry(entries, &entryCount, totalEntryCount, HW_DEV_FIXED_PIN, (uint16_t) ((port << 4) | pin), 0, state);
                }
            }
            else if (currentDevice == 0xFFFD)
            {
                // [exti_pins] 段: 存入 FuncMap, 稍后由 HW_ApplyExtiPins() 统一执行
                uint16_t port, pin, edge, pull;
                if (parsePin(keyBuf, &port, &pin) && port < 8 && pin <= 15 && parseExtiSpec(valBuf, &edge, &pull))
                    addEntry(entries, &entryCount, totalEntryCount, HW_DEV_EXTI_PIN, (uint16_t) ((port << 4) | pin), edge, pull);
            }
            else
            {
                parseKeyValue(keyBuf, valBuf, currentDevice, entries, &entryCount, totalEntryCount);
            }
        }

        if (f_eof(iniFile))
            break;
    }

    f_close(iniFile);

    // 写入InternalStorage
    if (entryCount > 0)
    {
        uint32_t dataSize = entryCount * sizeof(HW_ConfigEntry);
        uint32_t funcmap_crc = 0;
        SS_RESULT res = HW_SaveWithCRC("HardWare", "FuncMap", entries, dataSize, &funcmap_crc, 5.0);
        if (logOpened)
        {
            if (res == SS_OK)
                sprintf(logTxt, "\tConfig saved: %d entries, %lu bytes, FuncMap CRC32=0x%08lX.\r\n", entryCount, (unsigned long) dataSize, (unsigned long) funcmap_crc);
            else
                sprintf(logTxt, "\tInternalStorage write failed (err=%d).\r\n", (int) res);
            f_puts(logTxt, logFile);
        }
    }
    else
    {
        if (logOpened)
            f_puts("\tNo config entries parsed.\r\n", logFile);
    }

    // 保存 [parameters] 段二进制条目到 InternalStorage，供 InitialParametersInit 使用
    if (paramEntries && paramEntryLen > 0)
    {
        uint32_t paramSize = paramEntryLen * sizeof(HW_ParamInitEntry);
        uint32_t initparams_crc = 0;
        SS_RESULT res = HW_SaveWithCRC("HardWare", "InitParams", paramEntries, paramSize, &initparams_crc, 5.0);
        if (logOpened)
        {
            if (res == SS_OK)
            {
                sprintf(logTxt, "\tInitParams saved: %lu entries, %lu bytes, InitParams CRC32=0x%08lX.\r\nHardWare config load success.\r\n", (unsigned long) paramEntryLen, (unsigned long) paramSize,
                        (unsigned long) initparams_crc);
                f_puts(logTxt, logFile);
            }
        }
    }
    if (paramEntries)
        vPortFree(paramEntries);

    vPortFree(entries);

    if (logOpened)
        f_close(logFile);

    // 将 ini 文件复制到 log 挂载点的 ACFly 目录，先清除该目录下所有 ini 文件
    {
        // 目标目录: hw_log_mountPath + "ACFly"
        char dstDir[48];
        dstDir[0] = 0;
        strcat(dstDir, hw_log_mountPath);
        strcat(dstDir, "ACFly");

        // 删除目标目录下所有 *.ini
        {
            FILINFO delInfo;
            DIR delDir;
            if (f_findfirst(&delDir, &delInfo, dstDir, "*.ini") == FR_OK)
            {
                while (delInfo.fname[0] != 0)
                {
                    char delPath[80];
                    delPath[0] = 0;
                    strcat(delPath, dstDir);
                    strcat(delPath, "/");
                    strcat(delPath, delInfo.fname);
                    f_unlink(delPath);
                    if (f_findnext(&delDir, &delInfo) != FR_OK)
                        break;
                }
                f_closedir(&delDir);
            }
        }

        // 构造目标文件路径
        char dstPath[80];
        dstPath[0] = 0;
        strcat(dstPath, dstDir);
        strcat(dstPath, "/");
        strcat(dstPath, finfo.fname);

        // 重新打开源 ini 文件读取 (iniFile 此时已关闭)
        if (f_open(iniFile, filepath, FA_OPEN_EXISTING | FA_READ) == FR_OK)
        {
            // 打开目标文件写入 (logFile 此时已关闭)
            if (f_open(logFile, dstPath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
            {
                UINT br, bw;
                while (f_read(iniFile, lineBuf, sizeof(lineBuf), &br) == FR_OK && br > 0)
                    f_write(logFile, lineBuf, br, &bw);
                f_close(logFile);
            }
            f_close(iniFile);
        }

        // 删除源 ini 文件
        f_unlink(filepath);
    }
}

// FuncMap 全量缓存：首次调用时一次性读入 SRAM4 静态缓冲区，后续查找直接在 RAM 中进行
// 采用 SRAM4 静态分配（而非 new/pvPortMalloc），避免：
//   1) 跨线程调用（如 Commulink_Server 调 init_drv_can）时的 use-after-free 风险
//   2) HW_FreeConfigCache 释放后仍被使用的问题
//   3) 多任务同时首次加载导致的多重分配/泄漏
// 缓冲区永久驻留，一旦填充不再释放，所有路径变为无锁只读查询、天然线程安全。
// 12KB / sizeof(HW_ConfigEntry=8) = 1536 条容量，远超实际配置数量。
#define HW_FUNCMAP_CACHE_BYTES (12 * 1024)
Static_SRAM4Buf uint8_t g_hwFuncMapCacheRaw[HW_FUNCMAP_CACHE_BYTES];
static HW_ConfigEntry *const g_hwFuncMapCache = (HW_ConfigEntry *) g_hwFuncMapCacheRaw;
static uint32_t g_hwFuncMapCacheCount = 0; // 0 表示尚未加载（首次调用会触发加载）

/* ========== CRC 带校验的读写辅助 ==========
   存储格式：[原始 payload][uint32_t crc]（crc 覆盖 payload 全部字节）。

   HW_SaveWithCRC：WriteFileSpans 原子写入 payload + crc 两段（truncate=true 重建文件）。
                   用于 HardWareInit 读 INI 后新建 FuncMap / InitParams。
   HW_AppendCRC：  仅在文件末尾追加 4 字节 CRC（不重建文件）。
                   用于旧格式升级——读取时发现无 CRC，读完 payload 后追加 CRC 即可。

   读取侧采用内联 CRC 校验逻辑（见 HW_ReadConfigEntry / InitialParametersInit）：
     - 文件长度 == payload_len + 4（新格式）：3 次重读 + CRC 校验
     - 文件长度 == payload_len    （旧格式）：直接读 payload（不校验），再调 HW_AppendCRC 升级
     - 其它长度：视为损坏，调用者回退到默认
*/

static SS_RESULT HW_SaveWithCRC(const char *group, const char *name, const void *payload, uint32_t payload_len, uint32_t *out_crc, double TIMEOUT)
{
    if (payload_len > 0x10000) // CRC_Calc length 上限 65535
        return SS_ERR;

    uint32_t crc = 0;
    HW_CalcCRC((const uint8_t *) payload, payload_len, &crc);
    if (out_crc)
        *out_crc = crc;

    // 原子写入两段：[payload][crc]，truncate=true 表示重建文件
    StorageSpan spans[2];
    spans[0].content = payload;
    spans[0].offset = 0;
    spans[0].length = payload_len;
    spans[1].content = &crc;
    spans[1].offset = payload_len;
    spans[1].length = sizeof(uint32_t);
    return InternalStorage_WriteFileSpans(group, name, spans, 2, true, TIMEOUT);
}

// 仅在文件末尾追加 4 字节 CRC（不重建文件，用于旧格式无 CRC 文件升级）
// payload 为已读入 RAM 的原文件内容，用于计算 CRC
static SS_RESULT HW_AppendCRC(const char *group, const char *name, const void *payload, uint32_t payload_len, double TIMEOUT)
{
    if (payload_len > 0x10000)
        return SS_ERR;

    uint32_t crc = 0;
    HW_CalcCRC((const uint8_t *) payload, payload_len, &crc);
    // offset = payload_len 追加到末尾，truncate=false 不截断
    return InternalStorage_WriteFile(group, name, &crc, payload_len, sizeof(uint32_t), false, TIMEOUT);
}

bool HW_ReadConfigEntry(uint16_t device, uint16_t config_item, HW_ConfigEntry *entry)
{
    // 首次调用：一次性读取整个 FuncMap 到静态 SRAM4 缓冲（带 CRC 校验）
    if (g_hwFuncMapCacheCount == 0)
    {
        uint32_t fileSize = 0;
        if (InternalStorage_GetFileSize("HardWare", "FuncMap", &fileSize, 5.0) != SS_OK || fileSize == 0)
            return false;

        // 判定 payload 字节数 (文件可能为 [payload] 或 [payload][crc])
        const uint32_t entry_sz = sizeof(HW_ConfigEntry);
        const uint32_t crc_sz = sizeof(uint32_t);
        uint32_t payload_len = 0;

        bool has_crc = (fileSize >= crc_sz) && ((fileSize - crc_sz) % entry_sz == 0) && ((fileSize - crc_sz) > 0);
        bool no_crc = (fileSize % entry_sz == 0) && (fileSize > 0);

        if (has_crc)
            payload_len = fileSize - crc_sz;
        else if (no_crc)
            payload_len = fileSize;
        else
            return false;

        uint32_t count = payload_len / entry_sz;
        if (count == 0)
            return false;

        // 静态缓冲区容量上限校验（避免越界）
        // 注意: has_crc 路径下要读 fileSize = payload_len + 4 字节进缓冲区
        // 所以必须用 fileSize 做上限, 否则末尾 4 字节会颗超过缓冲区末尾
        // 例如 payload_len=12288(fileSize=12292) 会使 cache[12288..12291] 越界
        if (fileSize > HW_FUNCMAP_CACHE_BYTES)
            return false;

        HW_ConfigEntry *cache = g_hwFuncMapCache; // SRAM4 静态缓冲
        bool need_rewrite = false;

        if (has_crc)
        {
            // 新格式：3 次重读 + CRC 校验（读到静态缓冲，末尾 4 字节为 CRC）
            bool crc_pass = false;
            for (int attempt = 0; attempt < HW_CRC_MAX_RETRY && !crc_pass; ++attempt)
            {
                uint32_t got = fileSize;
                if (InternalStorage_ReadFile("HardWare", "FuncMap", cache, &got, 0, fileSize, 5.0) != SS_OK || got != fileSize)
                    break;
                uint32_t want = 0, calc = 0;
                memcpy(&want, (uint8_t *) cache + payload_len, crc_sz);
                HW_CalcCRC((const uint8_t *) cache, payload_len, &calc);
                crc_pass = (want == calc);
            }
            if (!crc_pass)
                return false;
            // cache 末尾 4 字节是 CRC，不作为合法条目；count 已设为 payload/setcount
        }
        else
        {
            // 旧格式：直接读 payload（不校验），读成功后回写带 CRC 文件
            uint32_t got = payload_len;
            if (InternalStorage_ReadFile("HardWare", "FuncMap", cache, &got, 0, payload_len, 5.0) != SS_OK || got != payload_len)
                return false;
            need_rewrite = true;
        }

        g_hwFuncMapCacheCount = count;

        // 旧格式补写 CRC：仅在末尾追加 4 字节，不重建文件
        // BootLoader 只读不改存储（避免文物损坏主固件写入的 FuncMap）
#ifndef HWCONFIG_READONLY
        if (need_rewrite)
            HW_AppendCRC("HardWare", "FuncMap", cache, payload_len, 5.0);
#endif
    }

    // 从 RAM 缓存中线性搜索
    for (uint32_t i = 0; i < g_hwFuncMapCacheCount; i++)
    {
        if (g_hwFuncMapCache[i].device == device && g_hwFuncMapCache[i].config_item == config_item)
        {
            *entry = g_hwFuncMapCache[i];
            return true;
        }
    }
    return false;
}

// 注意：自改为 SRAM4 静态分配后，cache 永久驻留、不再释放。
// 本函数保留作为空操作以兼容现有调用点（drv_Main.cpp 末尾调用），
// 不会真正释放内存。多线程竞态问题已通过静态分配 + 首次加载后只读的方式彻底消除。
void HW_FreeConfigCache()
{
    // no-op: 静态 SRAM4 缓冲不需释放永久驻留
}

// ===========================================================================
// HW_ApplyFixedPins: 执行 [fixed_pins] 段指定的 GPIO 固定输出
//   从 InternalStorage 的 FuncMap 中读取 device==HW_DEV_FIXED_PIN 的条目,
//   逐条配置为 STM32 GPIO 推挽输出并设定电平。
//   config_item = (port << 4) | pin
// ===========================================================================
void HW_ApplyFixedPins()
{
    // 触发缓存加载
    HW_ConfigEntry dummy;
    HW_ReadConfigEntry(0, 0xFFFF, &dummy);

    for (uint32_t i = 0; i < g_hwFuncMapCacheCount; i++)
    {
        const HW_ConfigEntry &e = g_hwFuncMapCache[i];
        if (e.device != HW_DEV_FIXED_PIN)
            continue;

        uint16_t port = (e.config_item >> 4) & 0xF;
        uint16_t pin = e.config_item & 0xF;
        if (port >= 8 || pin > 15)
            continue;
        uint16_t state = e.value_low;

        RCC->AHB4ENR |= (1u << port);
        (void) RCC->AHB4ENR; // 等待时钟使能生效
        GPIO_TypeDef *gpio = HW_GpioTable[port];
        gpio->OTYPER &= ~(1u << pin);
        gpio->PUPDR = (gpio->PUPDR & ~(3u << (pin * 2)));
        gpio->OSPEEDR = (gpio->OSPEEDR & ~(3u << (pin * 2)));
        if (state)
            gpio->BSRR = (1u << pin);
        else
            gpio->BSRR = (1u << (pin + 16));
        gpio->MODER = (gpio->MODER & ~(3u << (pin * 2))) | (1u << (pin * 2));
    }
}

// ===========================================================================
// [exti_pins] 段: GPIO 外部中断
//   HW_ApplyExtiPins 从 FuncMap 读取 device==HW_DEV_EXTI_PIN 的条目,
//   配置为 EXTI 输入中断 (GPIO输入+上下拉 → SYSCFG EXTICR → 边沿 → IMR1 → NVIC)。
//   config_item = (port << 4) | pin
//   value_high  = 触发沿 (HW_ExtiEdge), value_low = 上拉/下拉 (HW_Pull)
// ===========================================================================

// EXTI 回调注册表 [port][pin] (SRAM零初始化, 未注册为0)
static void (*s_extiCb[8][16])(void);

void HW_EXTI_RegisterCallback(uint16_t port, uint16_t pin, void (*cb)(void))
{
    if (port < 8 && pin < 16)
        s_extiCb[port][pin] = cb;
}

// EXTI 中断分发: lines = 本向量负责的EXTI线掩码
static void _extiDispatch(uint32_t lines)
{
    uint32_t pending = EXTI_D1->PR1 & lines;
    if (pending == 0)
        return;
    EXTI_D1->PR1 = pending; // rc_w1 清挂起标志
    for (uint8_t pin = 0; pin < 16; pin++)
    {
        if (pending & (1u << pin))
        {
            // 从EXTICR反查该线映射的GPIO端口
            uint8_t port = (SYSCFG->EXTICR[pin >> 2] >> (4u * (pin & 3u))) & 0xFu;
            if (pin == 15)
            {
                // pps时间同步
                // 在此开发对应的功能
            }
            if (port < 8 && s_extiCb[port][pin])
                s_extiCb[port][pin]();
        }
    }
}

extern "C" void EXTI0_IRQHandler(void)
{
    _extiDispatch(1u << 0);
}
extern "C" void EXTI1_IRQHandler(void)
{
    _extiDispatch(1u << 1);
}
extern "C" void EXTI2_IRQHandler(void)
{
    _extiDispatch(1u << 2);
}
extern "C" void EXTI3_IRQHandler(void)
{
    _extiDispatch(1u << 3);
}
extern "C" void EXTI4_IRQHandler(void)
{
    _extiDispatch(1u << 4);
}
extern "C" void EXTI9_5_IRQHandler(void)
{
    _extiDispatch(0x03E0u);
} // 线5~9
extern "C" void EXTI15_10_IRQHandler(void)
{
    _extiDispatch(0xFC00u);
} // 线10~15

void HW_ApplyExtiPins()
{
    // 触发缓存加载
    HW_ConfigEntry dummy;
    HW_ReadConfigEntry(0, 0xFFFF, &dummy);

    // SYSCFG时钟 (写EXTICR之前必须开启)
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    (void) RCC->APB4ENR;

    for (uint32_t i = 0; i < g_hwFuncMapCacheCount; i++)
    {
        const HW_ConfigEntry &e = g_hwFuncMapCache[i];
        if (e.device != HW_DEV_EXTI_PIN)
            continue;

        uint16_t port = (e.config_item >> 4) & 0xF;
        uint16_t pin = e.config_item & 0xF;
        if (port >= 8 || pin > 15)
            continue;
        uint16_t edge = e.value_high;
        uint16_t pull = e.value_low;

        // GPIO: 输入模式 + 上下拉
        RCC->AHB4ENR |= (1u << port);
        (void) RCC->AHB4ENR; // 等待时钟使能生效
        GPIO_TypeDef *gpio = HW_GpioTable[port];
        gpio->MODER &= ~(3u << (pin * 2)); // 00: 输入 (脱离JTAG等复用功能)
        gpio->PUPDR = (gpio->PUPDR & ~(3u << (pin * 2))) | ((uint32_t) pull << (pin * 2));

        // SYSCFG EXTICR: 线pin映射到port
        SYSCFG->EXTICR[pin >> 2] = (SYSCFG->EXTICR[pin >> 2] & ~(0xFu << (4u * (pin & 3u)))) | ((uint32_t) port << (4u * (pin & 3u)));

        // 触发沿选择
        uint32_t bit = 1u << pin;
        if (edge == HW_EXTI_RISING || edge == HW_EXTI_BOTH)
            EXTI->RTSR1 |= bit;
        else
            EXTI->RTSR1 &= ~bit;
        if (edge == HW_EXTI_FALLING || edge == HW_EXTI_BOTH)
            EXTI->FTSR1 |= bit;
        else
            EXTI->FTSR1 &= ~bit;

        // 清挂起标志并使能中断 (D1域 CM7)
        EXTI_D1->PR1 = bit;
        EXTI_D1->IMR1 |= bit;

        // 使能NVIC向量 (优先级5, 与PPM/输入捕获一致)
        IRQn_Type irq;
        if (pin <= 4)
            irq = (IRQn_Type) (EXTI0_IRQn + pin);
        else if (pin <= 9)
            irq = EXTI9_5_IRQn;
        else
            irq = EXTI15_10_IRQn;
        NVIC_SetPriority(irq, 5);
        NVIC_EnableIRQ(irq);
    }
}

// ===========================================================================
// 参数初始值初始化：从 InternalStorage 读取 [parameters] 段覆盖初始参数数组
// 在调用 ParamGroupRegister 前调用，可将 ini 中指定的初始值写入 initial_params
// initial_params: 已按编译默认值初始化的参数数组 (与 ParamGroupRegister 传入的同一数组)
// param_types:    各参数的 MAV_PARAM_TYPE 类型数组
// param_names:    各参数的 SName 名称数组
// count:          参数个数
// ===========================================================================
#ifndef HWCONFIG_READONLY
void InitialParametersInit(uint64_t initial_params[], const MAV_PARAM_TYPE param_types[], const SName param_names[], uint32_t count)
{
    uint32_t fileSize = 0;
    if (InternalStorage_GetFileSize("HardWare", "InitParams", &fileSize, 5.0) != SS_OK || fileSize == 0)
        return;

    // 判定 payload 字节数 (文件可能为 [payload] 或 [payload][crc])
    const uint32_t entry_sz = sizeof(HW_ParamInitEntry);
    const uint32_t crc_sz = sizeof(uint32_t);

    bool has_crc = (fileSize >= crc_sz) && ((fileSize - crc_sz) % entry_sz == 0) && ((fileSize - crc_sz) > 0);
    bool no_crc = (fileSize % entry_sz == 0) && (fileSize > 0);
    if (!has_crc && !no_crc)
        return;

    uint32_t payload_len = has_crc ? (fileSize - crc_sz) : fileSize;
    if (payload_len % entry_sz != 0 || payload_len == 0)
        return;

    HW_ParamInitEntry *entries = (HW_ParamInitEntry *) pvPortMalloc(payload_len);
    if (!entries)
        return;

    bool read_ok = false;
    bool need_rewrite = false;

    if (has_crc)
    { // 新格式：3 次重读 + CRC 校验
        uint8_t *fbuf = (uint8_t *) pvPortMalloc(fileSize);
        if (fbuf)
        {
            for (int attempt = 0; attempt < HW_CRC_MAX_RETRY && !read_ok; ++attempt)
            {
                uint32_t got = fileSize;
                if (InternalStorage_ReadFile("HardWare", "InitParams", fbuf, &got, 0, fileSize, 5.0) != SS_OK || got != fileSize)
                    break;
                uint32_t want = 0, calc = 0;
                memcpy(&want, fbuf + payload_len, crc_sz);
                HW_CalcCRC(fbuf, payload_len, &calc);
                if (want == calc)
                {
                    memcpy(entries, fbuf, payload_len);
                    read_ok = true;
                }
            }
            vPortFree(fbuf);
        }
    }
    else
    { // 旧格式：直接读 payload（不校验）
        uint32_t got = payload_len;
        if (InternalStorage_ReadFile("HardWare", "InitParams", entries, &got, 0, payload_len, 5.0) == SS_OK && got == payload_len)
        {
            read_ok = true;
            need_rewrite = true;
        }
    }

    if (!read_ok)
    {
        vPortFree(entries);
        return;
    }

    // 旧格式补写 CRC：仅在末尾追加 4 字节，不重建文件
    // BootLoader 只读不改存储（避免文物损坏主固件写入的 InitParams）
#ifndef HWCONFIG_READONLY
    if (need_rewrite)
        HW_AppendCRC("HardWare", "InitParams", entries, payload_len, 5.0);
#endif

    uint32_t actualCount = payload_len / sizeof(HW_ParamInitEntry);

    for (uint32_t e = 0; e < actualCount; e++)
    {
        // 构造 SName 用于比较（name[16] 与 SName 二进制兼容）
        SName key(entries[e].name);
        float fv = entries[e].defaultVal;

        for (uint32_t i = 0; i < count; i++)
        {
            if (param_names[i] == key)
            {
                switch (param_types[i])
                {
                    case MAV_PARAM_TYPE_REAL32:
                    {
                        uint32_t uv;
                        memcpy(&uv, &fv, 4);
                        initial_params[i] = (uint64_t) uv | ((uint64_t) uv << 32);
                        break;
                    }
                    case MAV_PARAM_TYPE_REAL64:
                    {
                        double dv = (double) fv;
                        memcpy(&initial_params[i], &dv, 8);
                        break;
                    }
                    case MAV_PARAM_TYPE_UINT8:
                    {
                        uint64_t uv = (uint8_t) (uint32_t) fv;
                        uv |= (uv << 8) | (uv << 16) | (uv << 24) | (uv << 32) | (uv << 40) | (uv << 48) | (uv << 56);
                        initial_params[i] = uv;
                        break;
                    }
                    case MAV_PARAM_TYPE_INT8:
                    {
                        uint64_t uv = (uint8_t) (int8_t) (int32_t) fv;
                        uv |= (uv << 8) | (uv << 16) | (uv << 24) | (uv << 32) | (uv << 40) | (uv << 48) | (uv << 56);
                        initial_params[i] = uv;
                        break;
                    }
                    case MAV_PARAM_TYPE_UINT16:
                    {
                        uint64_t uv = (uint16_t) (uint32_t) fv;
                        initial_params[i] = uv | (uv << 16) | (uv << 32) | (uv << 48);
                        break;
                    }
                    case MAV_PARAM_TYPE_INT16:
                    {
                        uint64_t uv = (uint16_t) (int16_t) (int32_t) fv;
                        initial_params[i] = uv | (uv << 16) | (uv << 32) | (uv << 48);
                        break;
                    }
                    case MAV_PARAM_TYPE_UINT32:
                    {
                        uint64_t uv = (uint32_t) fv;
                        initial_params[i] = uv | (uv << 32);
                        break;
                    }
                    case MAV_PARAM_TYPE_INT32:
                    {
                        uint64_t uv = (uint32_t) (int32_t) fv;
                        initial_params[i] = uv | (uv << 32);
                        break;
                    }
                    case MAV_PARAM_TYPE_UINT64:
                    {
                        initial_params[i] = (uint64_t) (uint32_t) fv;
                        break;
                    }
                    case MAV_PARAM_TYPE_INT64:
                    {
                        initial_params[i] = (uint64_t) (int64_t) (int32_t) fv;
                        break;
                    }
                    default:
                        break;
                }
                break; // 找到匹配项后退出内层循环
            }
        }
    }

    vPortFree(entries);
}
#endif // !HWCONFIG_READONLY

// ===========================================================================
// 定时器辅助函数
// ===========================================================================
TIM_TypeDef *HW_GetTimerPtr(uint16_t n)
{
    switch (n)
    {
        case 1:
            return TIM1;
        case 2:
            return TIM2;
        case 3:
            return TIM3;
        case 4:
            return TIM4;
        case 5:
            return TIM5;
        case 6:
            return TIM6;
        case 7:
            return TIM7;
        case 8:
            return TIM8;
        case 12:
            return TIM12;
        case 13:
            return TIM13;
        case 14:
            return TIM14;
        case 15:
            return TIM15;
        case 16:
            return TIM16;
        case 17:
            return TIM17;
        default:
            return nullptr;
    }
}

void HW_EnableTimerClock(uint16_t n)
{
    switch (n)
    {
        case 1:
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
            break;
        case 2:
            RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
            break;
        case 3:
            RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
            break;
        case 4:
            RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
            break;
        case 5:
            RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
            break;
        case 6:
            RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
            break;
        case 7:
            RCC->APB1LENR |= RCC_APB1LENR_TIM7EN;
            break;
        case 8:
            RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
            break;
        case 12:
            RCC->APB1LENR |= RCC_APB1LENR_TIM12EN;
            break;
        case 13:
            RCC->APB1LENR |= RCC_APB1LENR_TIM13EN;
            break;
        case 14:
            RCC->APB1LENR |= RCC_APB1LENR_TIM14EN;
            break;
        case 15:
            RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
            break;
        case 16:
            RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
            break;
        case 17:
            RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
            break;
        default:
            break;
    }
}
