#include "drv_ADC.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "Parameters.hpp"
#include "event_groups.h"
#include "math.h"
#include "semphr.h"
#include "task.h"

// VREFINT校准值
uint16_t VREFINT_CAL = *(volatile uint16_t *) 0x1FF1E860;
// 温度校准值
uint16_t TS_CAL1 = *(volatile uint16_t *) 0x1FF1E820;
uint16_t TS_CAL2 = *(volatile uint16_t *) 0x1FF1E840;

// ADC通道定义
struct ADC_ChannelDef
{
    ADC_TypeDef *adc;    // ADC外设指针
    uint8_t channel;     // 通道号
    uint8_t sample_time; // 采样时间 0~7
    uint16_t port;       // GPIO端口 (HW_Port)
    uint16_t pin;        // GPIO引脚号
    bool enabled;        // 是否启用
};

// 最大用户通道数 (bat1_volt, bat2_volt, bat1_current, bat2_current)
#define ADC_USER_CH_MAX 4
// 内部通道 (VREFINT=ADC3_CH19, TEMP=ADC3_CH18)
#define ADC_INTERNAL_CH 2
#define ADC_TOTAL_MAX (ADC_USER_CH_MAX + ADC_INTERNAL_CH)

// 通道索引
enum
{
    CH_BAT1_VOLT = 0,
    CH_BAT2_VOLT = 1,
    CH_BAT1_CUR = 2,
    CH_BAT2_CUR = 3,
    CH_VREFINT = 4,
    CH_TEMP = 5,
};

// 转换结果存储
static volatile uint32_t adc_data[ADC_TOTAL_MAX] = {0};
// 通道定义表
static ADC_ChannelDef adc_channels[ADC_TOTAL_MAX];
// 有效通道数
static uint8_t adc_channel_count = 0;
// 当前正在转换的通道索引
static volatile uint8_t adc_current_seq = 0;
// 配置分辨率
static uint8_t adc_resolution_bits = 16;
// 过采样倍率
static uint16_t adc_oversampling = 1024;

// ADC实例查找表
static ADC_TypeDef *get_adc_instance(uint8_t adc_num)
{
    switch (adc_num)
    {
        case 1:
            return ADC1;
        case 2:
            return ADC2;
        case 3:
            return ADC3;
        default:
            return 0;
    }
}

// 获取ADC COMMON寄存器
static ADC_Common_TypeDef *get_adc_common(ADC_TypeDef *adc)
{
    if (adc == ADC3)
        return ADC3_COMMON;
    return ADC12_COMMON;
}

// 根据分辨率获取CFGR RES位
static uint8_t get_res_bits(uint8_t bits)
{
    switch (bits)
    {
        case 8:
            return 0b111;
        case 10:
            return 0b011;
        case 12:
            return 0b010;
        case 14:
            return 0b001;
        case 16:
        default:
            return 0b000;
    }
}

// 计算过采样移位量 (右移位数, 使结果不溢出32位)
static uint8_t get_oversampling_shift(uint16_t ratio)
{
    uint8_t shift = 0;
    uint16_t r = ratio;
    while (r > 1)
    {
        r >>= 1;
        shift++;
    }
    return shift;
}

// 获取满量程值
static float get_full_scale()
{
    switch (adc_resolution_bits)
    {
        case 8:
            return 255.0f;
        case 10:
            return 1023.0f;
        case 12:
            return 4095.0f;
        case 14:
            return 16383.0f;
        case 16:
        default:
            return 65535.0f;
    }
}

/*
  获取电池电流大小,单位：A
*/
float adcGet_MainBaterry1_Current()
{
    if (!adc_channels[CH_BAT1_CUR].enabled)
        return 0;
    float kp[2];
    if (ReadParam("Bat_CurrentMKp", 0, 0, (uint64_t *) kp, 0) == PR_OK)
        return kp[0] * ((float) adc_data[CH_BAT1_CUR] * 3.3f / get_full_scale());
    return 0;
}

float adcGet_MainBaterry2_Current()
{
    if (!adc_channels[CH_BAT2_CUR].enabled)
        return 0;
    float kp[2];
    if (ReadParam("Bat2_CurMKp", 0, 0, (uint64_t *) kp, 0) == PR_OK)
        return kp[0] * ((float) adc_data[CH_BAT2_CUR] * 3.3f / get_full_scale());
    return 0;
}

/*
  获取电池电压,单位；V
*/
float adcGet_MainBaterry1_Voltage()
{
    if (!adc_channels[CH_BAT1_VOLT].enabled)
        return 0;
    float kp[2];
    if (ReadParam("Bat_VoltMKp", 0, 0, (uint64_t *) kp, 0) == PR_OK)
        return kp[0] * (float) adc_data[CH_BAT1_VOLT] * 3.3f / get_full_scale();
    return 0;
}
float adcGet_MainBaterry2_Voltage()
{
    if (!adc_channels[CH_BAT2_VOLT].enabled)
        return 0;
    float kp[2];
    if (ReadParam("Bat2_VoltMKp", 0, 0, (uint64_t *) kp, 0) == PR_OK)
        return kp[0] * (float) adc_data[CH_BAT2_VOLT] * 3.3f / get_full_scale();
    return 0;
}

/*
  获取VDDA基准电压,单位；V
*/
float adcGet_VDDA_Voltage()
{
    if (adc_data[CH_VREFINT] == 0)
        return 3.3f;
    return (3.3f * (float) VREFINT_CAL / (float) adc_data[CH_VREFINT]);
}

/*
  获取温度，单位：℃
*/
float adcGet_CPUTemperature()
{
    return (110.0f - 30.0f) / (TS_CAL2 - TS_CAL1) * ((float) adc_data[CH_TEMP] - TS_CAL1) + 30.0f;
}

// 启动下一通道转换
static void adc_start_next_conversion()
{
    // 找到下一个有效通道
    for (uint8_t tries = 0; tries < ADC_TOTAL_MAX; tries++)
    {
        if (adc_current_seq >= ADC_TOTAL_MAX)
            adc_current_seq = 0;
        if (adc_channels[adc_current_seq].enabled)
        {
            ADC_TypeDef *adc = adc_channels[adc_current_seq].adc;
            uint8_t ch = adc_channels[adc_current_seq].channel;
            // 设置单通道序列: SQR1 L=0(1次转换), SQ1=channel
            adc->SQR1 = (ch << 6) | (0 << 0);
            // PCSEL 预选通道
            adc->PCSEL = (1 << ch);
            // 设置采样时间
            uint8_t smpt = adc_channels[adc_current_seq].sample_time & 0x7;
            if (ch < 10)
            {
                uint32_t smpr1 = adc->SMPR1;
                smpr1 &= ~(0x7 << (ch * 3));
                smpr1 |= (smpt << (ch * 3));
                adc->SMPR1 = smpr1;
            }
            else
            {
                uint32_t smpr2 = adc->SMPR2;
                smpr2 &= ~(0x7 << ((ch - 10) * 3));
                smpr2 |= (smpt << ((ch - 10) * 3));
                adc->SMPR2 = smpr2;
            }
            // 启动转换
            adc->CR |= (1 << 2); // ADSTART
            return;
        }
        adc_current_seq++;
    }
}

// 记录哪些ADC已初始化
static bool adc_inited[4] = {false, false, false, false}; // index 1~3

// ADC初始化 (单个ADC外设)
static void adc_hw_init(ADC_TypeDef *adc, uint8_t adc_num)
{
    if (adc_num < 1 || adc_num > 3)
        return;
    if (adc_inited[adc_num])
        return;
    adc_inited[adc_num] = true;

    // 使能RCC时钟
    if (adc_num <= 2)
        RCC->AHB1ENR |= (1 << 5); // ADC12EN
    else
        RCC->AHB4ENR |= (1 << 24); // ADC3EN
    delay(1e-2);

    // ADC退出深度掉电模式, 使能稳压器
    adc->CR = 0;
    adc->CR = (1 << 28); // ADVREGEN
    delay(1e-2);
    while ((adc->CR & (1 << 28)) != (1 << 28))
    {
    };

    // 选择ADC时钟
    ADC_Common_TypeDef *common = get_adc_common(adc);
    common->CCR = (1 << 23) | (1 << 22) | (0b0000 << 18) | (0b00 << 16);

    // 校准 (单端输入线性校准)
    adc->CR |= (0b11 << 8); // BOOST=11 (fADC > 25MHz)
    delay(1e-2);
    adc->CR &= ~(1 << 30); // ADCALDIF=0, 单端
    adc->CR |= (1 << 16);  // ADCALLIN=1, 线性校准
    adc->CR |= (1 << 31);  // ADCAL
    delay(1e-2);
    while ((adc->CR & (1 << 31)) == (1 << 31))
    {
    };

    // 配置分辨率和过采样
    uint8_t res = get_res_bits(adc_resolution_bits);
    uint8_t shift = get_oversampling_shift(adc_oversampling);
    // CFGR: JQDIS=1, RES, 非连续转换
    adc->CFGR = (1 << 31) | (res << 2);
    // CFGR2: 过采样配置
    if (adc_oversampling > 1)
    {
        uint16_t ratio_reg = adc_oversampling - 1;
        adc->CFGR2 = ((uint32_t) ratio_reg << 16) | (shift << 5) | (1 << 0); // ROVSE
    }
    else
        adc->CFGR2 = 0;

    // 打开ADC
    adc->ISR = adc->ISR; // 清中断标志
    adc->CR |= (1 << 0); // ADEN
    delay(1e-2);
    while ((adc->ISR & (1 << 0)) != (1 << 0))
        ;
    adc->ISR = adc->ISR;

    // 使能EOC中断
    adc->IER = (1 << 2); // EOCIE
    if (adc_num <= 2)
    {
        NVIC_SetPriority(ADC_IRQn, 5);
        NVIC_EnableIRQ(ADC_IRQn);
    }
    else
    {
        NVIC_SetPriority(ADC3_IRQn, 5);
        NVIC_EnableIRQ(ADC3_IRQn);
    }
}

// 解析配置中的一个通道
static bool parse_channel_config(uint8_t ch_idx, HW_ConfigItem cfg_ch, HW_ConfigItem cfg_pin, HW_ConfigItem cfg_smpt)
{
    HW_ConfigEntry entry;
    if (!HW_ReadConfigEntry(HW_DEV_ADC, cfg_ch, &entry))
        return false;

    uint8_t adc_num = entry.value_high;
    uint8_t channel = entry.value_low;
    ADC_TypeDef *adc = get_adc_instance(adc_num);
    if (!adc)
        return false;

    adc_channels[ch_idx].adc = adc;
    adc_channels[ch_idx].channel = channel;
    adc_channels[ch_idx].enabled = true;

    // 引脚配置
    if (HW_ReadConfigEntry(HW_DEV_ADC, cfg_pin, &entry))
    {
        adc_channels[ch_idx].port = entry.value_high;
        adc_channels[ch_idx].pin = entry.value_low;
        // 配置GPIO为模拟输入
        RCC->AHB4ENR |= (1 << adc_channels[ch_idx].port);
        delay(1e-2);
        GPIO_TypeDef *gpio = HW_GpioTable[adc_channels[ch_idx].port];
        set_register(gpio->MODER, 0b11, adc_channels[ch_idx].pin * 2, 2); // 模拟模式
    }

    // 采样时间
    adc_channels[ch_idx].sample_time = 7; // 默认最大
    if (HW_ReadConfigEntry(HW_DEV_ADC, cfg_smpt, &entry))
        adc_channels[ch_idx].sample_time = entry.value_low & 0x7;

    // 初始化该ADC外设
    adc_hw_init(adc, adc_num);
    adc_channel_count++;
    return true;
}

// ADC EOC公共处理
static void adc_eoc_handler(ADC_TypeDef *adc)
{
    if (adc->ISR & (1 << 2)) // EOC
    {
        uint32_t data = adc->DR;
        // 检测是否为当前通道的ADC
        if (adc_current_seq < ADC_TOTAL_MAX && adc_channels[adc_current_seq].enabled && adc_channels[adc_current_seq].adc == adc)
        {
            adc_data[adc_current_seq] = data;
            adc_current_seq++;
            adc_start_next_conversion();
        }
        adc->ISR = (1 << 2); // 清EOC标志
    }
}

extern "C" void ADC_IRQHandler(void)
{
    // ADC1/2共享此中断
    if (ADC1->ISR & (1 << 2))
        adc_eoc_handler(ADC1);
    if (ADC2->ISR & (1 << 2))
        adc_eoc_handler(ADC2);
}

extern "C" void ADC3_IRQHandler(void)
{
    adc_eoc_handler(ADC3);
}

void init_drv_ADC(void)
{
    // 初始化通道表
    for (uint8_t i = 0; i < ADC_TOTAL_MAX; i++)
    {
        adc_channels[i].adc = 0;
        adc_channels[i].channel = 0;
        adc_channels[i].sample_time = 7;
        adc_channels[i].port = 0;
        adc_channels[i].pin = 0;
        adc_channels[i].enabled = false;
    }

    // 读取全局配置
    HW_ConfigEntry entry;
    if (HW_ReadConfigEntry(HW_DEV_ADC, HW_CFG_ADC_RESOLUTION, &entry))
        adc_resolution_bits = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_ADC, HW_CFG_ADC_OVERSAMPLING, &entry))
        adc_oversampling = ((uint32_t) entry.value_high << 16) | entry.value_low;

    // 使能GPIO C口时钟 (常用ADC引脚端口)
    RCC->AHB4ENR |= (1 << 2);
    delay(1e-2);

    // 解析用户通道
    parse_channel_config(CH_BAT1_VOLT, HW_CFG_ADC_BAT1_VOLT, HW_CFG_ADC_BAT1_VOLT_PIN, HW_CFG_ADC_BAT1_VOLT_SMPT);
    parse_channel_config(CH_BAT2_VOLT, HW_CFG_ADC_BAT2_VOLT, HW_CFG_ADC_BAT2_VOLT_PIN, HW_CFG_ADC_BAT2_VOLT_SMPT);
    parse_channel_config(CH_BAT1_CUR, HW_CFG_ADC_BAT1_CUR, HW_CFG_ADC_BAT1_CUR_PIN, HW_CFG_ADC_BAT1_CUR_SMPT);
    parse_channel_config(CH_BAT2_CUR, HW_CFG_ADC_BAT2_CUR, HW_CFG_ADC_BAT2_CUR_PIN, HW_CFG_ADC_BAT2_CUR_SMPT);

    // 添加内部通道: VREFINT (ADC3_CH19) 和 温度 (ADC3_CH18) 固定使用ADC3
    adc_channels[CH_VREFINT].adc = ADC3;
    adc_channels[CH_VREFINT].channel = 19;
    adc_channels[CH_VREFINT].sample_time = 7;
    adc_channels[CH_VREFINT].enabled = true;
    adc_channel_count++;

    adc_channels[CH_TEMP].adc = ADC3;
    adc_channels[CH_TEMP].channel = 18;
    adc_channels[CH_TEMP].sample_time = 7;
    adc_channels[CH_TEMP].enabled = true;
    adc_channel_count++;

    // 确保ADC3已初始化 (内部通道需要)
    adc_hw_init(ADC3, 3);

    // 使能VREFINT和温度传感器 (ADC3_COMMON CCR)
    ADC3_COMMON->CCR |= (1 << 23) | (1 << 22); // VREFEN + TSEN

    // 启动第一次转换
    adc_current_seq = 0;
    adc_start_next_conversion();
}