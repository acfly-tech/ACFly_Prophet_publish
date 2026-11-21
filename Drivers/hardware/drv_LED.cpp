#include "drv_LED.hpp"
#include "AC_Math.hpp"
#include "Basic.hpp"
#include "ControlSystem.hpp"
#include "HardWareConfig.hpp"
#include "TimeBase.h"
#include "stm32h743xx.h"

// ===========================================================================
// LED / 蜂鸣器驱动
// 运行时从 HardWareConfig ([led] 段) 读取 timer、引脚、通道和极性配置。
// 若 INI 中无 [led] 配置，回退到硬编码默认值 (TIM2, PA0~PA3)。
// ===========================================================================

// ---------- 运行时状态 ----------
static TIM_TypeDef *g_tim = nullptr; // LED / 蜂鸣器使用的定时器
static uint8_t g_ch_buz = 1;         // 蜂鸣器通道 (1~4)
static uint8_t g_ch_r = 4;           // 红色 LED 通道
static uint8_t g_ch_g = 3;           // 绿色 LED 通道
static uint8_t g_ch_b = 2;           // 蓝色 LED 通道

// 外部亮度缓存 (供其他模块读取)
float ExtLedR = 0;
float ExtLedG = 0;
float ExtLedB = 0;

// ---------- 内部辅助函数 ----------

// 根据通道号获取 CCR 寄存器指针 (TIM CCR1~CCR4 在内存中连续)
static inline volatile uint32_t *get_ccr(TIM_TypeDef *tim, uint8_t ch)
{
    return &(tim->CCR1) + (ch - 1);
}

// 配置单路 PWM 通道 (CCMR + CCER)
// inverted=true 使用 PWM2 模式并置位极性位 → 低电平有效 (共阳/漏极开路)
static void tim_ch_pwm_init(TIM_TypeDef *tim, uint8_t ch, bool inverted)
{
    // CCMR: 奇数通道 (1,3) 使用 CCMR1/CCMR2 的低字节; 偶数通道使用高字节
    // 字节偏移: CH1→0, CH2→8, CH3→0(CCMR2), CH4→8(CCMR2)
    volatile uint32_t *ccmr = (ch <= 2) ? &tim->CCMR1 : &tim->CCMR2;
    uint32_t shift = ((ch - 1) & 1) ? 8 : 0;
    uint32_t ocm = 0b111u; // PWM2 (inverted) or PWM1
    *ccmr &= ~(0xFFu << shift);
    *ccmr |= ((ocm << 4) | (1u << 3)) << shift; // OCnM[6:4] + OCnPE[3]

    // CCER: 使能位 = (ch-1)*4, 极性位 = (ch-1)*4+1
    uint32_t ccer_en = 1u << ((ch - 1) * 4);
    uint32_t ccer_pol = 1u << ((ch - 1) * 4 + 1);
    tim->CCER |= ccer_en;
    if (inverted)
        tim->CCER |= ccer_pol;
    else
        tim->CCER &= ~ccer_pol;
}

// 初始化一个 GPIO 引脚为 AF 复用推挽输出
static void gpio_af_init(uint16_t port, uint16_t pin, uint16_t af)
{
    RCC->AHB4ENR |= (1u << port);
    os_delay(1e-4);
    GPIO_TypeDef *gpio = HW_GpioTable[port];
    gpio->OTYPER &= ~(1u << pin);
    set_register(gpio->OSPEEDR, 0b11u, pin * 2, 2);
    if (pin < 8)
        set_register(gpio->AFR[0], af, pin * 4, 4);
    else
        set_register(gpio->AFR[1], af, (pin - 8) * 4, 4);
    set_register(gpio->MODER, 0b10u, pin * 2, 2);
}

// ===========================================================================
// 公开接口
// ===========================================================================

/*
    LED 亮度设置
    R、G、B: 亮度百分比 (0~100), 负数表示不更新该通道
*/
void set_LedBrightness(float R, float G, float B)
{
    if (!g_tim)
        return;
    if (R >= 0 && R <= 100)
    {
        ExtLedR = R;
        *get_ccr(g_tim, g_ch_r) = (uint32_t) (powf(R / 100.0f, 3.0f) * (float) g_tim->ARR);
    }
    if (G >= 0 && G <= 100)
    {
        ExtLedG = G;
        *get_ccr(g_tim, g_ch_g) = (uint32_t) (powf(G / 100.0f, 3.0f) * (float) g_tim->ARR);
    }
    if (B >= 0 && B <= 100)
    {
        ExtLedB = B;
        *get_ccr(g_tim, g_ch_b) = (uint32_t) (powf(B / 100.0f, 3.0f) * (float) g_tim->ARR);
    }
}

/*
    蜂鸣器频率调节
    freq: 目标频率 (Hz), 最低 200 Hz
    调用本函数时 ARR 随之变化，LED 亮度比例保持不变。
*/
void set_BuzzerFreq(unsigned short freq)
{
    if (!g_tim)
        return;
    // 保留各 LED 通道当前占空比比例
    float br = (float) *get_ccr(g_tim, g_ch_r) / (float) g_tim->ARR;
    float bg = (float) *get_ccr(g_tim, g_ch_g) / (float) g_tim->ARR;
    float bb = (float) *get_ccr(g_tim, g_ch_b) / (float) g_tim->ARR;

    if (freq < 200)
        freq = 200;
    bool buz_on = (*get_ccr(g_tim, g_ch_buz) != 0); // 记录蜂鸣器当前是否开启
    g_tim->ARR = (uint32_t) (10000000.0f / (float) freq);
    // 仅在蜂鸣器已开启时更新 CCR，避免误开蜂鸣器
    if (buz_on)
        *get_ccr(g_tim, g_ch_buz) = g_tim->ARR / 2;

    // 恢复 LED 亮度
    *get_ccr(g_tim, g_ch_r) = (uint32_t) (br * (float) g_tim->ARR);
    *get_ccr(g_tim, g_ch_g) = (uint32_t) (bg * (float) g_tim->ARR);
    *get_ccr(g_tim, g_ch_b) = (uint32_t) (bb * (float) g_tim->ARR);
}

/*
    蜂鸣器开关
    on: true=鸣响 (仅在非飞行状态有效)
*/
void set_BuzzerOnOff(bool on)
{
    if (!g_tim)
        return;
    bool inFlight;
    get_is_inFlight(&inFlight);
    *get_ccr(g_tim, g_ch_buz) = (on && !inFlight) ? (g_tim->ARR / 2) : 0;
}

// ===========================================================================
// 初始化
// ===========================================================================
void init_drv_LED(void)
{
    HW_ConfigEntry entry;

    // ------ 读取定时器编号 ------
    uint16_t timer_num = 2; // 默认 TIM2
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_TIMER, &entry))
        timer_num = entry.value_high;

    g_tim = HW_GetTimerPtr(timer_num);
    if (!g_tim)
        return;

    // 各通道配置结构体
    struct ChCfg
    {
        uint16_t port;
        uint16_t pin;
        uint16_t af;
        uint8_t ch;
        bool inv;
    };

    // 默认值 (与 K9_2025V1.ini 对应: PA0=Buzzer/CH1, PA1=B/CH2, PA2=G/CH3, PA3=R/CH4)
    ChCfg buzzer = {HW_PORT_PA, 0, 1, 1, true};
    ChCfg red = {HW_PORT_PA, 3, 1, 4, true};
    ChCfg green = {HW_PORT_PA, 2, 1, 3, true};
    ChCfg blue = {HW_PORT_PA, 1, 1, 2, true};

    // ------ 从配置文件读取各通道 ------
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_BUZZER_PIN, &entry))
    {
        buzzer.port = entry.value_high;
        buzzer.pin = entry.value_low;
    }
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_BUZZER_AF, &entry))
        buzzer.af = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_BUZZER_CH, &entry))
        buzzer.ch = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_BUZZER_INV, &entry))
        buzzer.inv = (entry.value_low != 0);

    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_R_PIN, &entry))
    {
        red.port = entry.value_high;
        red.pin = entry.value_low;
    }
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_R_AF, &entry))
        red.af = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_R_CH, &entry))
        red.ch = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_R_INV, &entry))
        red.inv = (entry.value_low != 0);

    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_G_PIN, &entry))
    {
        green.port = entry.value_high;
        green.pin = entry.value_low;
    }
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_G_AF, &entry))
        green.af = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_G_CH, &entry))
        green.ch = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_G_INV, &entry))
        green.inv = (entry.value_low != 0);

    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_B_PIN, &entry))
    {
        blue.port = entry.value_high;
        blue.pin = entry.value_low;
    }
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_B_AF, &entry))
        blue.af = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_B_CH, &entry))
        blue.ch = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_LED, HW_CFG_LED_B_INV, &entry))
        blue.inv = (entry.value_low != 0);

    // 保存通道号到全局状态
    g_ch_buz = buzzer.ch;
    g_ch_r = red.ch;
    g_ch_g = green.ch;
    g_ch_b = blue.ch;

    // ------ 配置 GPIO 引脚 ------
    gpio_af_init(buzzer.port, buzzer.pin, buzzer.af);
    gpio_af_init(red.port, red.pin, red.af);
    gpio_af_init(green.port, green.pin, green.af);
    gpio_af_init(blue.port, blue.pin, blue.af);

    // ------ 使能定时器时钟 ------
    HW_EnableTimerClock(timer_num);
    os_delay(1e-2);

    // 定时器预分频: 10 MHz 计数频率 (APB1/APB2 timer clock 均为 APB1TIMERCLK)
    g_tim->PSC = (APB1TIMERCLK / 10000000) - 1;

    // 默认 ARR: 1 kHz PWM 周期
    g_tim->ARR = 10000000 / 1000;

    // 清零所有 CCR
    g_tim->CCR1 = g_tim->CCR2 = g_tim->CCR3 = g_tim->CCR4 = 0;

    // ------ 配置各 PWM 通道 ------
    tim_ch_pwm_init(g_tim, buzzer.ch, buzzer.inv); // 蜂鸣器极性由配置决定
    tim_ch_pwm_init(g_tim, red.ch, red.inv);
    tim_ch_pwm_init(g_tim, green.ch, green.inv);
    tim_ch_pwm_init(g_tim, blue.ch, blue.inv);

    // 启动定时器 (ARPE=1 缓冲使能, CEN=1 计数器使能)
    g_tim->CR1 = (1u << 7) | (1u << 0);
}