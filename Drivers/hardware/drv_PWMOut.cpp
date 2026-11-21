#include "drv_PwmOut.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "TimeBase.hpp"
#include "event_groups.h"
#include "stm32h7xx.h"

// 全部通道输出
float PWM_CHANS[MAX_PWM_COUNT] = {0};

// 输入触发回调 (由捕获 ISR 调用)
static void (*s_inputTriggerCb)(void) = nullptr;

// PPM 捕获回调（TIM_UP 溢出和 TIM_CC 捕获各一个，均在 ISR 上下文调用）
static void (*s_ppmUpCb)(void) = nullptr;
static void (*s_ppmCcCb)(void) = nullptr;
static TIM_TypeDef *s_ppmTim = nullptr;

// 输入触发捕获运行时配置（由 init_drv_PWMOut 从 INI 填充，供 ISR 使用）
static TIM_TypeDef *s_captureTim = nullptr;
static volatile uint32_t *s_captureCcr = nullptr; // 对应通道的 CCR 寄存器
static uint32_t s_captureSrMask = 0;              // CC1IF=bit1 ... CC4IF=bit4

// 运行时 PWM 通道表 —— 由 init_drv_PWMOut 从 INI 填充
struct PwmChInfo
{
    TIM_TypeDef *tim;
    volatile uint32_t *ccr; // 指向 TIMx->CCR1~4
    GPIO_TypeDef *gpio;
    uint8_t gpio_pin;
    uint16_t timer_num; // 定时器编号
    uint8_t ch_num;     // 定时器通道 1-4
    bool valid;
};
static PwmChInfo s_pwmCh[MAX_PWM_COUNT] = {};
static uint8_t s_pwmChCount = 0; // INI 中实际配置的通道数

// ---- DShot 协议支持 ----
typedef struct
{
    uint16_t crc : 4;
    uint16_t telemetry : 1;
    uint16_t throttle : 11;
} __PACKED _DshotFrame;

static void Dshot_CRC_Check(_DshotFrame *frame)
{
    uint16_t data = *(uint16_t *) frame;
    data = data >> 4;
    frame->crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0f;
}

// 每通道输出类型 (HW_PwmType 枚举值)
static uint16_t s_chPwmType[MAX_PWM_COUNT] = {};
// DShot 油门值 (0=锁定, 48~2047=油门)
static uint16_t s_dshotThrottle[MAX_PWM_COUNT] = {};

// DShot 定时器信息 (ISR 位元拍打)
#define MAX_DSHOT_CH_PER_TIMER 4
#define MAX_DSHOT_TIMERS 8
struct DshotTimerInfo
{
    TIM_TypeDef *tim;
    uint8_t num_ch;
    uint8_t timer_num; // 定时器编号，用于 s_dshotNumMap 快速索引
    struct
    {
        uint8_t ch_num;  // 定时器通道 1-4
        uint8_t pwm_idx; // s_dshotThrottle 数组索引
    } ch[MAX_DSHOT_CH_PER_TIMER];
    uint16_t t1h;       // bit=1 的 CCR 值
    uint16_t t0h;       // bit=0 的 CCR 值
    uint16_t arr_orig;  // 位时序原始 ARR（用于帧间隔结束后恢复）
    uint16_t frame_gap; // 帧间隔（位周期数）
    // ISR 运行时状态
    volatile uint8_t send_step;
    uint16_t send_data[MAX_DSHOT_CH_PER_TIMER][16];
};
static DshotTimerInfo s_dshotTimers[MAX_DSHOT_TIMERS];
static uint8_t s_dshotTimerCount = 0;
// DShot 快速索引：[timer_num] → DshotTimerInfo*，避免 ISR 中线性搜索
static DshotTimerInfo *s_dshotNumMap[18] = {};

// 预计算 DShot 帧位数据（CRC + 位展开），由任务上下文调用，结果由 ISR 直接使用
static void _dshotPrecompute(DshotTimerInfo *ds)
{
    _DshotFrame frame;
    for (uint8_t i = 0; i < ds->num_ch; i++)
    {
        *(uint16_t *) &frame = 0;
        frame.throttle = s_dshotThrottle[ds->ch[i].pwm_idx];
        frame.telemetry = 0;
        Dshot_CRC_Check(&frame);
        uint16_t data = *(uint16_t *) &frame;
        for (uint8_t b = 0; b < 16; b++)
            ds->send_data[i][b] = (data & (0x8000u >> b)) ? ds->t1h : ds->t0h;
    }
}

static DshotTimerInfo *_findDshotTimer(TIM_TypeDef *tim)
{
    for (uint8_t i = 0; i < s_dshotTimerCount; i++)
        if (s_dshotTimers[i].tim == tim)
            return &s_dshotTimers[i];
    return nullptr;
}

// 主电机通道频率
#define MAIN_MT_FREQ 400
// 舵机通道频率
#define AUX_FREQ 50
// 主电机个数
static uint8_t MainMotorCount = 8;
// 主舵机个数
static uint8_t MainStCount = 0;
// 获取Aux通道个数
uint8_t get_AuxChannelCount()
{
    return s_pwmChCount - MainMotorCount;
}
// 获取主电机个数
uint8_t get_MainMotorCount()
{
    return MainMotorCount;
}
// 获取PWM通道总数
uint8_t get_PWMChannelsCount()
{
    return s_pwmChCount;
}

// channel类型记录
static ChannelOutputType channelsType[MAX_PWM_COUNT] = {};

// 根据通道索引获取 GPIO 端口/引脚（从运行时通道表）
static inline bool _getChannelPort(uint8_t ind, GPIO_TypeDef *&gpio, uint8_t &gpio_pin)
{
    if (ind >= s_pwmChCount || !s_pwmCh[ind].valid)
        return false;
    gpio = s_pwmCh[ind].gpio;
    gpio_pin = s_pwmCh[ind].gpio_pin;
    return true;
}

// func: 0-pwm推挽输出
//			3-上拉输入检测
static inline bool _setChannelFunc(uint8_t ind, ChannelOutputType func)
{
    if (ind >= s_pwmChCount)
        return false;
    if (func == channelsType[ind])
        return true;

    GPIO_TypeDef *gpio;
    uint8_t gpio_pin;
    if (_getChannelPort(ind, gpio, gpio_pin) == false)
        return false;

    switch (func)
    {
        default:
            return false;

        case ChannelOutputType_PWMPPOut:
        { // PWM输出

            // 复用模式
            set_register(gpio->MODER, 0b10, gpio_pin * 2, 2);
            // 推挽输出
            set_register(gpio->OTYPER, 0, gpio_pin, 1);
            // 上拉
            set_register(gpio->PUPDR, 0b00, gpio_pin * 2, 2);

            break;
        }

        case ChannelOutputType_ODPUIn:
        { // 上拉输入检测

            // 复用模式
            set_register(gpio->MODER, 0b00, gpio_pin * 2, 2);
            // 开漏输出
            set_register(gpio->OTYPER, 1, gpio_pin, 1);
            // 上拉
            set_register(gpio->PUPDR, 0b01, gpio_pin * 2, 2);

            break;
        }
    }
    // 更改channel类型
    channelsType[ind] = func;
    return true;
}

void set_MainMotorCount(uint8_t count, uint8_t st_count, float StFreq)
{
    // 电机数必须是偶数
    if (count & 1)
        ++count;
    // 舵机通道个数不得大于主通道数
    if (st_count > count)
        st_count = count;

    MainMotorCount = count;
    MainStCount = st_count;
    uint8_t MainMtCount = count - st_count;

    for (uint8_t i = 0; i < count && i < s_pwmChCount; ++i)
        _setChannelFunc(i, ChannelOutputType_PWMPPOut);

    // 对每个已初始化的定时器，根据通道所处位置设置 ARR（同一定时器只设一次）
    static TIM_TypeDef *visited[MAX_PWM_COUNT];
    uint8_t visitCnt = 0;
    for (uint8_t i = 0; i < s_pwmChCount; ++i)
    {
        if (!s_pwmCh[i].valid)
            continue;
        TIM_TypeDef *tim = s_pwmCh[i].tim;
        // 去重
        bool found = false;
        for (uint8_t j = 0; j < visitCnt; ++j)
            if (visited[j] == tim)
            {
                found = true;
                break;
            }
        if (found)
            continue;
        visited[visitCnt++] = tim;

        // DShot 定时器 ARR 固定用于位时序，不修改
        if (_findDshotTimer(tim))
            continue;

        // 决定该定时器的 ARR
        // 该定时器承载的通道中，最小的通道索引 i 决定类型
        float freq;
        if (i < MainMtCount)
            freq = MAIN_MT_FREQ;
        else if (i < count)
            freq = StFreq;
        else
            freq = AUX_FREQ;

        tim->ARR = (uint32_t) (1e6f / freq);
    }
}

void MainMotor_PWM_Out(double out[])
{
    for (uint8_t i = 0; i < MainMotorCount; ++i)
    {
        PWM_CHANS[i] = out[i];

        if (HTL_isEnabled())
            out[i] = 0;
    }

    for (uint8_t i = 0; i < MainMotorCount && i < s_pwmChCount; ++i)
    {
        if (!s_pwmCh[i].valid)
            continue;
        if (s_chPwmType[i] != HW_PWM_TYPE_PWM)
        {
            // DShot: 0~100 → 48~2047
            int32_t temp = (int32_t) (out[i] * 19.99) + 48;
            if (temp < 48)
                temp = 48;
            if (temp > 2047)
                temp = 2047;
            s_dshotThrottle[i] = (uint16_t) temp;
        }
        else if (s_pwmCh[i].ccr)
        {
            *s_pwmCh[i].ccr = (uint32_t) (out[i] * 10 + 1000);
        }
    }
    // 预计算所有 DShot 帧数据（CRC + 位展开），将运算移出 ISR 上下文
    for (uint8_t j = 0; j < s_dshotTimerCount; j++)
        _dshotPrecompute(&s_dshotTimers[j]);
}

void Aux_PWM_Out(double out, uint8_t ind)
{
    if (ind < MainMotorCount)
        return;

    if (ind < MAX_PWM_COUNT)
        PWM_CHANS[ind] = out;

    if (ind >= s_pwmChCount || !s_pwmCh[ind].valid)
        return;

    _setChannelFunc(ind, ChannelOutputType_PWMPPOut);

    if (s_pwmCh[ind].ccr)
        *s_pwmCh[ind].ccr = (uint32_t) (out * 10 + 1000);
}

int32_t getPWMus(uint8_t ind)
{
    if (ind >= MAX_PWM_COUNT)
        return -1;
    return (int32_t) (PWM_CHANS[ind] * 10 + 1000);
}

bool Aux_ChannelRead(double ind, bool *res)
{
    uint8_t i = (uint8_t) ind;
    if (i >= s_pwmChCount)
        return false;
    if (i < MainMotorCount)
        return false;

    GPIO_TypeDef *gpio;
    uint8_t gpio_pin;
    if (_getChannelPort(i, gpio, gpio_pin) == false)
        return false;

    if (_setChannelFunc(i, ChannelOutputType_ODPUIn) == false)
        return false;

    if (res)
    {
        *res = (gpio->IDR & (1u << gpio_pin)) != 0;
        return true;
    }
    return false;
}

void PWM_DisableAll()
{
    for (uint8_t i = 0; i < MAX_PWM_COUNT; ++i)
        PWM_CHANS[i] = 6000;

    for (uint8_t i = 0; i < s_pwmChCount; ++i)
    {
        if (!s_pwmCh[i].valid)
            continue;
        if (s_chPwmType[i] != HW_PWM_TYPE_PWM)
            s_dshotThrottle[i] = 0;
        else if (s_pwmCh[i].ccr)
            *s_pwmCh[i].ccr = 0;
    }
    for (uint8_t j = 0; j < s_dshotTimerCount; j++)
        _dshotPrecompute(&s_dshotTimers[j]);
}

void PWM_PullDownAll()
{
    for (uint8_t i = 0; i < MAX_PWM_COUNT; ++i)
        PWM_CHANS[i] = 0;

    for (uint8_t i = 0; i < s_pwmChCount; ++i)
    {
        if (!s_pwmCh[i].valid)
            continue;
        if (s_chPwmType[i] != HW_PWM_TYPE_PWM)
            s_dshotThrottle[i] = 0;
        else if (s_pwmCh[i].ccr)
            *s_pwmCh[i].ccr = 1000;
    }
    for (uint8_t j = 0; j < s_dshotTimerCount; j++)
        _dshotPrecompute(&s_dshotTimers[j]);
}

void MainMotor_PullUpAll()
{
    for (uint8_t i = 0; i < MainMotorCount; ++i)
        PWM_CHANS[i] = 100;

    if (HTL_isEnabled() == false)
    {
        uint8_t MainMTCount = MainMotorCount - MainStCount;
        for (uint8_t i = 0; i < MainMTCount && i < s_pwmChCount; ++i)
        {
            if (!s_pwmCh[i].valid)
                continue;
            if (s_chPwmType[i] != HW_PWM_TYPE_PWM)
                continue; // DShot 不需要 ESC 校准
            if (s_pwmCh[i].ccr)
                *s_pwmCh[i].ccr = 2000;
        }
    }
}

void MainMotor_PullDownAll()
{
    for (uint8_t i = 0; i < MainMotorCount; ++i)
        PWM_CHANS[i] = 0;

    uint8_t MainMTCount = MainMotorCount - MainStCount;
    for (uint8_t i = 0; i < MainMTCount && i < s_pwmChCount; ++i)
    {
        if (!s_pwmCh[i].valid)
            continue;
        if (s_chPwmType[i] != HW_PWM_TYPE_PWM)
            s_dshotThrottle[i] = 0;
        else if (s_pwmCh[i].ccr)
            *s_pwmCh[i].ccr = 1000;
    }
    for (uint8_t j = 0; j < s_dshotTimerCount; j++)
        _dshotPrecompute(&s_dshotTimers[j]);
}

void PWM_PullUpAll()
{
    for (uint8_t i = 0; i < MAX_PWM_COUNT; ++i)
        PWM_CHANS[i] = 100;

    for (uint8_t i = 0; i < s_pwmChCount; ++i)
    {
        if (!s_pwmCh[i].valid)
            continue;
        if (s_chPwmType[i] != HW_PWM_TYPE_PWM)
            continue; // DShot 不需要校准
        if (s_pwmCh[i].ccr)
            *s_pwmCh[i].ccr = 2000;
    }
}

static void _enableTimerIRQs(uint16_t timer_num, uint32_t prio); // 前向声明

void register_InputTriggerCallback(void (*cb)(void))
{
    s_inputTriggerCb = cb;
}

void register_PPMCallbacks(void (*up_cb)(void), void (*cc_cb)(void))
{
    s_ppmUpCb = up_cb;
    s_ppmCcCb = cc_cb;
}

void init_drv_PWMOut()
{
    // 使能全部 GPIO 时钟（保守策略：统一使能 PA~PH）
    RCC->AHB4ENR |= 0xFFu;
    os_delay(0.01);

    PWM_DisableAll();

    // ==== 从 INI 初始化 PWM 通道 ====
    // 辅助：判断定时器所在总线
    auto isApb2Timer = [](uint16_t n) -> bool { return (n == 1 || n == 8 || n == 15 || n == 16 || n == 17); };
    // TIM1/8 是高级定时器，TIM15/16/17 虽属通用定时器但同样有 BDTR 寄存器，
    // OCx 输出使能前必须置 BDTR.MOE，此处统一标记为"需要 MOE"
    auto needsMOE = [](uint16_t n) -> bool { return (n == 1 || n == 8 || n == 15 || n == 16 || n == 17); };

    // 获取 CCR 指针
    auto getTimCCR = [](TIM_TypeDef * tim, uint16_t ch)->volatile uint32_t *
    {
        switch (ch)
        {
            case 1:
                return &tim->CCR1;
            case 2:
                return &tim->CCR2;
            case 3:
                return &tim->CCR3;
            case 4:
                return &tim->CCR4;
            default:
                return nullptr;
        }
    };

    // 已使能过的定时器（去重）
    TIM_TypeDef *timerInited[MAX_PWM_COUNT] = {};
    uint8_t timerInitedCnt = 0;

    s_pwmChCount = 0;

    for (uint8_t ch_idx = 1; ch_idx <= 16; ++ch_idx)
    {
        uint16_t dev = (uint16_t) (HW_DEV_PWM1 + ch_idx - 1);
        HW_ConfigEntry entry;

        // 必须有 timer 配置
        if (!HW_ReadConfigEntry(dev, HW_CFG_TIMER, &entry))
            break; // 遇到第一个未配置的通道就停止

        uint16_t timer_num = entry.value_high;
        uint16_t ch_num = entry.value_low;

        // 读取 GPIO 配置
        uint16_t port = 0;
        uint16_t pin = 0;
        uint16_t af = 0;
        uint16_t pull = HW_PULL_NONE;
        uint16_t otype = HW_OTYPE_PUSH_PULL;
        uint16_t speed = 3; // very_high
        if (!HW_ReadConfigEntry(dev, HW_CFG_PIN1, &entry))
            break;
        port = entry.value_high;
        pin = entry.value_low;
        if (HW_ReadConfigEntry(dev, HW_CFG_AF1, &entry))
            af = entry.value_low;
        if (HW_ReadConfigEntry(dev, HW_CFG_PULL1, &entry))
            pull = entry.value_low;
        if (HW_ReadConfigEntry(dev, HW_CFG_OTYPE1, &entry))
            otype = entry.value_low;
        if (HW_ReadConfigEntry(dev, HW_CFG_SPEED1, &entry))
            speed = entry.value_low;

        // 读取通道类型 (PWM / DShot)
        uint16_t pwm_type = HW_PWM_TYPE_PWM;
        if (HW_ReadConfigEntry(dev, HW_CFG_TYPE, &entry))
            pwm_type = entry.value_low;

        // 获取定时器指针
        TIM_TypeDef *tim = HW_GetTimerPtr(timer_num);
        if (!tim)
            continue;

        // 填充通道表
        uint8_t idx = (uint8_t) (ch_idx - 1);
        s_pwmCh[idx].tim = tim;
        s_pwmCh[idx].ccr = getTimCCR(tim, ch_num);
        s_pwmCh[idx].gpio = HW_GpioTable[port];
        s_pwmCh[idx].gpio_pin = (uint8_t) pin;
        s_pwmCh[idx].timer_num = timer_num;
        s_pwmCh[idx].ch_num = (uint8_t) ch_num;
        s_pwmCh[idx].valid = true;
        s_chPwmType[idx] = pwm_type;
        s_pwmChCount = (uint8_t) (idx + 1);

        // ---- 配置 GPIO ----
        GPIO_TypeDef *gpio = HW_GpioTable[port];
        set_register(gpio->MODER, 0b10u, pin * 2, 2);
        set_register(gpio->OTYPER, (uint32_t) otype, pin, 1);
        set_register(gpio->PUPDR, (uint32_t) pull, pin * 2, 2);
        set_register(gpio->OSPEEDR, (uint32_t) speed, pin * 2, 2);
        if (pin < 8)
            set_register(gpio->AFR[0], (uint32_t) af, pin * 4, 4);
        else
            set_register(gpio->AFR[1], (uint32_t) af, (pin - 8) * 4, 4);

        // ---- 使能定时器时钟（去重）----
        bool timFound = false;
        for (uint8_t j = 0; j < timerInitedCnt; ++j)
            if (timerInited[j] == tim)
            {
                timFound = true;
                break;
            }

        if (!timFound)
        {
            timerInited[timerInitedCnt++] = tim;
            HW_EnableTimerClock(timer_num);
            os_delay(0.001);

            uint32_t psc = (uint32_t) (isApb2Timer(timer_num) ? (APB2TIMERCLK / 1000000u) : (APB1TIMERCLK / 1000000u)) - 1u;
            tim->PSC = psc;
            tim->ARR = 1000000u / MAIN_MT_FREQ;
            // 重新初始化前清零所有 CCR preload
            tim->CCR1 = 0;
            tim->CCR2 = 0;
            tim->CCR3 = 0;
            tim->CCR4 = 0;
            if (needsMOE(timer_num))
                tim->BDTR = (1u << 15); // MOE
            tim->EGR = (1u << 0);
        }

        // ---- 配置定时器通道为 PWM1 输出（带预装载）----
        switch (ch_num)
        {
            case 1:
                tim->CCMR1 = (tim->CCMR1 & ~0x00FFu) | (0b110u << 4) | (1u << 3);
                tim->CCER |= (1u << 0);
                break;
            case 2:
                tim->CCMR1 = (tim->CCMR1 & ~0xFF00u) | (0b110u << 12) | (1u << 11);
                tim->CCER |= (1u << 4);
                break;
            case 3:
                tim->CCMR2 = (tim->CCMR2 & ~0x00FFu) | (0b110u << 4) | (1u << 3);
                tim->CCER |= (1u << 8);
                break;
            case 4:
                tim->CCMR2 = (tim->CCMR2 & ~0xFF00u) | (0b110u << 12) | (1u << 11);
                tim->CCER |= (1u << 12);
                break;
            default:
                break;
        }
    }

    // ==== DShot 定时器后处理：重新配置 DShot 通道所在定时器 ====
    for (uint8_t i = 0; i < s_pwmChCount; i++)
    {
        if (!s_pwmCh[i].valid || s_chPwmType[i] == HW_PWM_TYPE_PWM)
            continue;

        TIM_TypeDef *tim = s_pwmCh[i].tim;
        DshotTimerInfo *ds = _findDshotTimer(tim);
        if (!ds)
        {
            if (s_dshotTimerCount >= MAX_DSHOT_TIMERS)
                continue;
            ds = &s_dshotTimers[s_dshotTimerCount++];
            ds->tim = tim;
            ds->num_ch = 0;
            ds->send_step = 0;

            uint32_t dshot_freq;
            switch (s_chPwmType[i])
            {
                case HW_PWM_TYPE_DSHOT150:
                    dshot_freq = 150000;
                    ds->frame_gap = 280;
                    break;
                case HW_PWM_TYPE_DSHOT300:
                    dshot_freq = 300000;
                    ds->frame_gap = 140;
                    break;
                case HW_PWM_TYPE_DSHOT600:
                default:
                    dshot_freq = 600000;
                    ds->frame_gap = 70;
                    break;
            }

            uint16_t tnum = s_pwmCh[i].timer_num;
            ds->timer_num = (uint8_t) tnum;
            if (tnum < 18)
                s_dshotNumMap[tnum] = ds;
            uint32_t timer_clk = isApb2Timer(tnum) ? APB2TIMERCLK : APB1TIMERCLK;
            tim->CR1 = 0;
            tim->PSC = (uint32_t) (timer_clk / 1e8) - 1;
            uint32_t arr = (uint32_t) (1e8 / dshot_freq);
            tim->ARR = arr;
            ds->t1h = (uint16_t) (arr * 3 / 4);
            ds->t0h = (uint16_t) (arr * 3 / 8);
            ds->arr_orig = (uint16_t) arr;
            tim->EGR = (1u << 0);
            tim->SR = 0;
            tim->DIER |= (1u << 0); // 使能更新中断
            if (needsMOE(tnum))
                tim->BDTR = (1u << 15);
            _enableTimerIRQs(tnum, 1); // DShot 优先级 1（高于 FreeRTOS BASEPRI 屏蔽阈值 0x20，不受临界区阻塞）
        }

        if (ds->num_ch < MAX_DSHOT_CH_PER_TIMER)
        {
            ds->ch[ds->num_ch].ch_num = s_pwmCh[i].ch_num;
            ds->ch[ds->num_ch].pwm_idx = i;
            ds->num_ch++;
        }
    }

    // 预计算 DShot 初始帧数据（throttle=0），确保 ISR 启动时 send_data 已就绪
    for (uint8_t i = 0; i < s_dshotTimerCount; i++)
        _dshotPrecompute(&s_dshotTimers[i]);

    // 启动所有已初始化的 PWM 定时器
    for (uint8_t j = 0; j < timerInitedCnt; ++j)
        timerInited[j]->CR1 = (1u << 7) | (1u << 0); // ARPE + CEN

    // ==== PPM 输入捕获 —— 从 INI [ppm] 读取配置 ====
    {
        HW_ConfigEntry entry;
        if (HW_ReadConfigEntry(HW_DEV_PPM, HW_CFG_TIMER, &entry))
        {
            uint16_t ppm_timer_num = entry.value_high;
            uint16_t ppm_ch_num = entry.value_low;

            uint16_t port = HW_PORT_PA;
            uint16_t pin = 8;
            uint16_t af = 1;
            uint16_t pull = HW_PULL_UP;
            uint16_t otype = HW_OTYPE_OPEN_DRAIN;
            if (HW_ReadConfigEntry(HW_DEV_PPM, HW_CFG_PIN1, &entry))
            {
                port = entry.value_high;
                pin = entry.value_low;
            }
            if (HW_ReadConfigEntry(HW_DEV_PPM, HW_CFG_AF1, &entry))
            {
                af = entry.value_low;
            }
            if (HW_ReadConfigEntry(HW_DEV_PPM, HW_CFG_PULL1, &entry))
            {
                pull = entry.value_low;
            }
            if (HW_ReadConfigEntry(HW_DEV_PPM, HW_CFG_OTYPE1, &entry))
            {
                otype = entry.value_low;
            }

            // 配置 GPIO
            GPIO_TypeDef *gpio = HW_GpioTable[port];
            set_register(gpio->MODER, 0b10u, pin * 2, 2);
            set_register(gpio->OTYPER, (uint32_t) otype, pin, 1);
            set_register(gpio->PUPDR, (uint32_t) pull, pin * 2, 2);
            set_register(gpio->OSPEEDR, 0b01u, pin * 2, 2);
            if (pin < 8)
                set_register(gpio->AFR[0], (uint32_t) af, pin * 4, 4);
            else
                set_register(gpio->AFR[1], (uint32_t) af, (pin - 8) * 4, 4);

            TIM_TypeDef *ppmTim = HW_GetTimerPtr(ppm_timer_num);
            if (ppmTim)
            {
                s_ppmTim = ppmTim;

                // 检查该定时器是否已被 PWM 通道使用
                bool ppmTimInited = false;
                for (uint8_t j = 0; j < timerInitedCnt; ++j)
                    if (timerInited[j] == ppmTim)
                    {
                        ppmTimInited = true;
                        break;
                    }

                if (!ppmTimInited)
                {
                    // 独占定时器：使能时钟，配置 ARR=0xFFFF 供 PPM 时序使用
                    HW_EnableTimerClock(ppm_timer_num);
                    os_delay(0.001);
                    uint32_t psc = (uint32_t) (isApb2Timer(ppm_timer_num) ? (APB2TIMERCLK / 1000000u) : (APB1TIMERCLK / 1000000u)) - 1u;
                    ppmTim->PSC = psc;
                    ppmTim->ARR = 0xFFFFu;
                    if (needsMOE(ppm_timer_num))
                        ppmTim->BDTR = (1u << 15); // MOE
                    ppmTim->EGR = (1u << 0);
                    ppmTim->CR1 = (1u << 7) | (1u << 0); // ARPE + CEN
                }

                // 配置捕获通道（输入捕获，4 时钟滤波）
                switch (ppm_ch_num)
                {
                    case 1:
                        ppmTim->CCMR1 = (ppmTim->CCMR1 & ~0x00FFu) | (0b0010u << 4) | (0b01u << 0);
                        ppmTim->CCER |= (1u << 0);
                        ppmTim->DIER |= (1u << 1) | (1u << 0); // CC1IE + UIE
                        break;
                    case 2:
                        ppmTim->CCMR1 = (ppmTim->CCMR1 & ~0xFF00u) | (0b0010u << 12) | (0b01u << 8);
                        ppmTim->CCER |= (1u << 4);
                        ppmTim->DIER |= (1u << 2) | (1u << 0); // CC2IE + UIE
                        break;
                    case 3:
                        ppmTim->CCMR2 = (ppmTim->CCMR2 & ~0x00FFu) | (0b0010u << 4) | (0b01u << 0);
                        ppmTim->CCER |= (1u << 8);
                        ppmTim->DIER |= (1u << 3) | (1u << 0); // CC3IE + UIE
                        break;
                    case 4:
                        ppmTim->CCMR2 = (ppmTim->CCMR2 & ~0xFF00u) | (0b0010u << 12) | (0b01u << 8);
                        ppmTim->CCER |= (1u << 12);
                        ppmTim->DIER |= (1u << 4) | (1u << 0); // CC4IE + UIE
                        break;
                    default:
                        break;
                }

                // 使能 NVIC（统一 helper）
                _enableTimerIRQs(ppm_timer_num, 5);
            }
        }
    }

    // ==== 输入触发捕获 —— 从 INI [input_trigger] 读取配置 ====
    {
        HW_ConfigEntry entry;
        if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_TIMER, &entry))
        {
            uint16_t timer_num = entry.value_high;
            uint16_t ch_num = entry.value_low;

            uint16_t port = HW_PORT_PB;
            uint16_t pin = 7;
            uint16_t af = 2;
            uint16_t pull = HW_PULL_UP;
            uint16_t otype = HW_OTYPE_OPEN_DRAIN;
            if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_PIN1, &entry))
            {
                port = entry.value_high;
                pin = entry.value_low;
            }
            if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_AF1, &entry))
            {
                af = entry.value_low;
            }
            if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_PULL1, &entry))
            {
                pull = entry.value_low;
            }
            if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_OTYPE1, &entry))
            {
                otype = entry.value_low;
            }
            uint16_t it_prescale = 0;
            uint16_t it_filter = 15;
            if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_PARAM1, &entry))
                it_prescale = entry.value_low;
            if (HW_ReadConfigEntry(HW_DEV_INPUT_TRIGGER, HW_CFG_FILTER, &entry))
                it_filter = entry.value_low & 0xFu;

            // 配置 GPIO
            GPIO_TypeDef *gpio = HW_GpioTable[port];
            set_register(gpio->MODER, 0b10u, pin * 2, 2);
            set_register(gpio->OTYPER, (uint32_t) otype, pin, 1);
            set_register(gpio->PUPDR, (uint32_t) pull, pin * 2, 2);
            set_register(gpio->OSPEEDR, 0b01u, pin * 2, 2);
            if (pin < 8)
                set_register(gpio->AFR[0], (uint32_t) af, pin * 4, 4);
            else
                set_register(gpio->AFR[1], (uint32_t) af, (pin - 8) * 4, 4);

            // 使能定时器时钟（若还未使能）
            HW_EnableTimerClock(timer_num);
            TIM_TypeDef *tim = HW_GetTimerPtr(timer_num);
            if (tim)
            {
                // 若该定时器未被 PWM 通道初始化，则在此初始化（1MHz 基准 × prescale 附加分频）
                bool itTimInited = false;
                for (uint8_t j = 0; j < timerInitedCnt; ++j)
                    if (timerInited[j] == tim)
                    {
                        itTimInited = true;
                        break;
                    }
                if (!itTimInited)
                {
                    uint32_t psc_base = (uint32_t) (isApb2Timer(timer_num) ? APB2TIMERCLK : APB1TIMERCLK) / 1000000u;
                    tim->PSC = psc_base * (it_prescale + 1u) - 1u;
                    tim->ARR = 0xFFFFu;
                    tim->EGR = (1u << 0);
                    tim->CR1 = (1u << 7) | (1u << 0); // ARPE + CEN
                }

                switch (ch_num)
                {
                    case 1:
                        tim->CCMR1 = (tim->CCMR1 & ~0x00FFu) | ((uint32_t) it_filter << 4) | (0b01u << 0);
                        tim->CCER |= (1u << 1) | (1u << 0);
                        tim->DIER |= (1u << 1);
                        break;
                    case 2:
                        tim->CCMR1 = (tim->CCMR1 & ~0xFF00u) | ((uint32_t) it_filter << 12) | (0b01u << 8);
                        tim->CCER |= (1u << 5) | (1u << 4);
                        tim->DIER |= (1u << 2);
                        break;
                    case 3:
                        tim->CCMR2 = (tim->CCMR2 & ~0x00FFu) | ((uint32_t) it_filter << 4) | (0b01u << 0);
                        tim->CCER |= (1u << 9) | (1u << 8);
                        tim->DIER |= (1u << 3);
                        break;
                    case 4:
                        tim->CCMR2 = (tim->CCMR2 & ~0xFF00u) | ((uint32_t) it_filter << 12) | (0b01u << 8);
                        tim->CCER |= (1u << 13) | (1u << 12);
                        tim->DIER |= (1u << 4);
                        break;
                    default:
                        tim = nullptr;
                        break;
                }
                if (tim)
                {
                    // 记录捕获配置供 ISR 使用
                    s_captureTim = tim;
                    s_captureSrMask = (1u << ch_num); // CC1IF=bit1, CC2IF=bit2, ...
                    switch (ch_num)
                    {
                        case 1:
                            s_captureCcr = &tim->CCR1;
                            break;
                        case 2:
                            s_captureCcr = &tim->CCR2;
                            break;
                        case 3:
                            s_captureCcr = &tim->CCR3;
                            break;
                        case 4:
                            s_captureCcr = &tim->CCR4;
                            break;
                        default:
                            s_captureCcr = nullptr;
                            break;
                    }
                    _enableTimerIRQs(timer_num, 5);
                }
            }
        }
    }

    // 初始化所有 PWM 通道为推挽输出状态
    for (uint8_t i = 0; i < s_pwmChCount; ++i)
        _setChannelFunc(i, ChannelOutputType_PWMPPOut);
}

// ---- 定时器 NVIC 使能 helper（PPM 和 input_trigger 共用）----
// TIM1/TIM8 同时使能 UP 和 CC 向量，其余定时器使能单一向量。
static void _enableTimerIRQs(uint16_t timer_num, uint32_t prio)
{
    switch (timer_num)
    {
        case 1:
            NVIC_SetPriority(TIM1_UP_IRQn, prio);
            NVIC_EnableIRQ(TIM1_UP_IRQn);
            NVIC_SetPriority(TIM1_CC_IRQn, prio);
            NVIC_EnableIRQ(TIM1_CC_IRQn);
            break;
        case 2:
            NVIC_SetPriority(TIM2_IRQn, prio);
            NVIC_EnableIRQ(TIM2_IRQn);
            break;
        case 3:
            NVIC_SetPriority(TIM3_IRQn, prio);
            NVIC_EnableIRQ(TIM3_IRQn);
            break;
        case 4:
            NVIC_SetPriority(TIM4_IRQn, prio);
            NVIC_EnableIRQ(TIM4_IRQn);
            break;
        case 8:
            NVIC_SetPriority(TIM8_UP_TIM13_IRQn, prio);
            NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
            NVIC_SetPriority(TIM8_CC_IRQn, prio);
            NVIC_EnableIRQ(TIM8_CC_IRQn);
            break;
        case 12:
            NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, prio);
            NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
            break;
        case 13:
            NVIC_SetPriority(TIM8_UP_TIM13_IRQn, prio);
            NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
            break;
        case 14:
            NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, prio);
            NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
            break;
        case 15:
            NVIC_SetPriority(TIM15_IRQn, prio);
            NVIC_EnableIRQ(TIM15_IRQn);
            break;
        default:
            break;
    }
}

// ---- 通用 IRQ 处理 ----

// DShot ISR 位元拍打处理：在定时器更新中断中逐位发送 DShot 帧
//
// 状态机（每次定时器 UEV 触发一次）：
//   step 0     : 输出 bit 0（帧数据已由 MainMotor_PWM_Out 预计算，合并原 step 0+1）
//   step 2~16  : 逐位写入 CCR bit 1~15（1.667µs/bit @ DShot600）
//   step 17    : 输出清零；将 ARR 改为帧间隔长度（shadow，下次 UEV 生效）
//   step 18    : 原始 ARR 最后一次 UEV 过渡，帧间隔计时器已启动
//   step 19+   : 帧间隔结束；恢复 ARR → EGR 强制 UEV → tail-chain 至 step 0
//
// 优化效果：① 帧间隔期间仅 2 次 ISR；② CRC/位展开移至任务上下文；
// ③ ISR 中 DShot 定时器通过 s_dshotNumMap 直接索引，无线性搜索。
static inline void _handleDshotUpdate(DshotTimerInfo *ds)
{
    if (ds->send_step == 0)
    {
        // 第 0 步：帧数据已由 MainMotor_PWM_Out 预计算，直接输出 bit 0
        // 合并了原 step 0（计算）和 step 1（首位输出），每帧减少 1 次 ISR
        for (uint8_t i = 0; i < ds->num_ch; i++)
        {
            volatile uint32_t *ccr = &ds->tim->CCR1 + (ds->ch[i].ch_num - 1);
            *ccr = ds->send_data[i][0];
        }
        ds->send_step = 2; // 已输出 bit 0，跳过旧 step 1，从 bit 1 继续
    }
    else if (ds->send_step <= 16)
    {
        // 第 2~16 步：逐位写入 CCR（bit 1~15）
        for (uint8_t i = 0; i < ds->num_ch; i++)
        {
            volatile uint32_t *ccr = &ds->tim->CCR1 + (ds->ch[i].ch_num - 1);
            *ccr = ds->send_data[i][ds->send_step - 1];
        }
        ds->send_step++;
    }
    else if (ds->send_step == 17)
    {
        // 最后一位已发出：CCR 清零（复位脉冲），将帧间隔时长写入 ARR shadow
        // ARR 新值在下一个 UEV 时生效，当前 bit 周期仍以 arr_orig 运行
        for (uint8_t i = 0; i < ds->num_ch; i++)
        {
            volatile uint32_t *ccr = &ds->tim->CCR1 + (ds->ch[i].ch_num - 1);
            *ccr = 0;
        }
        ds->tim->ARR = (uint32_t) ds->arr_orig * ds->frame_gap;
        ds->send_step = 18;
    }
    else if (ds->send_step == 18)
    {
        // 帧间隔 ARR 已生效（本 UEV 时 shadow 加载），间隔计时开始，等待下次 UEV
        ds->send_step = 19;
    }
    else
    {
        // 帧间隔结束：恢复 arr_orig（写入 shadow），EGR 强制 UEV
        // EGR 将 arr_orig 从 shadow 加载到活动 ARR，复位计数器，同时尾链触发下次 ISR（step 0）
        ds->tim->ARR = ds->arr_orig;
        ds->send_step = 0; // 必须在 EGR 之前设置，确保 tail-chain ISR 进入 step 0
        ds->tim->EGR = (1u << 0);
    }
}

// UP 中断（溢出）：DShot 更新 / PPM UP 回调
// ds 由调用方通过 s_dshotNumMap[timer_num] 直接传入，无需线性搜索
static inline void _handleTimUpIRQ(TIM_TypeDef *tim, DshotTimerInfo *ds = nullptr)
{
    uint32_t SR = tim->SR;
    tim->SR = (uint32_t) (~SR);

    // DShot 位元拍打处理（更新中断）
    if (SR & (1u << 0))
    {
        if (ds)
        {
            _handleDshotUpdate(ds);
            return;
        }
    }

    if (tim == s_ppmTim && (SR & (1u << 0)))
    {
        if (s_ppmUpCb)
            s_ppmUpCb();
    }
}

// CC 中断（捕获比较）：根据定时器用途分别处理 PPM 和 input_trigger
static inline void _handleTimCcIRQ(TIM_TypeDef *tim)
{
    uint32_t SR = tim->SR;
    tim->SR = (uint32_t) (~SR);
    // PPM 捕获回调
    if (tim == s_ppmTim)
    {
        if (s_ppmCcCb)
            s_ppmCcCb();
    }
    // input_trigger 捕获回调
    if ((tim == s_captureTim) && (SR & s_captureSrMask))
    {
        if (s_captureCcr)
            (void) *s_captureCcr;
        if (s_inputTriggerCb)
            s_inputTriggerCb();
    }
}

// 对只有一条向量的定时器，UP 溢出和 CC 捕获共用同一 IRQ
static inline void _handleTimIRQ(TIM_TypeDef *tim, DshotTimerInfo *ds = nullptr)
{
    _handleTimUpIRQ(tim, ds);
    _handleTimCcIRQ(tim);
}

// TIM1: UP 和 CC 分开
extern "C" void TIM1_UP_IRQHandler()
{
    _handleTimUpIRQ(TIM1, s_dshotNumMap[1]);
}
extern "C" void TIM1_CC_IRQHandler()
{
    _handleTimCcIRQ(TIM1);
}

extern "C" void TIM2_IRQHandler()
{
    _handleTimIRQ(TIM2, s_dshotNumMap[2]);
}
extern "C" void TIM3_IRQHandler()
{
    _handleTimIRQ(TIM3, s_dshotNumMap[3]);
}
extern "C" void TIM4_IRQHandler()
{
    _handleTimIRQ(TIM4, s_dshotNumMap[4]);
}
// TIM5 被 TimeBase 占用，不在此定义
// TIM8: UP 共享 TIM8_UP_TIM13 向量，CC 独立
extern "C" void TIM8_CC_IRQHandler()
{
    _handleTimCcIRQ(TIM8);
}
extern "C" void TIM8_BRK_TIM12_IRQHandler()
{
    _handleTimIRQ(TIM12, s_dshotNumMap[12]);
}
extern "C" void TIM8_UP_TIM13_IRQHandler()
{
    _handleTimUpIRQ(TIM8, s_dshotNumMap[8]);
    _handleTimIRQ(TIM13, s_dshotNumMap[13]);
}
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
    _handleTimIRQ(TIM14, s_dshotNumMap[14]);
}
extern "C" void TIM15_IRQHandler()
{
    _handleTimIRQ(TIM15, s_dshotNumMap[15]);
}
