#include "drv_Alt1_Ultrasonic_TE.hpp"

#include "Basic.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "TimeBase.h"
#include "TimeBase.hpp"
#include "task.h"

/*
    超声波定高驱动 (Trig/Echo 电平式, 如 US-100 / HC-SR04 电平模式)
    运行时从 HardWareConfig ([ultrasonic1] 段) 读取定时器、引脚与传感器序号配置,
    INI 中无该段则驱动不启动 (无机型不启用)。

    回波捕获原理: 回波引脚(定时器通道 TIx)同时作为从模式触发源与 IC1 捕获输入,
    SMCR 使用组合复位+触发模式 —— 回波上升沿复位并启动计数, 下降沿由 IC1 捕获,
    因此 CCR1 即为回波高电平持续时间。
    计数频率 500kHz(2us/tick), 不使用定时器中断, 由采样任务轮询 CC1IF。
*/

#define TE_TICK_CM 0.034       // 2us/tick 折算距离 (2e-6 * 17000)
#define TE_TRIG_HOLD 0.001     // 触发脉冲保持时间
#define TE_ECHO_TIMEOUT 0.06   // 回波等待超时
#define TE_ECHO_MIN_CM 1.0     // 有效距离下限
#define TE_ECHO_MAX_CM 1000.0  // 有效距离上限
#define TE_SAMPLE_INTERVAL 0.1 // 采样周期

struct DriverInfo
{
    uint8_t sensor_ind;
    uint32_t sensor_key;
    GPIO_TypeDef *trig_gpio;
    uint16_t trig_pin;
    TIM_TypeDef *tim;
};

static void UltrasonicTE_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    TIM_TypeDef *tim = driver_info.tim;
    while (1)
    {
        // 单脉冲模式下每次触发前重新使能计数
        tim->CR1 |= (1u << 0);
        tim->SR = ~(1u << 1); // 清 CC1IF
        // 触发脉冲
        driver_info.trig_gpio->BSRR = (1u << driver_info.trig_pin);
        delay(TE_TRIG_HOLD);
        driver_info.trig_gpio->BSRR = (1u << (driver_info.trig_pin + 16));

        // 等待回波下降沿捕获 (读 CCR1 同时清 CC1IF)
        TIME start_time = TIME::now();
        while ((tim->SR & (1u << 1)) == 0 && start_time.get_pass_time() < TE_ECHO_TIMEOUT)
            os_delay(0.001);
        if (tim->SR & (1u << 1))
        {
            double distance = tim->CCR1 * TE_TICK_CM;
            if (distance > TE_ECHO_MIN_CM && distance < TE_ECHO_MAX_CM)
            {
                vector3<double> position;
                position.z = distance;
                // 获取倾角
                Quaternion quat;
                get_Airframe_quat(&quat);
                double lean_cosin = quat.get_lean_angle_cosin();
                // 更新
                position.z *= lean_cosin;
                PositionSensorUpdatePosition(driver_info.sensor_ind, driver_info.sensor_key, position, true);
            }
            else
                PositionSensorSetInavailable(driver_info.sensor_ind, driver_info.sensor_key);
        }
        else
            PositionSensorSetInavailable(driver_info.sensor_ind, driver_info.sensor_key);

        os_delay(TE_SAMPLE_INTERVAL);
    }
}

static bool UltrasonicTE_SetUpTimer(uint16_t timer_num, uint16_t echo_ch)
{
    // 仅支持回波接在定时器通道1/2 (IC1 映射到 TI1/TI2)
    uint16_t ccmr1;
    uint16_t ts;
    if (echo_ch == 1)
    {
        ccmr1 = (0b01 << 0) | (1 << 4) | (0b01 << 8) | (1 << 12); // IC1<-TI1, IC2<-TI1
        ts = 0b101;                                               // TI1FP1
    }
    else if (echo_ch == 2)
    {
        ccmr1 = (0b10 << 0) | (1 << 4) | (0b01 << 8) | (1 << 12); // IC1<-TI2, IC2<-TI2
        ts = 0b110;                                               // TI2FP2
    }
    else
        return false;

    HW_EnableTimerClock(timer_num);
    TIM_TypeDef *tim = HW_GetTimerPtr(timer_num);
    if (tim == nullptr)
        return false;

    // 500kHz 计数
    bool is_apb2 = (timer_num == 1 || timer_num == 8 || timer_num == 15 || timer_num == 16 || timer_num == 17);
    uint32_t timer_clk = is_apb2 ? APB2TIMERCLK : APB1TIMERCLK;
    tim->PSC = timer_clk / 500000u - 1u;
    tim->ARR = 50000; // 0.1s
    tim->CCMR1 = ccmr1;
    tim->SMCR = (ts << 4) | (1u << 16); // 组合复位+触发模式, 触发源为回波通道
    tim->CCER = (1u << 1) | (1u << 0);  // CH1 下降沿捕获 (CH2 仅作触发输入)
    tim->DIER = 0;                      // 不使用中断
    tim->EGR = (1u << 0);
    tim->CR1 = (1u << 3) | (1u << 0); // 单脉冲模式 + 使能
    return true;
}

void init_drv_Alt1_Ultrasonic_TE()
{
    HW_ConfigEntry entry;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_TIMER, &entry) == false)
        return; // 该机型无超声波配置

    uint16_t timer_num = entry.value_high;
    uint16_t echo_ch = entry.value_low;

    HW_ConfigEntry trig_entry;
    HW_ConfigEntry echo_entry;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_PIN1, &trig_entry) == false)
        return;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_PIN2, &echo_entry) == false)
        return;
    uint16_t trig_port = trig_entry.value_high;
    uint16_t trig_pin = trig_entry.value_low;
    uint16_t echo_port = echo_entry.value_high;
    uint16_t echo_pin = echo_entry.value_low;
    uint8_t echo_af = 2;
    uint8_t echo_pull = HW_PULL_UP;
    uint8_t echo_otype = HW_OTYPE_OPEN_DRAIN;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_AF2, &entry))
        echo_af = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_PULL2, &entry))
        echo_pull = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_OTYPE2, &entry))
        echo_otype = (uint8_t) entry.value_low;

    uint8_t sensor_ind = default_altimeter1_sensor_index + 1;
    if (HW_ReadConfigEntry(HW_DEV_ULTRASONIC1, HW_CFG_PARAM1, &entry) && entry.value_low > 0 && entry.value_low <= default_altimeter_sensors)
        sensor_ind = entry.value_low - 1;

    if (UltrasonicTE_SetUpTimer(timer_num, echo_ch) == false)
        return;

    // 触发引脚: 推挽输出, 默认拉低
    RCC->AHB4ENR |= (1u << trig_port);
    RCC->AHB4ENR |= (1u << echo_port);
    (void) RCC->AHB4ENR;
    GPIO_TypeDef *trig_gpio = HW_GpioTable[trig_port];
    set_register(trig_gpio->MODER, 0b01, trig_pin * 2, 2);
    set_register(trig_gpio->OTYPER, HW_OTYPE_PUSH_PULL, trig_pin, 1);
    set_register(trig_gpio->PUPDR, HW_PULL_NONE, trig_pin * 2, 2);
    set_register(trig_gpio->OSPEEDR, HW_SPEED_LOW, trig_pin * 2, 2);
    trig_gpio->BSRR = (1u << (trig_pin + 16));

    // 回波引脚: 复用输入
    GPIO_TypeDef *echo_gpio = HW_GpioTable[echo_port];
    set_register(echo_gpio->MODER, 0b10, echo_pin * 2, 2);
    set_register(echo_gpio->OTYPER, echo_otype, echo_pin, 1);
    set_register(echo_gpio->PUPDR, echo_pull, echo_pin * 2, 2);
    set_register(echo_gpio->OSPEEDR, HW_SPEED_LOW, echo_pin * 2, 2);
    if (echo_pin < 8)
        set_register(echo_gpio->AFR[0], echo_af, echo_pin * 4, 4);
    else
        set_register(echo_gpio->AFR[1], echo_af, (echo_pin - 8) * 4, 4);

    // 注册传感器
    uint32_t sensor_key = PositionSensorRegister(sensor_ind, "Ultrasonic", Position_Sensor_Type_RangePositioning, Position_Sensor_DataType_s_z, Position_Sensor_frame_ENU, 0.05, // 延时
                                                 0,                                                                                                                              // xy信任度
                                                 0                                                                                                                               // z信任度
    );
    if (sensor_key == 0)
        return;

    DriverInfo *driver_info = new DriverInfo;
    driver_info->sensor_ind = sensor_ind;
    driver_info->sensor_key = sensor_key;
    driver_info->trig_gpio = trig_gpio;
    driver_info->trig_pin = trig_pin;
    driver_info->tim = HW_GetTimerPtr(timer_num);
    xTaskCreate(UltrasonicTE_Server, "USonicTE", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
}
