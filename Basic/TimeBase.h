#pragma once

#include "FreeRTOS.h"
#include "stm32h743xx.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef SYSFREQ480
// HSE=40MHz, PLL1: M=2 N=48 P=2 → SYSCLK=480MHz
// D1HPRE=20 → HCLK=24MHz; D2PPRE1=4 → APB1=120MHz; D3PPRE=1 → D3PCLK=120MHz
#define SYSCLK 480000000
#define HCLK 24000000
#define APB1CLK 120000000
#define APB1TIMERCLK 240000000
#define APB2CLK 120000000
#define APB2TIMERCLK 240000000
#define APB3CLK 120000000
#define APB4CLK 120000000
#define D3PCLK 120000000 // D3域外设时钟 (= APB4CLK)，I2C4/SPI6/LPTIM等使用
#else
// HSE=40MHz, PLL1: M=2 N=40 P=2 → SYSCLK=400MHz
// D1HPRE=20 → HCLK=20MHz; D2PPRE1=4 → APB1=100MHz; D3PPRE=1 → D3PCLK=100MHz
#define SYSCLK 400000000
#define HCLK 20000000
#define APB1CLK 100000000
#define APB1TIMERCLK 200000000
#define APB2CLK 100000000
#define APB2TIMERCLK 200000000
#define APB3CLK 100000000
#define APB4CLK 100000000
#define D3PCLK 100000000 // D3域外设时钟 (= APB4CLK)，I2C4/SPI6/LPTIM等使用
#endif
#define USART234578CLK 100000000
#define USART16CLK 100000000

#define TIMEBASECLK 10000000

    /*
            操作系统延时函数（通过挂起线程）
            只能在操作系统任务中使用
            t:秒单位延时时间
    */
    inline void os_delay(double t)
    {
        TickType_t ticks = t * configTICK_RATE_HZ;
        if (ticks < 1)
            ticks = 1;
        vTaskDelay(ticks);
    }

    bool get_RTC_Updated();

    // 锁定RTC操作
    bool Lock_RTC();
    void UnLock_RTC();

    /*RTC时间获取*/
    typedef struct
    {
        uint16_t Year;
        uint8_t Month;
        uint8_t Date;
        uint8_t WeekDay;

        uint8_t Hours;
        uint8_t Minutes;
        uint8_t Seconds;
        uint32_t SubSeconds;
        uint8_t TimeFormat;

        double Seconds_f;
    } RTC_TimeStruct;

    // 获取RTC时间
    RTC_TimeStruct Get_RTC_Time();
    time_t Get_RTC_Time_t(void);
    struct tm Get_RTC_Time_tm(void);

    // 设置RTC日期和时间 (返回true表示已写入, false表示未获取到RTC锁未写入)
    // lock_timeout: RTC 互斥锁等待超时, 单位秒; 0 表示不等待, 负数表示无限等待
    bool Set_RTC_Time(const RTC_TimeStruct *T, double lock_timeout);

    // 根据经纬度计算时区
    int GetTimeZone(double lat, double lon);

    // UTC转本地时间
    void UTC2LocalTime(RTC_TimeStruct *rtc_local, uint16_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second, int8_t TimeZone, int8_t leapS);

    // 本地时间转UNIX时间
    uint32_t LocalTime2Unix(const RTC_TimeStruct *rtc_local, int8_t TimeZone);

    /*RTC时间获取*/
#ifdef __cplusplus
}
#endif