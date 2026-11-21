#include "drv_rc_sbus.hpp"
#include "Basic.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "ReceiverBackend.hpp"
#include "StorageSystem.hpp"
#include "drv_Uart.hpp"
#include "timers.h"

// 运行时确定的USART外设指针和中断号
static USART_TypeDef *sbus_usart = 0;
static IRQn_Type sbus_irqn = (IRQn_Type) 0;

static float rc_buf[16];
static bool failsafe;
static bool frameLost;
static uint8_t fByte = 0;
static uint8_t eByte = 0;

static void sbus_IRQHandlerTCB(void *pvParameter1, uint32_t ulParameter2)
{
    if (!frameLost)
        ReceiverUpdate("Sbus", !failsafe, rc_buf, 16, 0.01);
    NVIC_EnableIRQ(sbus_irqn);
}

static void sbus_UsartCallback()
{
    static uint8_t last_rdata = 0xff;

    bool err = false;
    if (sbus_usart->ISR & (1 << 3))
    {
        err = true;
        sbus_usart->ICR = 1 << 3;
    }
    if (sbus_usart->ISR & (1 << 0))
    {
        err = true;
        sbus_usart->ICR = 1 << 0;
    }
    sbus_usart->ICR = 1 << 11;

    static uint8_t SBUS_RC_State = 0;
    static unsigned char current_rc_channel = 0;
    static unsigned char rc_bit_count = 0;
    static unsigned short current_rc = 0;

#define SBUS_RC_Reset (SBUS_RC_State = current_rc_channel = rc_bit_count = current_rc = 0)
    while ((sbus_usart->ISR & (1 << 5)) != 0)
    {
        uint8_t rdata = sbus_usart->RDR & 0XFF;

        if (err)
            SBUS_RC_Reset;

        if (SBUS_RC_State == 0)
        {
            if (rdata == 0x0f && (last_rdata == 0 || (last_rdata & 0x0f) == 0x04))
            {
                ++SBUS_RC_State;
            }
        }
        else if (SBUS_RC_State <= 22)
        {
            rc_bit_count += 8;
            if (rc_bit_count >= 11)
            {
                rc_bit_count -= 11;
                unsigned char l_byte_count = 8 - rc_bit_count;
                rc_buf[current_rc_channel++] = 0.04885197850512945774303859306302f * (float) (current_rc | (((unsigned char) (rdata << rc_bit_count)) << 3));
                current_rc = (unsigned char) (rdata >> l_byte_count);
            }
            else
            {
                current_rc |= rdata << (rc_bit_count - 8);
            }
            ++SBUS_RC_State;
        }
        else if (SBUS_RC_State == 23)
        {
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2
            fByte = rdata;
            failsafe = rdata & (1 << SBUS_FAILSAFE_BIT);
            frameLost = rdata & (1 << SBUS_FRAMELOST_BIT);
            ++SBUS_RC_State;
        }
        else
        {
            if (rdata == 0 || (rdata & 0x0f) == 0x04)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                BaseType_t res = xTimerPendFunctionCallFromISR(sbus_IRQHandlerTCB, 0, 0, &xHigherPriorityTaskWoken);
                if (res == pdPASS)
                {
                    NVIC_DisableIRQ(sbus_irqn);
                }
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            SBUS_RC_Reset;
        }

        if (!err)
            last_rdata = rdata;
        else
            last_rdata = 0xff;
    }
}

void init_drv_rc_sbus()
{
    // 从配置读取SBUS使用的UART编号
    HW_ConfigEntry entry;
    if (!HW_ReadConfigEntry(HW_DEV_SBUS, HW_CFG_UART_REF, &entry))
        return;
    uint8_t uart_num = (uint8_t) entry.value_low;
    if (uart_num < 1 || uart_num > 8)
        return;

    sbus_usart = HW_UsartTable[uart_num];
    sbus_irqn = HW_UsartIrqnTable[uart_num];

    // 读取RX引脚配置
    uint16_t rx_port = 0, rx_pin = 0, rx_af = 0;
    uint16_t rx_pull = HW_PULL_UP, rx_otype = HW_OTYPE_OPEN_DRAIN;
    if (HW_ReadConfigEntry(HW_DEV_SBUS, HW_CFG_PIN2, &entry))
    {
        rx_port = entry.value_high;
        rx_pin = entry.value_low;
    }
    else
        return;
    if (HW_ReadConfigEntry(HW_DEV_SBUS, HW_CFG_AF2, &entry))
        rx_af = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_SBUS, HW_CFG_PULL2, &entry))
        rx_pull = entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_SBUS, HW_CFG_OTYPE2, &entry))
        rx_otype = entry.value_low;

    /*GPIO初始化*/
    GPIO_TypeDef *gpio = HW_GpioTable[rx_port];
    RCC->AHB4ENR |= (1 << rx_port);
    os_delay(1e-2);

    set_register(gpio->OTYPER, rx_otype, rx_pin, 1);
    set_register(gpio->PUPDR, rx_pull, rx_pin * 2, 2);
    uint8_t afIdx = rx_pin >= 8 ? 1 : 0;
    uint8_t afOff = (rx_pin >= 8 ? rx_pin - 8 : rx_pin) * 4;
    set_register(gpio->AFR[afIdx], rx_af, afOff, 4);
    set_register(gpio->MODER, 0b10, rx_pin * 2, 2);
    /*GPIO初始化*/

    /*USART初始化 (SBUS协议: 100kbps, 反向电平, 2停止位, 9bit数据, 接收超时)*/
    // 打开USART时钟
    *HW_UsartRccTable[uart_num].reg |= HW_UsartRccTable[uart_num].bit;
    os_delay(1e-2);

    sbus_usart->CR1 = (1 << 29) | (1 << 26) | (1 << 12) | (1 << 10) | (1 << 8) | (1 << 2);
    sbus_usart->CR2 = (1 << 23) | (1 << 16) | (0b10 << 12);
    sbus_usart->CR3 = (1 << 28) | (0b011 << 25);
    sbus_usart->RTOR = 10;
    sbus_usart->BRR = HW_UsartBaudClkTable[uart_num] / 100000;
    sbus_usart->CR1 |= (1 << 0);
    /*USART初始化*/

    // 注册USART中断回调
    USART_CallBackFunc_Register(uart_num, sbus_UsartCallback);
    NVIC_SetPriority(sbus_irqn, 5);
    NVIC_EnableIRQ(sbus_irqn);

    // 接收机注册
    ReceiverRegister("Sbus", RC_CLASS::MainPhysicalRc, 10);
}