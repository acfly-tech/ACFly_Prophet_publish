#include "drv_rc_sbus2.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "ReceiverBackend.hpp"
#include "drv_Uart.hpp"
#include "task.h"

// 移植自X9 drv_rc_sbus2 (Port任务级解码方案, 区别于K9原生Sbus1的ISR方案)
// 依赖K9串口底层能力: SetRxInvert(RX反相) / SetParity(偶校验+通配头0x0F转义)

struct DriverInfo
{
    uint32_t param;
    Port port;
};

static void SBus_Port_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    // 配置 100kbps 8E2
    driver_info.port.SetBaudRate(100000, 2, 2);
    // 开启Rx反相器
    if (driver_info.port.setRxInvert)
        driver_info.port.setRxInvert(true);
// 开启偶校验，通配头0xFF
#define WILDCARD_BYTE 0xFF
    if (driver_info.port.SetParity)
        driver_info.port.SetParity(UartParity_Even, WILDCARD_BYTE, 2, 2);

    // 接收到的数据
    float rc_buf[16] = {0};
    bool failsafe = false;

    // 状态机
    uint8_t SBUS_RC_State = 0;
    uint8_t current_rc_channel = 0;
    uint8_t rc_bit_count = 0;
    uint16_t current_rc = 0;
#define SBUS_RC_Reset (SBUS_RC_State = current_rc_channel = rc_bit_count = current_rc = 0)

    uint8_t last_rdata = 0x00;
    bool esc_pending = false;

    while (1)
    {
        uint8_t rdata;
        if (driver_info.port.read(&rdata, 1, 0.1, 0.5) == 0)
        {
            // 接收超时，复位状态机
            SBUS_RC_Reset;
            last_rdata = 0x00;
            esc_pending = false;
            continue;
        }

        // 通配头转义解码
        bool parity_error = false;
        uint8_t decoded;
        if (esc_pending)
        {
            esc_pending = false;
            if (rdata == UART_PARITY_ESC_ERROR)
            {
                // 奇偶校验错误
                parity_error = true;
                decoded = 0;
            }
            else if (rdata == UART_PARITY_ESC_LITERAL)
            {
                // 数据就是通配头本身
                decoded = WILDCARD_BYTE;
            }
            else
            {
                // 不应该出现，视为错误
                parity_error = true;
                decoded = 0;
            }
        }
        else if (rdata == WILDCARD_BYTE)
        {
            // 通配头，等待下一个字节
            esc_pending = true;
            continue;
        }
        else
        {
            decoded = rdata;
        }

        // 奇偶校验错误处理
        if (parity_error)
        {
            SBUS_RC_Reset;
            last_rdata = 0xFF;
            continue;
        }

        // SBUS协议解析（与drv_rc_sbus.cpp相同逻辑）
        if (SBUS_RC_State == 0)
        {
            if (decoded == 0x0F && last_rdata == 0x00)
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
                uint8_t l_byte_count = 8 - rc_bit_count;
                rc_buf[current_rc_channel++] = 0.04885197850512945774303859306302f * (float) (current_rc | (((uint8_t) (decoded << rc_bit_count)) << 3));
                current_rc = (uint8_t) (decoded >> l_byte_count);
            }
            else
            {
                current_rc |= decoded << (rc_bit_count - 8);
            }
            ++SBUS_RC_State;
        }
        else if (SBUS_RC_State == 23)
        {
#define SBUS_FAILSAFE_BIT 3
            failsafe = decoded & (1 << SBUS_FAILSAFE_BIT);
            ++SBUS_RC_State;
        }
        else
        {
            if (decoded == 0 || (decoded & 0x0F) == 0x04)
            {
                ReceiverUpdate("Sbus2", !failsafe, rc_buf, 16, 0.01);
            }
            SBUS_RC_Reset;
        }
        if (!parity_error)
            last_rdata = decoded;
        else
            last_rdata = 0xFF;
    }
}

static bool SBus_Port_DriverInit(Port port, uint32_t param)
{
    if (!port.read || !port.SetBaudRate || !port.SetParity || !port.setRxInvert)
        return false;

    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(SBus_Port_Server, "Sbus2", 1024, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_rc_sbus2()
{
    // 注册接收机
    ReceiverRegister("Sbus2", RC_CLASS::MainPhysicalRc, 10);

    // X9使用151, K9的151已被D1800引擎占用, 改用51(与CRSF的50相邻)
    PortFunc_Register(51, SBus_Port_DriverInit);
}
