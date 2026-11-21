#include "drv_extIO_FPVESC_U.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "MavlinkCMDProcess.hpp"
#include "StorageSystem.hpp"
#include "mavlink.h"
#include "task.h"

static uint8_t componentID = MAV_COMP_ID_USER56;

struct DriverInfo
{
    uint32_t param;
    Port port;
};

typedef enum
{
    type_pchar,
    type_uint32_t,
    type_int32_t,
    type_float,
    type_enum,
} eEscParameterType;

#define _SMotorMsg_HEAD1 (0XAA)
#define _SMotorMsg_HEAD2 (0XCC)
typedef struct __attribute__((__packed__))
{
    uint8_t head1;
    uint8_t head2;

    uint8_t target_id;  // 电调ID
    uint8_t cmd;        // 0:基本信息 1:参数 2:状态 3:注册
    uint8_t read_write; // 0:读 1:写
    uint8_t msg_id;     // 参数id

    union
    {
        uint8_t payload[48];
        uint32_t payload_uint[12];
        int32_t payload_int[12];
        float payload_float[12];
    };

    uint8_t sum;
} _SMotorMsg;

typedef struct __attribute__((packed))
{
    uint16_t ID;
    char Name[20];
    uint8_t type;
    union
    {
        uint8_t payload[20];
        uint32_t payload_uint[5];
        int32_t payload_int[5];
        float payload_float[5];
    };
    uint16_t version;
} sEscParameterUnit;

typedef struct __attribute__((__packed__))
{
    float Temperature;   // 电调温度，仅4号电调连接了
    float rpm;           // 转速
    uint32_t Period;     // 换向周期
    uint32_t pwm_duty;   // 实际占空比 0~1000
    uint32_t pulsewidth; // 脉宽 单位（us）
} _SMotorProperty;

typedef struct __attribute__((__packed__))
{
    uint32_t SoftwareVersion; // 固件版本号
    uint32_t RegisterDate;    // 注册日期
    uint8_t RegisterState;    // 注册状态
    uint8_t EscModel_len;     // 电调型号长度
    uint8_t EscModel[19];     // 电调型号
    uint32_t ProducedDate;    // 生产日期
    uint32_t HardwareVersion; // 硬件版本号
    uint16_t ParameterCount;  // 参数数量
} _SMotorBasicMsg;

float esc_temperature = -1;
static bool esc_ready_flag = false;
static bool esc_send_flag = false;
static bool gcs_send_flag = false;
_SMotorMsg ack_msg;
_SMotorMsg send_msg;
_SMotorMsg gcs_msg;
_SMotorProperty esc_property[4];

_SMotorBasicMsg basic_msg;
sEscParameterUnit EscParameterUnit;

QueueHandle_t esc_receive_message_queue = nullptr;

static void Write_LPUart1(uint8_t *buff, uint16_t length)
{
    LPUART1->CR1 &= ~(1 << 2);
    for (uint16_t i = 0; i < length; i++)
    {
        while ((LPUART1->ISR & (1 << 7)) == 0)
            ;
        LPUART1->TDR = buff[i];

        // 要提前打开接收，否则接收会丢失前几个字节
        if (i > 10)
            LPUART1->CR1 |= (1 << 2);
    }
    LPUART1->CR1 |= (1 << 2);
}

static void ESC_SendMSG(_SMotorMsg *msg, uint8_t target_id, uint8_t cmd, uint8_t read_write, uint16_t msg_id)
{
    uint8_t *buff = (uint8_t *) msg;

    msg->head1 = _SMotorMsg_HEAD1;
    msg->head2 = _SMotorMsg_HEAD2;
    msg->target_id = target_id;
    msg->cmd = cmd;
    msg->read_write = read_write;
    msg->msg_id = msg_id;

    msg->sum = 0;

    for (uint16_t i = 0; i < sizeof(_SMotorMsg) - 1; i++)
    {
        msg->sum += buff[i];
    }

    Write_LPUart1(buff, sizeof(_SMotorMsg));
}

static void ESC_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    TIME Esc_TIME(false);
    TIME freq_time(true);
    TIME Esc_Status_TIME(false);
    static uint8_t over_time_count = 0;
    mavlink_message_t msg;

    while (1)
    {
        // 接收参数信息
        double TIMEOUT = 0.01;
        bool msgAvailable = false;
        TickType_t TIMEOUT_Ticks;
        if (TIMEOUT >= 0)
            TIMEOUT_Ticks = TIMEOUT * configTICK_RATE_HZ;
        else
            TIMEOUT_Ticks = portMAX_DELAY;

        if (esc_send_flag == false && gcs_send_flag == false)
        {
            if (xQueueReceive(esc_receive_message_queue, &msg, TIMEOUT_Ticks) == pdTRUE)
                msgAvailable = true;
            else
                msgAvailable = false;
        }

        if (msgAvailable)
        { // 处理地面站数据
            mavlink_data64_t *msg_rd = (mavlink_data64_t *) &msg.payload64;
            memcpy((uint8_t *) &gcs_msg.target_id, (const uint8_t *) &msg_rd->data, sizeof(_SMotorMsg) - 3);

            // 转发命令到电调
            ESC_SendMSG(&gcs_msg, gcs_msg.target_id, gcs_msg.cmd, gcs_msg.read_write, gcs_msg.msg_id);

            gcs_send_flag = true;
            over_time_count = 0;
            Esc_TIME = TIME::now();
        }
        else if (gcs_send_flag == true)
        { // 等待电调回复
            if (Esc_TIME.get_pass_time() > 0.04)
            {
                Esc_TIME = TIME::now();

                over_time_count++;
                if (over_time_count > 5)
                {
                    over_time_count = 0;
                    gcs_send_flag = false;
                }
            }
        }
        else
        { // 无地面站数据，获取ESC信息
            if (Esc_TIME.get_pass_time() > 0.04)
            {
                Esc_TIME = TIME::now();
                static uint8_t id = 1;

                if (esc_send_flag == false)
                { // 获取状态
                    esc_send_flag = true;
                    over_time_count = 0;

                    // 获取电调状态
                    ESC_SendMSG(&send_msg, id, 2, 0, 0);
                    id >= 4 ? id = 1 : id++;
                }
                else
                { // 超时未回复
                    over_time_count++;
                    if (over_time_count > 5)
                    {
                        over_time_count = 0;
                        esc_send_flag = false;
                    }
                }
            }
        }

        if (esc_ready_flag == true)
        { // 收到回复
            esc_ready_flag = false;

            if (ack_msg.cmd == 0)
            { // 获取基本信息
                gcs_send_flag = false;
                over_time_count = 0;

                memcpy((uint8_t *) &basic_msg, (const uint8_t *) &ack_msg.payload[0], sizeof(_SMotorBasicMsg));

                // 转发地面站
                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                {
                    const Port *port = get_CommuPort(i);
                    mavlink_message_t msg_sd;
                    if (port->write != 0 && port)
                    {
                        if (mavlink_lock_chan(i, 0.05))
                        {
                            mavlink_msg_data64_pack_chan(get_CommulinkSysId(), // system id
                                                         componentID,          // component id
                                                         i,                    // chan
                                                         &msg_sd, MAV_COMP_ID_USER56, sizeof(_SMotorMsg) - 3, &ack_msg.target_id);

                            mavlink_send_msg(port, &msg_sd);
                            mavlink_unlock_chan(i);
                        }
                    }
                }
            }
            else if (ack_msg.cmd == 1)
            { // 获取参数
                gcs_send_flag = false;
                over_time_count = 0;

                memcpy((uint8_t *) &EscParameterUnit, (const uint8_t *) &ack_msg.payload[0], sizeof(sEscParameterUnit));
                // 转发地面站
                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                {
                    const Port *port = get_CommuPort(i);
                    mavlink_message_t msg_sd;
                    if (port->write != 0 && port)
                    {
                        if (mavlink_lock_chan(i, 0.2))
                        {
                            mavlink_msg_data64_pack_chan(get_CommulinkSysId(), // system id
                                                         componentID,          // component id
                                                         i,                    // chan
                                                         &msg_sd, MAV_COMP_ID_USER56, sizeof(_SMotorMsg) - 3, &ack_msg.target_id);

                            mavlink_send_msg(port, &msg_sd);
                            mavlink_unlock_chan(i);
                        }
                    }
                }
            }
            else if (ack_msg.cmd == 2)
            { // ESC状态
                esc_send_flag = false;
                memcpy((uint8_t *) &esc_property[ack_msg.target_id - 1], (const uint8_t *) &ack_msg.payload[0], 20);
                // 获取电调温度，只有四号电调温度通道连接到热敏电阻
                esc_temperature = esc_property[3].Temperature;

                int32_t rpm[4];
                rpm[0] = (int32_t) esc_property[0].rpm;
                rpm[1] = (int32_t) esc_property[1].rpm;
                rpm[2] = (int32_t) esc_property[2].rpm;
                rpm[3] = (int32_t) esc_property[3].rpm;

                float voltage[4];
                voltage[0] = esc_temperature;

                float current[4];

                if (Esc_Status_TIME.get_pass_time() > 0.2)
                { // 发送电调温度和转速
                    Esc_Status_TIME = TIME::now();

                    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                    {
                        const Port *port = get_CommuPort(i);
                        mavlink_message_t msg_sd;
                        if (port->write != 0 && port)
                        {
                            if (mavlink_lock_chan(i, 0.05))
                            {
                                mavlink_msg_esc_status_pack_chan(get_CommulinkSysId(), // system id
                                                                 componentID,          // component id
                                                                 i,                    // chan
                                                                 &msg_sd, 4, 0, rpm, voltage, current);

                                mavlink_send_msg(port, &msg_sd);
                                mavlink_unlock_chan(i);
                            }
                        }
                    }
                }

                bool attCtrlEna;
                is_Attitude_Control_Enabled(&attCtrlEna);

                if (attCtrlEna)
                {
                    // 记录日志
                    double vect[4];

                    vect[0] = esc_property[0].Temperature;
                    vect[1] = esc_property[1].Temperature;
                    vect[2] = esc_property[2].Temperature;
                    vect[3] = esc_property[3].Temperature;
                    SDLog_Msg_DebugVect("ESC_Temp", vect, 4);

                    vect[0] = esc_property[0].rpm;
                    vect[1] = esc_property[1].rpm;
                    vect[2] = esc_property[2].rpm;
                    vect[3] = esc_property[3].rpm;
                    SDLog_Msg_DebugVect("ESC_Rpm", vect, 4);

                    vect[0] = esc_property[0].Period;
                    vect[1] = esc_property[1].Period;
                    vect[2] = esc_property[2].Period;
                    vect[3] = esc_property[3].Period;
                    SDLog_Msg_DebugVect("ESC_Period", vect, 4);

                    vect[0] = esc_property[0].pwm_duty + 1000;
                    vect[1] = esc_property[1].pwm_duty + 1000;
                    vect[2] = esc_property[2].pwm_duty + 1000;
                    vect[3] = esc_property[3].pwm_duty + 1000;
                    SDLog_Msg_DebugVect("ESC_Duty", vect, 4);

                    vect[0] = esc_property[0].pulsewidth;
                    vect[1] = esc_property[1].pulsewidth;
                    vect[2] = esc_property[2].pulsewidth;
                    vect[3] = esc_property[3].pulsewidth;
                    SDLog_Msg_DebugVect("ESC_Pulsewidth", vect, 4);

                    vect[0] = 1 / freq_time.get_pass_time();
                    freq_time = TIME::now();
                    SDLog_Msg_DebugVect("ESC_Freq", vect, 1);
                }
            }
            else if (ack_msg.cmd == 3)
            { // ESC注册
                gcs_send_flag = false;
                over_time_count = 0;

                // 转发地面站
                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                {
                    const Port *port = get_CommuPort(i);
                    mavlink_message_t msg_sd;
                    if (port->write != 0 && port)
                    {
                        if (mavlink_lock_chan(i, 0.2))
                        {
                            mavlink_msg_data64_pack_chan(get_CommulinkSysId(), // system id
                                                         componentID,          // component id
                                                         i,                    // chan
                                                         &msg_sd, MAV_COMP_ID_USER56, sizeof(_SMotorMsg) - 3, &ack_msg.target_id);

                            mavlink_send_msg(port, &msg_sd);
                            mavlink_unlock_chan(i);
                        }
                    }
                }
            }
        }

        os_delay(0.001);
    }
}

extern "C" void LPUART1_IRQHandler()
{
    static int rc_counter = 0;
    static uint8_t sum = 0;
    static uint8_t payload[100];

    if (LPUART1->ISR & (1 << 5))
    {
        uint8_t rdata = LPUART1->RDR;

        if (rc_counter == 0)
        {
            if (rdata == _SMotorMsg_HEAD1)
            {
                payload[rc_counter] = rdata;
                rc_counter++;
                sum = 0;
                sum += rdata;
            }
            else
            {
                rc_counter = 0;
                sum = 0;
            }
        }
        else if (rc_counter == 1)
        {
            if (rdata == _SMotorMsg_HEAD2)
            {
                payload[rc_counter] = rdata;
                rc_counter++;
                sum += rdata;
            }
            else
            {
                rc_counter = 0;
                sum = 0;
            }
        }
        else if (rc_counter < 54)
        {
            payload[rc_counter] = rdata;
            rc_counter++;
            sum += rdata;
        }
        else
        {
            payload[rc_counter] = rdata;
            if (sum == rdata)
            {
                memcpy((uint8_t *) &ack_msg, payload, sizeof(_SMotorMsg));
                esc_ready_flag = true;
            }
            sum = 0;
            rc_counter = 0;
        }
    }

    LPUART1->ICR = (1 << 3);
}

static bool ESC_DriverInit()
{
    /*从INI配置读取引脚参数*/
    HW_ConfigEntry entry;
    if (!HW_ReadConfigEntry(HW_DEV_ESC_FPV, HW_CFG_PIN2, &entry))
        return false; // 未配置 esc_fpv，跳过初始化
    uint16_t rx_port = entry.value_high;
    uint16_t rx_pin = entry.value_low;

    uint16_t rx_af = 8; // 默认 LPUART1 AF
    if (HW_ReadConfigEntry(HW_DEV_ESC_FPV, HW_CFG_AF2, &entry))
        rx_af = entry.value_low;

    uint16_t rx_pull = HW_PULL_UP;
    if (HW_ReadConfigEntry(HW_DEV_ESC_FPV, HW_CFG_PULL2, &entry))
        rx_pull = entry.value_low;

    uint16_t rx_otype = HW_OTYPE_OPEN_DRAIN;
    if (HW_ReadConfigEntry(HW_DEV_ESC_FPV, HW_CFG_OTYPE2, &entry))
        rx_otype = entry.value_low;

    if (rx_port > 7)
        return false;
    GPIO_TypeDef *rx_gpio = HW_GpioTable[rx_port];
    /*GPIO初始化*/
    RCC->AHB4ENR |= (1u << rx_port);
    os_delay(1e-2);

    if (rx_otype == HW_OTYPE_OPEN_DRAIN)
        rx_gpio->OTYPER |= (1u << rx_pin);
    else
        rx_gpio->OTYPER &= ~(1u << rx_pin);
    set_register(rx_gpio->PUPDR, rx_pull, rx_pin * 2, 2);
    if (rx_pin < 8)
        set_register(rx_gpio->AFR[0], (uint32_t) rx_af, rx_pin * 4, 4);
    else
        set_register(rx_gpio->AFR[1], (uint32_t) rx_af, (rx_pin - 8) * 4, 4);
    set_register(rx_gpio->MODER, 0b10, rx_pin * 2, 2);
    /*GPIO初始化*/

    /*LPUART1初始化*/
    // 打开LPUart1时钟
    RCC->APB4ENR |= (1 << 3);
    os_delay(1e-2);

    LPUART1->CR1 = (1 << 5) | (1 << 3) | (1 << 2);
    LPUART1->CR2 = (1 << 15); // RX/TX交换
    LPUART1->CR3 = (1 << 3);  // 单线半双工
    LPUART1->BRR = USART234578CLK / 115200 * 256;
    LPUART1->CR1 |= (1 << 0); // LPUART1使能

    NVIC_SetPriority(LPUART1_IRQn, 3);
    NVIC_EnableIRQ(LPUART1_IRQn);
    /*LPUART1初始化*/

    /* 注册消息队列 */
    esc_receive_message_queue = xQueueCreate(30, sizeof(mavlink_message_t));

    DriverInfo *driver_info = new DriverInfo;
    xTaskCreate(ESC_Server, "ESC", 1024, (void *) driver_info, SysPriority_UserTask, NULL);
    return true;
}

void init_drv_extIO_FPVESC_U()
{
    HW_ConfigEntry entry;
    if (!HW_ReadConfigEntry(HW_DEV_ESC_FPV, HW_CFG_PIN2, &entry))
        return; // 未配置 esc_fpv

    ESC_DriverInit();
}