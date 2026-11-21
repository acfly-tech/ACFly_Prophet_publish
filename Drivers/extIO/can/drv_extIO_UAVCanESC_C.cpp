#include "drv_extIO_UAVCanESC_C.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "Parameters.hpp"
#include "StorageSystem.hpp"
#include "drv_ExtIO.hpp"
#include "drv_PWMOut.hpp"
#include "drv_can.hpp"

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
};

#define FC_ID 10

typedef struct
{                              // 广播帧
    uint8_t Soure_node_id : 7; // 自身节点id 取值为1~127 其中126~127为保留id
    uint8_t notBroadcast : 1;  // 数据帧类型 0为message帧 1为service帧
    uint16_t Message_type_id;  // 消息id 0~65535
    uint8_t Priority : 5;      // 优先级 HIGHEST=0x0 HIGH=0x8 MEDIUM=0x1 LOW=0x18 LOWEST=0x1f
    uint8_t none : 3;          // 空 0
} __AC_PACKED _EscMessageFrame;

typedef struct
{                                     // 非广播帧
    uint8_t Soure_node_id : 7;        // 自身节点id 取值为1~127 其中0、126~127为保留id
    uint8_t notBroadcast : 1;         // 数据帧类型 0为message帧 1为service帧
    uint8_t Destination_node_id : 7;  // 对方节点id 取值为1~127 其中0、126~127为保留id
    uint8_t Request_not_response : 1; // 数据帧类型 0为应答 1为请求
    uint8_t Service_type_id;          // 消息id 0~255
    uint8_t Priority : 5;             // 优先级 HIGHEST=0x0 HIGH=0x8 MEDIUM=0x1 LOW=0x18 LOWEST=0x1f
    uint8_t none : 3;                 // 空 0
} __AC_PACKED _EscServiceFrame;

typedef struct
{                                  // 尾部数据
    uint8_t Transfer_id : 5;       // 对于相同id的数据，每发一包数据加一，0~31循环累加；同一包数据中的多帧该值不变
    uint8_t Toggle_bit : 1;        // 单帧传输=0；多帧传输，首帧=0，此后每发一帧翻转一次
    uint8_t End_of_transfer : 1;   // 单帧传输=1；多帧传输，最后一帧=1，否则=0
    uint8_t Start_of_transfer : 1; // 单帧传输=1；多帧传输，首帧=1，否则=0
} __AC_PACKED _EscTailByte;

// UAVCAN RawCommand DTID and signature
#define RAW_COMMAND_DTID 1030
#define RAW_COMMAND_SIGNATURE 0x217F5C87D7EC951D

#define ArmingStatus_DTID 1100

#define ESC_Status_DTID 1034
#define ESC_Status_SIGNATURE 0xA9AF28AEA2FBB254

static uint16_t crcAddByte(uint16_t crc_val, uint8_t byte)
{
    crc_val ^= (uint16_t) ((uint16_t) (byte) << 8);
    for (int j = 0; j < 8; j++)
    {
        if (crc_val & 0x8000U)
        {
            crc_val = (uint16_t) ((uint16_t) (crc_val << 1) ^ 0x1021U);
        }
        else
        {
            crc_val = (uint16_t) (crc_val << 1);
        }
    }
    return crc_val;
}

static uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature)
{
    for (int shift_val = 0; shift_val < 64; shift_val += 8)
    {
        crc_val = crcAddByte(crc_val, (uint8_t) (data_type_signature >> shift_val));
    }
    return crc_val;
}
static uint16_t crcAdd(uint16_t crc_val, const uint8_t *bytes, size_t len)
{
    while (len--)
    {
        crc_val = crcAddByte(crc_val, *bytes++);
    }
    return crc_val;
}

static float half_to_float(uint16_t h)
{
    uint32_t sign = (h & 0x8000) << 16;
    uint32_t exp = (h & 0x7C00) >> 10;
    uint32_t frac = (h & 0x03FF);

    uint32_t f;

    if (exp == 0)
    {
        if (frac == 0)
        {
            // zero
            f = sign;
        }
        else
        {
            // subnormal
            exp = 127 - 15 + 1;
            while ((frac & 0x0400) == 0)
            {
                frac <<= 1;
                exp--;
            }
            frac &= 0x03FF;
            f = sign | (exp << 23) | (frac << 13);
        }
    }
    else if (exp == 31)
    {
        // inf / NaN
        f = sign | 0x7F800000 | (frac << 13);
    }
    else
    {
        // normal
        exp = exp - 15 + 127;
        f = sign | (exp << 23) | (frac << 13);
    }

    float result;
    *(uint32_t *) &result = f;
    return result;
}

#define MAX_ESC_COUNT 16
#define COUNTER_RANGE 32 // 计数器范围

struct PacketLossStats
{
    uint8_t last_counter;   // 上一次接收到的计数器值
    uint32_t total_packets; // 总包数统计
    uint32_t lost_packets;  // 丢包数统计
    float loss_rate;        // 当前丢包率(%)
    int quality;            // 当前通信质量(0-100)
    bool is_first_packet;   // 是否是第一个包(特殊处理标志)
} __AC_PACKED;

struct EscMSG
{
    uint16_t crc;
    uint32_t error_count;
    float voltage;
    float current;
    float temperature;
    int32_t rpm;
    uint16_t power_rating_pct;
    uint8_t esc_index;
};

struct EscInfoSend
{
    uint8_t index;
    uint8_t comQuality;
    uint16_t errorFlag;
    int32_t motorRpm;
    float voltage;
    float current;
    int16_t escTemp;
    int16_t motorTemp;
    uint32_t newErrorFlag;
    uint32_t newErrorFlag2;
    uint32_t errorFlagVersion; // errorFlagVersion==UINT32_MAX时, 启用newErrorFlag 和 newErrorFlag2
} __AC_PACKED;

struct EscInfo
{
    EscInfoSend send;
    PacketLossStats LossStats;

    // 电调信息
    EscMSG escmsg;
    uint8_t escmsg_buff[14];
    uint8_t escmsg_step;

} __AC_PACKED;

struct EscsInfo
{
    uint8_t currentRegInd;
    EscInfo escs[MAX_ESC_COUNT];
};

static void init_packet_loss_stats(PacketLossStats *stats)
{
    stats->last_counter = 0;
    stats->total_packets = 0;
    stats->lost_packets = 0;
    stats->loss_rate = 0.0f;
    stats->quality = 100; // 初始质量为100%
    stats->is_first_packet = true;
}

static EscInfo *register_esc(EscsInfo *escsInfo, uint8_t actual_id)
{
    // 检查id是否已经存在
    for (int i = 0; i < MAX_ESC_COUNT; ++i)
    {
        if (actual_id == escsInfo->escs[i].send.index)
        {
            return &escsInfo->escs[i];
        }
    }

    // 检查是否已经满
    if (escsInfo->currentRegInd >= MAX_ESC_COUNT)
    {
        return 0;
    }

    ++escsInfo->currentRegInd;

    uint8_t ind = escsInfo->currentRegInd - 1;
    escsInfo->escs[ind].send.index = actual_id;
    escsInfo->escs[ind].send.errorFlag = 0;
    escsInfo->escs[ind].send.newErrorFlag = 0;
    escsInfo->escs[ind].send.newErrorFlag2 = 0;
    escsInfo->escs[ind].send.errorFlagVersion = UINT32_MAX;
    init_packet_loss_stats(&escsInfo->escs[ind].LossStats);

    return &escsInfo->escs[ind];
}

static int update_packet_loss_stats(PacketLossStats *stats, uint8_t current_counter)
{
    // 第一个包特殊处理
    if (stats->is_first_packet)
    {
        stats->last_counter = current_counter;
        stats->total_packets = 1;
        stats->is_first_packet = false;
        stats->quality = 100;
        return stats->quality;
    }

    // 计算预期下一个计数器值
    uint8_t expected_counter = (stats->last_counter + 1) % COUNTER_RANGE;

    // 检测丢包情况
    if (current_counter != expected_counter)
    {
        // 计算丢失的包数(考虑循环特性)
        int8_t lost = current_counter - expected_counter;
        if (lost < 0)
            lost += COUNTER_RANGE;

        stats->lost_packets += lost;
    }

    // 更新统计值
    stats->total_packets++;
    stats->last_counter = current_counter;

    // 计算丢包率(百分比)
    stats->loss_rate = (float) stats->lost_packets / stats->total_packets * 100.0f;

    // 计算通信质量(可根据需求调整算法)
    stats->quality = 100 - stats->loss_rate; //(int)(stats->loss_rate * 1.5); // 例如:5%丢包→92.5质量
    if (stats->quality < 0)
        stats->quality = 0;

    return stats->quality;
}

static EscInfo *get_registered_esc(EscsInfo *escsInfo, uint8_t actual_id)
{
    // 检查id
    for (int i = 0; i < MAX_ESC_COUNT; ++i)
    {
        if (actual_id == escsInfo->escs[i].send.index)
        {
            return &escsInfo->escs[i];
        }
    }
    return 0;
}

static void extIO_UAVCanESC_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *) pvParameters;
    delete (DriverInfo *) pvParameters;

    uint8_t escCnt = 0;

    // 电调信息
    EscsInfo escsInfo = {0};
    memset(&escsInfo, 0, sizeof(EscsInfo));

    TIME lastGcsSdTIME = TIME::now();

    // 状态机
    uint16_t pwm_send_counter = 0;
    uint16_t disarm_send_counter = 0;
    uint16_t param_send_counter = 0;
    uint16_t param_counter = 0;
    uint16_t escGBits_counter = 0;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    // uavcan电调没初始化完发油门过去会傻逼
    os_delay(10);
    while (1)
    {
        // 800hz
        vTaskDelayUntil(&xLastWakeTime, 1);

        // 循环进度落后过多时(如CAN拔线发送阻塞)重置节拍基准, 避免恢复后连发补帧
        TickType_t nowTick = xTaskGetTickCount();
        if ((TickType_t) (nowTick - xLastWakeTime) > 2)
            xLastWakeTime = nowTick;

// PWM设置
#define PWM_Db_Down 0

        bool attCtrlEna;
        is_Attitude_Control_Enabled(&attCtrlEna);

        if (++pwm_send_counter >= 6)
        {
            pwm_send_counter = 0;

            extern volatile float PWM_CHANS[MAX_PWM_COUNT];
            uint8_t src[16] = {0};
            uint8_t throttle_data[14] = {0};
            int index = 0;
            bool arming = false;

            for (int i = 0; i < 8; ++i)
            {
                int16_t throttle;
                if (i < get_MainMotorCount() && PWM_CHANS[i] >= -20 && PWM_CHANS[i] <= 120)
                {
                    if (PWM_CHANS[i] > PWM_Db_Down)
                        throttle = (PWM_CHANS[i] - PWM_Db_Down) * (8191.0 / (100 - PWM_Db_Down));
                    else
                        throttle = 0;
                    throttle = constrain(throttle, (int16_t) 0, (int16_t) 8191);

                    arming = true;
                }
                else
                {
                    throttle = 0;
                }

                src[index] = (throttle & 0xff);
                index++;
                src[index] = ((throttle >> 8) & 0xff);
                index++;
            }

            if (++disarm_send_counter >= 133)
            {
                disarm_send_counter = 0;

                if (arming == true)
                { // 发送解锁
                    CanPacket mail;
                    mail.IdType = 1;
                    mail.FrameType = 0;
                    mail.FDFormat = 0;

                    _EscMessageFrame frame;
                    frame.none = 0;
                    frame.Priority = 0;
                    frame.Message_type_id = ArmingStatus_DTID;
                    frame.notBroadcast = 0;
                    frame.Soure_node_id = FC_ID;
                    mail.Identifier = *(uint32_t *) &frame;

                    _EscTailByte tail;
                    tail.Start_of_transfer = 1;
                    tail.End_of_transfer = 1;
                    tail.Toggle_bit = 0;
                    tail.Transfer_id = 0;

                    mail.DataLength = 2;
                    mail.data[0] = 255;
                    mail.data[1] = *(uint8_t *) &tail;

                    driver_info.mail_box->SendMail(mail);
                }
                else
                { // 上锁
                    CanPacket mail;
                    mail.IdType = 1;
                    mail.FrameType = 0;
                    mail.FDFormat = 0;

                    _EscMessageFrame frame;
                    frame.none = 0;
                    frame.Priority = 0;
                    frame.Message_type_id = ArmingStatus_DTID;
                    frame.notBroadcast = 0;
                    frame.Soure_node_id = FC_ID;
                    mail.Identifier = *(uint32_t *) &frame;

                    _EscTailByte tail;
                    tail.Start_of_transfer = 1;
                    tail.End_of_transfer = 1;
                    tail.Toggle_bit = 0;
                    tail.Transfer_id = 0;

                    mail.DataLength = 2;
                    mail.data[0] = 0;
                    mail.data[1] = *(uint8_t *) &tail;

                    driver_info.mail_box->SendMail(mail);
                }
            }

            throttle_data[0] = src[0];
            throttle_data[1] = (src[1] & 0x3f) << 2 | (src[2] >> 6);
            throttle_data[2] = (src[2] & 0x3f) << 2 | (src[3] & 0x3f) >> 4;
            throttle_data[3] = (src[3] & 0xf) << 4 | (src[4] & 0xf0) >> 4;
            throttle_data[4] = (src[4] & 0xf) << 4 | (src[5] & 0x3f) >> 2;
            throttle_data[5] = (src[5] & 0x3) << 6 | (src[6] & 0xfc) >> 2;
            throttle_data[6] = (src[6] & 0x3) << 6 | (src[7] & 0x3f);

            throttle_data[7] = src[8];
            throttle_data[8] = (src[9] & 0x3f) << 2 | (src[10] >> 6);
            throttle_data[9] = (src[10] & 0x3f) << 2 | (src[11] & 0x3f) >> 4;
            throttle_data[10] = (src[11] & 0xf) << 4 | (src[12] & 0xf0) >> 4;
            throttle_data[11] = (src[12] & 0xf) << 4 | (src[13] & 0x3f) >> 2;
            throttle_data[12] = (src[13] & 0x3) << 6 | (src[14] & 0xfc) >> 2;
            throttle_data[13] = (src[14] & 0x3) << 6 | (src[15] & 0x3f);

            // crc
            uint16_t signatureCrc = crcAddSignature(0xffff, RAW_COMMAND_SIGNATURE);
            uint16_t crc = crcAdd(signatureCrc, (uint8_t *) &throttle_data[0], 14);

            CanPacket mail;
            mail.IdType = 1;
            mail.FrameType = 0;
            mail.FDFormat = 0;

            _EscMessageFrame frame;
            frame.none = 0;
            frame.Priority = 0;
            frame.Message_type_id = RAW_COMMAND_DTID;
            frame.notBroadcast = 0;
            frame.Soure_node_id = FC_ID;
            mail.Identifier = *(uint32_t *) &frame;

            // 单帧
            //            _EscTailByte tail;
            //            tail.Start_of_transfer = 1;
            //            tail.End_of_transfer = 1;
            //            tail.Toggle_bit = 0;
            //            tail.Transfer_id = 0;

            //            mail.DataLength = 8;
            //            memcpy((uint8_t *)&mail.data[0], (const uint8_t *)&throttle_data[0], 7);
            //            mail.data[7] = *(uint8_t *)&tail;
            //            driver_info.mail_box->SendMail(mail);

            // 多帧第一包
            mail.DataLength = 8;
            mail.data[0] = crc & 0xff;
            mail.data[1] = (crc & 0xff00) >> 8;
            memcpy((uint8_t *) &mail.data[2], (const uint8_t *) &throttle_data[0], 5);

            _EscTailByte tail;
            tail.Start_of_transfer = 1;
            tail.End_of_transfer = 0;
            tail.Toggle_bit = 0;
            tail.Transfer_id = 0;
            mail.data[7] = *(uint8_t *) &tail;
            driver_info.mail_box->SendMail(mail);
            // 多帧第二包
            mail.DataLength = 8;
            memcpy((uint8_t *) &mail.data[0], (const uint8_t *) &throttle_data[5], 7);

            tail.Start_of_transfer = 0;
            tail.End_of_transfer = 0;
            tail.Toggle_bit = 1;
            tail.Transfer_id = 0;
            mail.data[7] = *(uint8_t *) &tail;
            driver_info.mail_box->SendMail(mail);
            // 多帧第三包
            mail.DataLength = 3;
            memcpy((uint8_t *) &mail.data[0], (const uint8_t *) &throttle_data[12], 2);

            tail.Start_of_transfer = 0;
            tail.End_of_transfer = 1;
            tail.Toggle_bit = 0;
            tail.Transfer_id = 0;
            mail.data[2] = *(uint8_t *) &tail;
            driver_info.mail_box->SendMail(mail);
        }

        // Drain all pending frames each cycle (ESC status frames arrive in bursts).
        // Cap per cycle = queue depth, bounds worst-case bus flood.
        CanPacket mail;
        uint8_t rx_drain_cnt = 0;
        while (rx_drain_cnt < 30 && driver_info.mail_box->receiveMail(&mail, 0))
        {
            ++rx_drain_cnt;
            _EscMessageFrame *frame = (_EscMessageFrame *) &mail.Identifier;

            // 尝试注册电调
            EscInfo *escInfo = register_esc(&escsInfo, frame->Soure_node_id);

            if (escInfo && mail.DataLength > 0 && mail.DataLength <= 8)
            {
                switch (frame->Message_type_id)
                {
                    case ESC_Status_DTID:
                    {
                        _EscTailByte *tail = (_EscTailByte *) &mail.data[mail.DataLength - 1];
                        if (tail->Start_of_transfer == 1 && tail->End_of_transfer == 0)
                        { // 包1
                            if (mail.DataLength == 8)
                            {
                                memcpy((uint8_t *) &escInfo->escmsg_buff[0], (uint8_t *) &mail.data[2], 5);

                                escInfo->escmsg.crc = *(uint16_t *) &mail.data[0];
                                escInfo->escmsg.error_count = *(uint32_t *) &mail.data[2];

                                escInfo->escmsg_step = 1;
                            }
                            else
                                escInfo->escmsg_step = 0;
                        }
                        else if (tail->Start_of_transfer == 0 && tail->End_of_transfer == 0)
                        { // 包2
                            if (mail.DataLength == 8 && escInfo->escmsg_step == 1)
                            {
                                memcpy((uint8_t *) &escInfo->escmsg_buff[5], (uint8_t *) &mail.data[0], 7);

                                uint8_t buff[2];
                                buff[0] = escInfo->escmsg_buff[4];
                                buff[1] = mail.data[0];

                                // escInfo->escmsg.voltage = *(_Float16 *)&buff[0];
                                // escInfo->escmsg.current = *(_Float16 *)&mail.data[1];
                                // escInfo->escmsg.temperature = *(_Float16 *)&mail.data[3] - 273.15;
                                escInfo->escmsg.voltage = half_to_float(*(uint16_t *) &buff[0]);
                                {
                                    uint16_t _v16;
                                    memcpy(&_v16, &mail.data[1], 2);
                                    escInfo->escmsg.current = half_to_float(_v16);
                                }
                                {
                                    uint16_t _v16;
                                    memcpy(&_v16, &mail.data[3], 2);
                                    escInfo->escmsg.temperature = half_to_float(_v16) - 273.15;
                                }

                                escInfo->escmsg_step++;
                            }
                            else
                                escInfo->escmsg_step = 0;
                        }
                        else if (tail->End_of_transfer == 1)
                        { // 包3
                            if (mail.DataLength == 3 && escInfo->escmsg_step == 2)
                            {
                                memcpy((uint8_t *) &escInfo->escmsg_buff[12], (uint8_t *) &mail.data[0], 2);

                                // 求电机转速
                                int32_t rpm = ((escInfo->escmsg_buff[12] & 0xC0) << 11) | (escInfo->escmsg_buff[11] << 8) | escInfo->escmsg_buff[10];
                                rpm = rpm & 0x3FFFF;
                                if (rpm & 0x20000)
                                    rpm |= 0xFFFC0000;
                                else
                                    rpm &= 0x3FFFF;

                                escInfo->escmsg.rpm = rpm;
                                escInfo->escmsg.power_rating_pct = ((escInfo->escmsg_buff[12] & 0x3f) << 1) | ((escInfo->escmsg_buff[13] & 0x80) >> 7);
                                escInfo->escmsg.esc_index = (escInfo->escmsg_buff[13] & 0x7c) >> 2;

                                // crc
                                uint16_t signatureCrc = crcAddSignature(0xffff, ESC_Status_SIGNATURE);
                                uint16_t crc = crcAdd(signatureCrc, (uint8_t *) &escInfo->escmsg_buff[0], 14);

                                if (escInfo->escmsg.crc == crc)
                                {
                                    escInfo->send.comQuality = update_packet_loss_stats(&escInfo->LossStats, tail->Transfer_id);
                                    escInfo->send.errorFlag = 0;
                                    escInfo->send.motorRpm = escInfo->escmsg.rpm;
                                    escInfo->send.voltage = escInfo->escmsg.voltage;
                                    escInfo->send.current = escInfo->escmsg.current;
                                    escInfo->send.escTemp = escInfo->escmsg.temperature * 10;
                                    escInfo->send.motorTemp = escInfo->escmsg.error_count;

                                    if (attCtrlEna)
                                    {
                                        double logMsg[7];
                                        logMsg[0] = escInfo->escmsg.error_count;
                                        logMsg[1] = escInfo->escmsg.voltage;
                                        logMsg[2] = escInfo->escmsg.current;
                                        logMsg[3] = escInfo->escmsg.temperature;
                                        logMsg[4] = escInfo->escmsg.rpm;
                                        logMsg[5] = escInfo->escmsg.power_rating_pct;
                                        logMsg[6] = escInfo->escmsg.esc_index;

                                        char logName[16];
                                        sprintf(logName, "ESC%dSta", frame->Soure_node_id);
                                        SDLog_Msg_DebugVect(logName, logMsg, 7);
                                    }
                                }
                            }
                            escInfo->escmsg_step = 0;
                        }

                        break;
                    }
                }
            }
        }

        if (lastGcsSdTIME.get_pass_time() > 1.5)
        {
            lastGcsSdTIME = TIME::now();

            for (uint8_t ind = 0; ind < escsInfo.currentRegInd; ind++)
            {
                uint8_t payload[43];
                memset(payload, 0, 43);
                memcpy(payload, (uint8_t *) &escsInfo.escs[ind], sizeof(EscInfoSend));

                for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                {
                    const Port *port = get_CommuPort(i);
                    if (port && port->write != 0)
                    {
                        if (mavlink_lock_chan(i, 0.01))
                        {
                            mavlink_message_t msg_sd;
                            mavlink_msg_component_extension43_pack_chan(get_CommulinkSysId(),  // system id
                                                                        get_CommulinkCompId(), // component id
                                                                        i,                     // chan
                                                                        &msg_sd,
                                                                        0,                    // target_network
                                                                        get_CommulinkSysId(), // target system
                                                                        80,                   // target component
                                                                        80,                   // message_type
                                                                        payload);
                            mavlink_send_msg(port, &msg_sd, 0.01, 0.01);
                            mavlink_unlock_chan(i);
                        }
                    }
                }
            }
        }
    }
}

static bool extIO_UAVCanESC_DriverInit()
{
    return true;
}
static bool extIO_UAVCanESC_DriverRun(CanMailBox *mailBox)
{
    CanIdWithMask Ids[1];

    _EscMessageFrame frame = {0};
    _EscMessageFrame maskFrame = {0};
    maskFrame.notBroadcast = 1;
    maskFrame.Message_type_id = 0xffff;

    // ESCStatus
    frame.Message_type_id = ESC_Status_DTID;
    frame.notBroadcast = 0;
    Ids[0].IdType = 1;
    Ids[0].mask = *(uint32_t *) &maskFrame;
    Ids[0].Identifier = *(uint32_t *) &frame;

    // Queue depth must cover one full ESC burst (multi-frame status messages)
    mailBox->init(30, Ids, 1);
    DriverInfo *driver_info = new DriverInfo;
    driver_info->mail_box = mailBox;
    xTaskCreate(extIO_UAVCanESC_Server, "extIO_UAVCanESC", 1600, (void *) driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_extIO_UAVCanESC_C()
{
    CanFunc_Register(18, extIO_UAVCanESC_DriverInit, extIO_UAVCanESC_DriverRun);
}