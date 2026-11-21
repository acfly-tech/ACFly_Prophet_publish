#include "drv_msp.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "StorageSystem.hpp"
#include "task.h"

// CRC8-DVB-S2查找表
static const uint8_t crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};

// 全局命令处理函数
static msp_cmd_handler_t g_handler = NULL;

/**
 * @brief CRC8校验
 */
uint8_t msp_crc8(uint8_t crc, uint8_t byte)
{
    return crc8_table[crc ^ byte];
}

/**
 * @brief CRC8批量校验
 */
uint8_t msp_crc8_buf(uint8_t crc, const uint8_t *data, uint16_t len)
{
    while (len-- > 0)
    {
        crc = msp_crc8(crc, *data++);
    }
    return crc;
}

/**
 * @brief 初始化MSP端口
 */
void msp_init(msp_port_t *port)
{
    memset(port, 0, sizeof(msp_port_t));
    port->state = MSP_IDLE;
    port->version = MSP_V1;
}

/**
 * @brief 解析接收到的字节（状态机）
 */
bool msp_parse_byte(msp_port_t *port, uint8_t c)
{
    switch (port->state)
    {
        case MSP_IDLE:
            if (c == '$')
            {
                port->version = MSP_V1;
                port->state = MSP_HEADER_START;
            }
            break;

        case MSP_HEADER_START:
            if (c == 'M')
            {
                port->state = MSP_HEADER_M;
            }
            else if (c == 'X')
            {
                port->state = MSP_HEADER_X;
            }
            else
            {
                port->state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_M:
            if (c == '<')
            {
                port->offset = 0;
                port->checksum1 = 0;
                port->checksum2 = 0;
                port->state = MSP_HEADER_V1;
            }
            else
            {
                port->state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_X:
            if (c == '<')
            {
                port->offset = 0;
                port->checksum2 = 0;
                port->version = MSP_V2_NATIVE;
                port->state = MSP_HEADER_V2_NATIVE;
            }
            else
            {
                port->state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V1:
            port->in_buf[port->offset++] = c;
            port->checksum1 ^= c;
            if (port->offset == 2)
            {
                port->data_size = port->in_buf[0];
                port->cmd = port->in_buf[1];

                if (port->data_size == JUMBO_FRAME_SIZE_LIMIT)
                {
                    port->state = MSP_PAYLOAD_V1;
                }
                else if (port->data_size == 0)
                {
                    port->state = MSP_CHECKSUM_V1;
                }
                else if (port->cmd == 255)
                {
                    port->version = MSP_V2_OVER_V1;
                    port->state = MSP_HEADER_V2_OVER_V1;
                }
                else
                {
                    port->state = MSP_PAYLOAD_V1;
                }
            }
            break;

        case MSP_PAYLOAD_V1:
            port->in_buf[port->offset++] = c;
            port->checksum1 ^= c;
            if (port->offset >= (2 + port->data_size))
            {
                port->state = MSP_CHECKSUM_V1;
            }
            break;

        case MSP_CHECKSUM_V1:
            if (port->checksum1 == c)
            {
                port->state = MSP_COMMAND_RECEIVED;
                return true;
            }
            port->state = MSP_IDLE;
            break;

        case MSP_HEADER_V2_NATIVE:
            port->in_buf[port->offset++] = c;
            port->checksum2 = msp_crc8(port->checksum2, c);
            if (port->offset == 5)
            {
                port->cmd_flags = port->in_buf[0];
                port->cmd = port->in_buf[1] | (port->in_buf[2] << 8);
                port->data_size = port->in_buf[3] | (port->in_buf[4] << 8);

                if (port->data_size == 0)
                {
                    port->state = MSP_CHECKSUM_V2_NATIVE;
                }
                else
                {
                    port->state = MSP_PAYLOAD_V2_NATIVE;
                }
            }
            break;

        case MSP_PAYLOAD_V2_NATIVE:
            port->in_buf[port->offset++] = c;
            port->checksum2 = msp_crc8(port->checksum2, c);
            if (port->offset >= (5 + port->data_size))
            {
                port->state = MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_CHECKSUM_V2_NATIVE:
            if (port->checksum2 == c)
            {
                port->state = MSP_COMMAND_RECEIVED;
                return true;
            }
            port->state = MSP_IDLE;
            break;

        default:
            port->state = MSP_IDLE;
            break;
    }

    return false;
}

/**
 * @brief 构建MSP响应到输出缓冲区
 */
uint32_t msp_send_response(msp_port_t *port, const msp_packet_t *packet)
{
    uint8_t *ptr = port->out_buf;
    uint8_t *buf_end = port->out_buf + MSP_PORT_OUTBUF_SIZE;

    // 检查缓冲区空间
    if ((packet->len + 10) > MSP_PORT_OUTBUF_SIZE)
    {
        port->out_len = 0;
        return 0;
    }

    // 帧头
    *ptr++ = '$';
    *ptr++ = 'M';
    *ptr++ = (packet->result >= 0) ? '>' : '!';

    if (port->version == MSP_V1)
    {
        // MSP V1格式
        *ptr++ = packet->len;
        *ptr++ = packet->cmd;

        uint8_t checksum = packet->len ^ packet->cmd;
        for (uint16_t i = 0; i < packet->len; i++)
        {
            *ptr++ = packet->data[i];
            checksum ^= packet->data[i];
        }
        *ptr++ = checksum;
    }
    else if (port->version == MSP_V2_OVER_V1)
    {
        // MSP V2 over V1格式
        uint16_t v1_size = 5 + packet->len + 1;

        *ptr++ = (v1_size < JUMBO_FRAME_SIZE_LIMIT) ? v1_size : JUMBO_FRAME_SIZE_LIMIT;
        *ptr++ = 255;

        if (v1_size >= JUMBO_FRAME_SIZE_LIMIT)
        {
            *ptr++ = v1_size & 0xFF;
            *ptr++ = (v1_size >> 8) & 0xFF;
        }

        uint8_t *v2_start = ptr;
        *ptr++ = packet->flags;
        *ptr++ = packet->cmd & 0xFF;
        *ptr++ = (packet->cmd >> 8) & 0xFF;
        *ptr++ = packet->len & 0xFF;
        *ptr++ = (packet->len >> 8) & 0xFF;

        memcpy(ptr, packet->data, packet->len);
        ptr += packet->len;

        uint8_t crc = msp_crc8_buf(0, v2_start, 5 + packet->len);
        *ptr++ = crc;

        uint8_t checksum = 0;
        for (uint8_t *p = &port->out_buf[3]; p < ptr; p++)
        {
            checksum ^= *p;
        }
        *ptr++ = checksum;
    }
    else if (port->version == MSP_V2_NATIVE)
    {
        // MSP V2 Native格式
        uint8_t *v2_start = ptr;
        *ptr++ = packet->flags;
        *ptr++ = packet->cmd & 0xFF;
        *ptr++ = (packet->cmd >> 8) & 0xFF;
        *ptr++ = packet->len & 0xFF;
        *ptr++ = (packet->len >> 8) & 0xFF;

        memcpy(ptr, packet->data, packet->len);
        ptr += packet->len;

        uint8_t crc = msp_crc8_buf(0, v2_start, 5 + packet->len);
        *ptr++ = crc;
    }

    // 保存输出数据长度
    port->out_len = ptr - port->out_buf;
    return port->out_len;
}

/**
 * @brief 注册命令处理函数
 */
void msp_register_handler(msp_cmd_handler_t handler)
{
    g_handler = handler;
}

/**
 * @brief 处理接收到的命令
 */
void msp_process_command(msp_port_t *port)
{
    if (g_handler == NULL)
    {
        return;
    }

    uint8_t out_buf[MSP_PORT_OUTBUF_SIZE];
    msp_buf_t buf;
    msp_buf_init(&buf, out_buf, sizeof(out_buf));

    // 调用处理函数
    int result = g_handler(port->cmd, &buf);

    // 构造响应包
    msp_packet_t packet;
    packet.cmd = port->cmd;
    packet.flags = 0;
    packet.result = result;
    packet.data = out_buf;
    packet.len = msp_buf_bytes_written(&buf, out_buf);

    // 发送响应
    msp_send_response(port, &packet);

    // 重置状态机
    port->state = MSP_IDLE;
}
