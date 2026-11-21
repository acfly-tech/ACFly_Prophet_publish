#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// ====================================================================================
// 协议常量
// ====================================================================================

#define MSP_PORT_INBUF_SIZE 256
#define MSP_PORT_OUTBUF_SIZE 512
#define JUMBO_FRAME_SIZE_LIMIT 255

// API版本（兼容Betaflight）
#define MSP_API_VERSION_MAJOR 1
#define MSP_API_VERSION_MINOR 42

// MSP命令定义
typedef enum
{
    MSP_API_VERSION = 1,
    MSP_FC_VARIANT = 2,
    MSP_FC_VERSION = 3,
    MSP_BOARD_INFO = 4,
    MSP_BUILD_INFO = 5,
    MSP_NAME = 10,
    MSP_SET_NAME = 11,
    MSP_OSD_CONFIG = 84,
    MSP_FILTER_CONFIG = 92,
    MSP_PID_ADVANCED = 94,
    MSP_STATUS = 101,
    MSP_STATUS_EX = 150,
    MSP_RC = 105,
    MSP_RAW_GPS = 106,
    MSP_COMP_GPS = 107,
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_ANALOG = 110,
    MSP_RC_TUNING = 111,
    MSP_PID = 112,
    MSP_BATTERY_STATE = 130,
    MSP_ESC_SENSOR_DATA = 134,
    MSP_DISPLAYPORT = 182, // DisplayPort - 主动推送字符到OSD
    MSP_RTC = 247,
} msp_command_e;

// MSP协议版本
typedef enum
{
    MSP_V1 = 0,
    MSP_V2_OVER_V1 = 1,
    MSP_V2_NATIVE = 2,
} msp_version_e;

// MSP解析状态
typedef enum
{
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,
    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,
    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,
    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,
    MSP_COMMAND_RECEIVED
} msp_state_e;

// MSP命令结果
typedef enum
{
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0
} msp_result_e;

// ====================================================================================
// 数据结构
// ====================================================================================

/**
 * @brief 缓冲区结构
 */
typedef struct
{
    uint8_t *ptr; // 当前指针
    uint8_t *end; // 结束指针
} msp_buf_t;

/**
 * @brief MSP端口状态
 */
typedef struct
{
    msp_state_e state;                     // 解析状态机
    uint8_t in_buf[MSP_PORT_INBUF_SIZE];   // 接收缓冲区
    uint16_t offset;                       // 当前偏移
    uint16_t data_size;                    // 数据大小
    msp_version_e version;                 // 协议版本
    uint8_t cmd_flags;                     // 命令标志
    uint16_t cmd;                          // 当前命令
    uint8_t checksum1;                     // 校验和1
    uint8_t checksum2;                     // 校验和2（CRC8）
    uint8_t out_buf[MSP_PORT_OUTBUF_SIZE]; // 输出缓冲区（用于打包响应）
    uint16_t out_len;                      // 输出数据长度
} msp_port_t;

/**
 * @brief MSP数据包
 */
typedef struct
{
    uint16_t cmd;   // 命令ID
    uint8_t flags;  // 标志位
    int16_t result; // 结果状态
    uint8_t *data;  // 数据指针
    uint16_t len;   // 数据长度
} msp_packet_t;

/**
 * @brief MSP命令处理函数类型
 * @param cmd 命令ID
 * @param buf 数据缓冲区
 * @return 处理结果
 */
typedef int (*msp_cmd_handler_t)(uint16_t cmd, msp_buf_t *buf);

// ====================================================================================
// 平台接口（需要用户实现）
// ====================================================================================

/**
 * @brief 系统时间（毫秒）- 可选，用于定时任务
 * @note 如果不使用定时功能，可以返回固定值
 */
extern uint32_t msp_millis(void);

// ====================================================================================
// 核心API
// ====================================================================================

/**
 * @brief 初始化MSP端口
 * @param port MSP端口结构指针
 */
void msp_init(msp_port_t *port);

/**
 * @brief 解析接收到的字节
 * @param port MSP端口结构指针
 * @param c 接收到的字节
 * @return true=完整命令接收, false=继续接收
 */
bool msp_parse_byte(msp_port_t *port, uint8_t c);

/**
 * @brief 构建MSP响应到输出缓冲区
 * @param port MSP端口结构指针
 * @param packet 数据包指针
 * @return 填充的字节数（保存在port->out_buf中）
 * @note 用户需要通过 msp_get_out_buf() 和 msp_get_out_len() 获取数据后自行发送
 */
uint32_t msp_send_response(msp_port_t *port, const msp_packet_t *packet);

/**
 * @brief 注册命令处理函数
 * @param handler 处理函数指针
 */
void msp_register_handler(msp_cmd_handler_t handler);

/**
 * @brief 处理接收到的命令
 * @param port MSP端口结构指针
 */
void msp_process_command(msp_port_t *port);

// ====================================================================================
// 缓冲区辅助函数
// ====================================================================================

/**
 * @brief 初始化缓冲区
 */
static inline void msp_buf_init(msp_buf_t *buf, uint8_t *data, uint16_t size)
{
    buf->ptr = data;
    buf->end = data + size;
}

/**
 * @brief 写入一个字节
 */
static inline void msp_buf_write_u8(msp_buf_t *buf, uint8_t val)
{
    if (buf->ptr < buf->end)
    {
        *buf->ptr++ = val;
    }
}

/**
 * @brief 写入16位整数（小端序）
 */
static inline void msp_buf_write_u16(msp_buf_t *buf, uint16_t val)
{
    msp_buf_write_u8(buf, val & 0xFF);
    msp_buf_write_u8(buf, (val >> 8) & 0xFF);
}

/**
 * @brief 写入32位整数（小端序）
 */
static inline void msp_buf_write_u32(msp_buf_t *buf, uint32_t val)
{
    msp_buf_write_u8(buf, val & 0xFF);
    msp_buf_write_u8(buf, (val >> 8) & 0xFF);
    msp_buf_write_u8(buf, (val >> 16) & 0xFF);
    msp_buf_write_u8(buf, (val >> 24) & 0xFF);
}

/**
 * @brief 写入数据块
 */
static inline void msp_buf_write_data(msp_buf_t *buf, const void *data, uint16_t len)
{
    const uint8_t *ptr = (const uint8_t *) data;
    while (len-- > 0 && buf->ptr < buf->end)
    {
        *buf->ptr++ = *ptr++;
    }
}

/**
 * @brief 获取已写入的字节数
 */
static inline uint16_t msp_buf_bytes_written(const msp_buf_t *buf, const uint8_t *base)
{
    return buf->ptr - base;
}

/**
 * @brief 读取一个字节
 */
static inline uint8_t msp_buf_read_u8(msp_buf_t *buf)
{
    return (buf->ptr < buf->end) ? *buf->ptr++ : 0;
}

/**
 * @brief 读取16位整数（小端序）
 */
static inline uint16_t msp_buf_read_u16(msp_buf_t *buf)
{
    uint16_t val = msp_buf_read_u8(buf);
    val |= (uint16_t) msp_buf_read_u8(buf) << 8;
    return val;
}

/**
 * @brief 读取32位整数（小端序）
 */
static inline uint32_t msp_buf_read_u32(msp_buf_t *buf)
{
    uint32_t val = msp_buf_read_u8(buf);
    val |= (uint32_t) msp_buf_read_u8(buf) << 8;
    val |= (uint32_t) msp_buf_read_u8(buf) << 16;
    val |= (uint32_t) msp_buf_read_u8(buf) << 24;
    return val;
}

// ====================================================================================
// 输出缓冲区访问函数
// ====================================================================================

/**
 * @brief 获取输出缓冲区指针
 * @param port MSP端口结构指针
 * @return 输出缓冲区指针
 * @note 在调用 msp_send_response() 后使用，获取打包好的字节流
 */
static inline const uint8_t *msp_get_out_buf(const msp_port_t *port)
{
    return port->out_buf;
}

/**
 * @brief 获取输出数据长度
 * @param port MSP端口结构指针
 * @return 输出数据长度（字节）
 */
static inline uint16_t msp_get_out_len(const msp_port_t *port)
{
    return port->out_len;
}

/**
 * @brief CRC8校验（用于MSP V2）
 */
uint8_t msp_crc8(uint8_t crc, uint8_t byte);
uint8_t msp_crc8_buf(uint8_t crc, const uint8_t *data, uint16_t len);
