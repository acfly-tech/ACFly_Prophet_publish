#pragma once

#include "Basic.hpp"
#include "drv_Uart.hpp"
#include "drv_can.hpp"
#include "mavlink.h"

struct CommulinkConfig
{
    // 通信端口id
    uint16_t sys_id[4];
    uint16_t comp_id[4];

    // 通信模式开关选项
    uint32_t commuCfg[2];

    // 姿态包发送频率
    float usb_attRate[2];
    // IMU包发送频率
    float usb_imuRate[2];
    // 位置包发送频率
    float usb_estPosRate[2];
    // GPS包发送频率
    float usb_rawPosRate[2];
    // 状态包发送频率
    float usb_stateRate[2];
    // 状态包发送频率
    float usb_hudRate[2];
    // 避障数据发送频率
    float usb_obstacleRate[2];
    // 遥控数据包发送频率(RC_CHANNELS/RC_CHANNELS_SCALED/MANUAL_CONTROL)
    float usb_rcRate[2];
    // Odometry包发送频率（msg331 SLAM传感器原始读数）
    float usb_odometryRate[2];

    // 姿态包发送频率
    float uart_attRate[2];
    // IMU包发送频率
    float uart_imuRate[2];
    // 位置包发送频率
    float uart_estPosRate[2];
    // GPS包发送频率
    float uart_rawPosRate[2];
    // 状态包发送频率
    float uart_stateRate[2];
    // 状态包发送频率
    float uart_hudRate[2];
    // 避障数据发送频率
    float uart_obstacleRate[2];
    // 遥控数据包发送频率(RC_CHANNELS/RC_CHANNELS_SCALED/MANUAL_CONTROL)
    float uart_rcRate[2];
    // Odometry包发送频率（msg331 SLAM传感器原始读数）
    float uart_odometryRate[2];

    // Mission Planner兼容模式开关 (按物理端口的位掩码, 0=全部关闭)
    // bit i(i=0..15)=UART i, bit16=USB; 例: 1<<2=4 仅UART2, 1<<16=65536 仅USB, 0x1FFFF=全部
    // 开启后仅对应端口的心跳/HIGH_LATENCY中AUTO主模式置0以兼容MP显示, 其他端口照常上报AUTO
    // 原为收到base_mode==0的心跳自动开启, 现改为参数手动控制
    uint32_t gcsMpMode[2];
};
#define CommuCfg_lowParamSendRateOnUart_BIT (1 << 16)
#define CommuCfg_noParamListSend_BIT (1 << 17)
#define CommuCfg_mavlinkRoute_BIT (1 << 18)
#define CommuCfg_newMavlinkStatus_BIT (1 << 19)

struct CANConfig
{
    uint32_t Baud[2];
    uint32_t DataBaud[2];
    uint32_t Ena1[2];
    uint32_t Ena2[2];
    uint32_t Ena3[2];
    uint32_t Ena4[2];
};

struct I2CConfig
{
    uint32_t Ena1[2];
    uint32_t Ena2[2];
    uint32_t Ena3[2];
    uint32_t Ena4[2];
};

/*声光提示*/
enum LEDSignal
{
    LEDSignal_Start1,
    LEDSignal_Start2,

    LEDSignal_Continue1,
    LEDSignal_Success1,

    LEDSignal_Err1,
    LEDSignal_Err2 // 低电量报警
};
enum LEDMode
{
    // 关闭自动模式
    LEDMode_Manual,

    // 正常模式
    LEDMode_Normal1,
    LEDMode_Normal2,

    // 正在处理
    LEDMode_Processing1,
    LEDMode_Processing2,

    // 飞行模式灯语
    LEDMode_FlyingManualAlt, // 黄灯双闪(定高-无水平定位)
    LEDMode_FlyingManualPos, // 蓝灯双闪(定点-有水平定位)
    LEDMode_FlyingManual,    // 红灯慢闪(手动)
    LEDMode_FlyingAttitude,  // 白灯慢闪(姿态)
    LEDMode_FlyingSemiAuto,  // 黄灯慢闪(半自动)
    LEDMode_FlyingAuto,      // 绿灯慢闪(自动)
    LEDMode_FlyingRTL,       // 紫灯快闪(返航+降落)
};
void sendLedSignal(LEDSignal signal);
void setLedMode(LEDMode mode);
void setLedManualCtrl(float R, float G, float B, bool BuzzerOn, uint16_t BuzzerFreq);
/*声光提示*/

/*端口*/
// 端口定义
typedef struct
{
    // 写端口函数
    uint16_t (*write)(const uint8_t *data, uint16_t length, double Write_waitTime, double Sync_waitTime);
    // 等待发送完成
    bool (*wait_sent)(double waitTime);
    // 获取缓冲区剩余空间
    uint16_t (*txSpacesAvailable)();
    // 发送上锁解锁
    bool (*lock)(double Sync_waitTime);
    void (*unlock)();
    // 读端口函数
    uint16_t (*read)(uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime);
    // 清空接收缓冲区
    bool (*reset_rx)(double Sync_waitTime);
    // 更改波特率函数
    bool (*SetBaudRate)(uint32_t baud_rate, double Send_waitTime, double Sync_waitTime);
    // TX/RX电平反相
    bool (*setTxInvert)(bool invert);
    bool (*setRxInvert)(bool invert);
    // 奇偶校验
    bool (*SetParity)(UartParityMode parity_mode, uint8_t wildcard_byte, double Send_waitTime, double Sync_waitTime);
} Port;
#define Port_isBasicFunc(port) (port.read && port.write && port.lock && port.unlock)
#define Port_isFullFunc(port) (port.read && port.write && port.lock && port.unlock && port.reset_rx && port.wait_sent)

static inline uint16_t mavlink_send_msg(const Port *port, const mavlink_message_t *msg, double syncTimeout = 0, double timeout = 0.01)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    if (len == 0 || !port || !port->write || !port->lock || !port->unlock)
        return 0;
    if (!port->lock(syncTimeout))
        return 0;
    uint16_t written = port->write(buf, len, timeout, syncTimeout);
    port->unlock();
    return (written == len) ? len : 0;
}

#define MAXPorts 10
// 注册端口
bool PortRegister(uint8_t ind, Port port);
/*端口*/

/*CAN功能*/
bool CanFunc_Register(uint8_t FuncInd, bool (*init)(), bool (*run)(CanMailBox *));
/*CAN功能*/

/*I2C功能*/
class I2C_CLASS;
bool I2CFunc_Internal_Register(uint8_t FuncInd, bool (*init)(I2C_CLASS *i2c), bool (*run)(I2C_CLASS *i2c));
bool I2CFunc_Register(uint8_t FuncInd, bool (*init)(I2C_CLASS *i2c), bool (*run)(I2C_CLASS *i2c));
/*I2C功能*/

/*通信端口*/
// 在指定端口设置消息速率
bool SetMsgRate(uint8_t port_index, uint16_t Msg, float RateHz, double TIMEOUT = -1);
// 在指定端口设置消息速率
//  rate: 0-不发送 <0-发送指定次数，-1一次 >0-按分频系数发送,1不分频,2频率/2
bool SetPosSensorMsg(uint8_t port_index, uint8_t ind, int16_t rate, double TIMEOUT = -1);
// 清除所有消息(次数发送除外)
bool ClearPosSensorMsg(uint8_t port_index, uint8_t ind, double TIMEOUT = -1);
// 获取端口
const Port *get_CommuPort(uint8_t port);
const Port *get_CommuPortByPortId(uint8_t portId, uint8_t *ind = 0);

// 设定mavlink模式
bool set_mav_state(MAV_STATE state);
void get_mav_modes(uint16_t &mav_mode, uint16_t &mav_main_mode, uint16_t &mav_sub_mode);

// 获取本机id
uint8_t get_CommulinkSysId();
uint8_t get_CommulinkCompId();

// 发送status text到单个mavlink端口
bool sendStatusText(uint8_t port_index, MAV_SEVERITY severity, const char *text, uint16_t id = 0, uint8_t chunk_seq = 0);
// 广播status text到所有可用的mavlink端口
void sendStatusText_toAll(MAV_SEVERITY severity, const char *text, uint16_t id = 0, uint8_t chunk_seq = 0);
/*通信端口*/

/*RTK端口*/
// 端口定义
typedef struct
{
    bool ena;
    // 写端口函数
    uint16_t (*write)(const uint8_t *data, uint16_t length, double Write_waitTime, double Sync_waitTime);
    // 发送上锁解锁
    bool (*lock)(double Sync_waitTime);
    void (*unlock)();
} RtkPort;

// 注册Rtk端口
int8_t RtkPortRegister(RtkPort port);
// 使能失能Rtk端口
bool RtkPort_setEna(uint8_t port, bool ena);
// 获取端口
const RtkPort *get_RtkPort(uint8_t port);
// 往rtk端口发送注入数据
void inject_RtkPorts(const uint8_t data[], uint16_t length);
/*RTK端口*/

/*功能接口*/
bool PortFunc_Register(uint8_t FuncInd, bool (*init)(Port port, uint32_t param));
/*功能接口*/

void init_Commulink();