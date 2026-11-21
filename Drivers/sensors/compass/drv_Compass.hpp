#pragma once

#include "HardWareConfig.hpp"
#include "i2c_common.hpp"

// i2c罗盘协议定义
struct I2C_MAG_PROTOCOL
{
    // 设备
    HW_ConfigItem device;

    // 传感器名称
    SName name;

    // iic地址
    unsigned char device_address;
    // 数据寄存器地址
    unsigned char data_address;

    // 是否高位在前
    bool MSB;
    // 是否需要手动开始采样
    bool need_set_sample;
    // 采样寄存器地址
    unsigned char sample_address;
    // 采样寄存器值
    unsigned char sample_cfg;

    // ID寄存器地址
    unsigned char ID_address;
    // ID
    unsigned char ID;

    // 配置寄存器地址
    unsigned char configurations_addresses[10];
    // 配置数目
    unsigned char configuration_count;
    // 配置
    unsigned char configurations[10];

    // ==================== 新增：状态寄存器轮询支持（抽象 RM3100 等） ====================
    bool need_status_check;           // 是否需要在读数据前检查状态寄存器
    unsigned char status_address;     // 状态寄存器地址
    unsigned char status_ready_mask;  // 就绪位掩码
    unsigned char status_ready_value; // 就绪时应满足的值

    // ==================== 新增：读取数据的字节长度 ====================
    unsigned char data_bytes;
    // 灵敏度（数据->Gauss）
    double sensitivity;

    // 采样率（多少ms一次数据）
    unsigned char sample_mseconds;

    // I2C速度
    I2C_SPEED iic_speed;
};

// 运行时罗盘实例 (协议引用+轴向)
struct I2C_MagnetoMeter
{
    const I2C_MAG_PROTOCOL *protocol;
    int8_t axis_index[3];
    uint8_t addr_override; // 0 = 使用协议默认地址
};

// 罗盘任务参数
class I2C_CLASS;
struct I2C_CompassInfo
{
    I2C_CLASS *i2c;
    I2C_MagnetoMeter *meters;
    uint8_t count;
};

// 支持的I2C罗盘协议表
extern const I2C_MAG_PROTOCOL I2C_Mag_Protocols[];
extern const uint8_t I2C_Mag_Protocol_Count;

// 从HW_CFG_AXIS的value_low解码轴向到int8_t axis[3]
void HW_DecodeAxis(uint16_t value_low, int8_t axis[3]);

void init_drv_Compass();
