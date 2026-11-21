#include "drv_Compass.hpp"
#include "Basic.hpp"

// i2c
#include "drv_compass_ExtMag_I.hpp"
#include "drv_compass_InternalMag_I.hpp"

/*======================== I2C罗盘协议表 ========================*/
// 所有支持的I2C罗盘协议定义 (通用表, 不区分内外罗盘, 由配置文件决定启用哪些)
// clang-format off
const I2C_MAG_PROTOCOL I2C_Mag_Protocols[] = {
    // QMC5883P
    {
        HW_CFG_QMC5883P,                                         // device
        "QMC5883P",                                              // 名称
        0x2C,                                                    // iic地址
        1,                                                       // 数据寄存器
        false,                                                   // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        0x00,                                                    // ID地址
        0x80,                                                    // ID
        {0x29, 0xb, 0xa},                                        // 配置寄存器地址
        3,                                                       // 配置数
        {0x06, (0b11 << 2), (0b10 << 2) | (0b11 << 0)},         // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        8.33333333e-5,                                           // 灵敏度(Gauss/LSB)
        10,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // QMC5883
    {
        HW_CFG_QMC5883,                                          // device
        "QMC5883",                                               // 名称
        0x0d,                                                    // iic地址
        0,                                                       // 数据寄存器
        false,                                                   // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        0x0d,                                                    // ID地址
        0xff,                                                    // ID
        {0x9, 0xa, 0xb},                                         // 配置寄存器地址
        3,                                                       // 配置数
        {(0 << 6) | (0 << 4) | (0b10 << 2) | (1 << 0), (1 << 6), 1}, // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        8.33333333e-5,                                           // 灵敏度(Gauss/LSB)
        10,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // FXOS8700
    {
        HW_CFG_FXOS8700,                                         // device
        "FXOS8700",                                              // 名称
        0x1c,                                                    // iic地址
        0x33,                                                    // 数据寄存器
        true,                                                    // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        0x0d,                                                    // ID地址
        0xc7,                                                    // ID
        {0x2a, 0x5b},                                            // 配置寄存器地址
        2,                                                       // 配置数
        {(3 << 3) | (1 << 0), (7 << 2) | (1 << 0)},             // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        0.001,                                                   // 灵敏度(Gauss/LSB)
        30,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // QMC6983
    {
        HW_CFG_QMC6983,                                          // device
        "QMC6983",                                               // 名称
        0x2c,                                                    // iic地址
        0,                                                       // 数据寄存器
        false,                                                   // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        0x0c,                                                    // ID地址
        0x01,                                                    // ID
        {0x0b, 0x09, 0x0a},                                      // 配置寄存器地址
        3,                                                       // 配置数
        {0xff, (0 << 6) | (1 << 4) | (3 << 2) | (1 << 0), (1 << 6)}, // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        0.0004,                                                  // 灵敏度(Gauss/LSB)
        20,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // HMC5983
    {
        HW_CFG_HMC5983,                                          // device
        "HMC5983",                                               // 名称
        0x1e,                                                    // iic地址
        3,                                                       // 数据寄存器
        true,                                                    // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        0,                                                       // ID地址
        0x10,                                                    // ID
        {0, 1, 2},                                               // 配置寄存器地址
        3,                                                       // 配置数
        {(1 << 7) | (0 << 5) | (7 << 2) | (0 << 0), (1 << 5), 0}, // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        0.00092,                                                 // 灵敏度(Gauss/LSB)
        5,                                                       // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // HMC5883
    {
        HW_CFG_HMC5883,                                          // device
        "HMC5883",                                               // 名称
        0x1e,                                                    // iic地址
        3,                                                       // 数据寄存器
        true,                                                    // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        0,                                                       // ID地址
        0xf0,                                                    // ID
        {0, 1, 2},                                               // 配置寄存器地址
        3,                                                       // 配置数
        {(0 << 7) | (0 << 5) | (6 << 2) | (0 << 0), (1 << 5), 0}, // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        0.00092,                                                 // 灵敏度(Gauss/LSB)
        20,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // Mag3110
    {
        HW_CFG_MAG3110,                                          // device
        "Mag3110",                                               // 名称
        0x0e,                                                    // iic地址
        1,                                                       // 数据寄存器
        true,                                                    // MSB高位在前
        false,                                                   // 手动采样
        0,                                                       // 采样地址
        0,                                                       // 采样值
        7,                                                       // ID地址
        0xc4,                                                    // ID
        {0x10},                                                  // 配置寄存器地址
        1,                                                       // 配置数
        {(0 << 5) | (0 << 3) | (0 << 2) | (0 << 1) | (1 << 0)}, // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        0.001,                                                   // 灵敏度(Gauss/LSB)
        10,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // IST8310
    {
        HW_CFG_IST8310,                                          // device
        "IST8310",                                               // 名称
        0x0e,                                                    // iic地址
        3,                                                       // 数据寄存器
        false,                                                   // MSB高位在前
        true,                                                    // 手动采样
        0xa,                                                     // 采样地址
        1,                                                       // 采样值
        0,                                                       // ID地址
        0x10,                                                    // ID
        {0x41, 0x42},                                            // 配置寄存器地址
        2,                                                       // 配置数
        {0x24, 0xc0},                                            // 配置值
        false,                                                   // 需要状态检查
        0,                                                       // 状态检查寄存器地址
        0,                                                       // 状态检查掩码
        0,                                                       // 状态值
        6,                                                       // 单次读取字节数
        0.003,                                                   // 灵敏度(Gauss/LSB)
        30,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
    // RM3100
    {
        HW_CFG_RM3100,                                           // device
        "RM3100",                                                // 名称
        0x20,                                                    // iic地址
        0x24,                                                    // 数据寄存器
        true,                                                    // MSB高位在前
        true,                                                    // 手动采样
        0x00,                                                    // 采样地址
        0x70,                                                    // 采样值
        0x35,                                                    // ID地址
        0x1b,                                                    // ID
        {4, 5, 6, 7, 8, 9},                                     // 配置寄存器地址
        6,                                                       // 配置数
        {0, 200, 0, 200, 0, 200},                                // 配置值
        true,                                                    // 需要状态检查
        0x34,                                                    // 状态检查寄存器地址
        0x80,                                                    // 状态检查掩码
        0x80,                                                    // 状态值
        9,                                                       // 单次读取字节数
        0.003,                                                   // 灵敏度(Gauss/LSB)
        30,                                                      // 采样ms
        I2C_SPEED_100k,                                          // IIC速度
    },
};
// clang-format on
const uint8_t I2C_Mag_Protocol_Count = sizeof(I2C_Mag_Protocols) / sizeof(I2C_MAG_PROTOCOL);

void HW_DecodeAxis(uint16_t value_low, int8_t axis[3])
{
    for (int i = 0; i < 3; i++)
    {
        uint8_t nibble = (value_low >> (i * 4)) & 0xF;
        bool neg = (nibble & 0x8) != 0;
        uint8_t sAxis = nibble & 0x7; // 1=X, 2=Y, 3=Z
        axis[i] = neg ? -(int8_t) sAxis : (int8_t) sAxis;
    }
}

void init_drv_Compass()
{
    init_drv_compass_InternalMag_I();
    init_drv_compass_ExtMag_I();
}
