#pragma once

#include <stdint.h>

enum CRC_POLY_SIZE
{
    CRC_POLY_SIZE_7bit,
    CRC_POLY_SIZE_8bit,
    CRC_POLY_SIZE_16bit,
    CRC_POLY_SIZE_32bit,
};

typedef struct
{
    CRC_POLY_SIZE Size; // 多项式宽度
    uint32_t Poly;      // 多项式
    uint32_t Init;      // 初始值
    uint32_t XOROUT;    // 结果异或值
    bool isREFIN;       // 输入数据反转
    bool isREFOUT;      // 输出数据反转
} CRC_Cfg;

/*CRC多项式配置*/
// 默认的CRC-32校验-比如以太网
extern const CRC_Cfg cg_CRC_CONFIG_CRC32;

// mavlink CRC-32
extern const CRC_Cfg cg_CRC_CONFIG_CRC32_MAVLINK;

// CRC-32/MPEG-2校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC32_MPEG2;

// CRC-16/XMODEM校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC16_XMODEM;

// CRC-16/CCITT校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC16_CCITT;

// CRC-16/MODBUS校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC16_MODBUS;

// CRC-16/USB校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC16_USB;

// CRC-16/IBM校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC16_IBM;

// CRC-8
extern const CRC_Cfg cg_CRC_CONFIG_CRC8;

// CRC-8/MAXIM校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC8_MAXIM;

// CRC-8/ROHC校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC8_ROHC;

// CRC-8/ITU校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC8_ITU;

// CRC-7/MMC校验
extern const CRC_Cfg cg_CRC_CONFIG_CRC7_MMC;
/*CRC多项式配置*/

/*CRC转换(一次转换完)
        注意：缓冲区必须为非CACHE区域或长度为32整数倍且32字节对齐的Aligned_DMABuf
        cfg：crc配置
        data：本次输入的crc数据
        length：crc数据长度
        res：返回的crc校验结果
        buffer_size：data所指缓冲区的实际大小(字节数)，用于校验SafeCleanDCache时cache行对齐所需空间是否足够。
                     当length != buffer_size且(data, length)跨越cache行时，需buffer_size >= alignUp(data+length)-data 才能原地清洗。
                     传入0表示无法原地清洗(强制走内部拷贝缓冲区)。
        crcLastRs：上次crc结果,仅当crcConinue=true时有效
        crcConinue：true时从上次结果继续校验
        返回：true成功 false失败
*/
bool CRC_Calc(CRC_Cfg cfg, const uint8_t *data, uint16_t length, uint32_t *res, uint32_t buffer_size, uint32_t crcLastRs = 0, bool crcConinue = false, double waitTime = -1);

void init_drv_CRC();