#pragma once

#include "i2c_common.hpp"
#include "stm32h7xx.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include <stdint.h>

class I2C_CLASS
{
  public:
    bool init(uint8_t i2c_num);

    bool lock(double Sync_waitTime = -1);
    void unlock();

    I2C_Result write(const uint16_t address, const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size, I2C_SPEED speed = I2C_SPEED_100k, double TIMEOUT = -1);
    I2C_Result read(const uint16_t address, uint8_t *rx_data, const uint8_t rx_size, const uint8_t rx_buffer_size, I2C_SPEED speed = I2C_SPEED_100k, double TIMEOUT = -1);
    I2C_Result writeNread(const uint16_t address, const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size, uint8_t *rx_data, const uint8_t rx_size, const uint8_t rx_buffer_size,
                          I2C_SPEED speed = I2C_SPEED_100k, double TIMEOUT = -1);

    void handleEvIRQ();
    void handleErIRQ();

  private:
    I2C_TypeDef *i2c;
    DMA_Stream_TypeDef *dmaTxStream;
    DMA_Stream_TypeDef *dmaRxStream;
    volatile uint32_t *dmaTxIfcr;
    volatile uint32_t *dmaRxIfcr;
    uint32_t dmaTxIfcrVal;
    uint32_t dmaRxIfcrVal;

    SemaphoreHandle_t mutex;
    EventGroupHandle_t events;
    uint16_t rmTxLength;
    uint16_t rmRxLength;
    uint8_t i2cNum;
    bool initialized;
    bool useBDMA; // I2C4 专用: 使用 BDMA 而非 DMA1/DMA2

    uint8_t *noncacheBuf;
    uint32_t timingR[3]; // 预计算各速率 TIMINGR 值，按 I2C_SPEED 枚举索引
};

// 根据硬件配置文件自动初始化所有I2C端口
void init_drv_i2c();

// 获取指定编号的I2C实例 (1=I2C1, 2=I2C2, 3=I2C3, 4=I2C4)
// 返回0表示该端口未初始化或不存在
I2C_CLASS *get_i2c_instance(uint8_t i2c_num);

// 获取内部I2C功能使用的I2C编号 (由硬件配置文件[i2cInt]指定)
// 返回0表示未配置
uint8_t get_i2cInt_num();

// 获取内部I2C是否同时注册为外部总线 (由硬件配置文件[i2cInt] ext=true 指定)
bool get_i2cInt_ext();
