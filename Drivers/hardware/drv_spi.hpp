#pragma once

#include "Basic.hpp"
#include "spi_common.hpp"
#include <stdint.h>

class SPI_CLASS
{
  public:
    bool init(uint8_t spi_num);

    SPI_Result write(const uint8_t *tx_data, uint8_t tx_size, uint8_t tx_buffer_size, double timeout = -1);
    SPI_Result writeNread(const uint8_t *tx_data, uint8_t tx_size, uint8_t tx_buffer_size, uint8_t *rx_data, uint8_t rx_buffer_size, double timeout = -1);
    bool lock(double timeout);
    void unlock();

    // DMA RX 完成回调 (由 DMA 回调系统调用)
    void handleRxDmaIRQ(uint8_t isr, BaseType_t *HigherPriorityTaskWoken);

  private:
    void *spi;         // SPI_TypeDef*
    void *txDmaStream; // DMA_Stream_TypeDef*
    uint16_t tx_dma_num, tx_dma_chn;
    void *rxDmaStream; // DMA_Stream_TypeDef*
    uint16_t rx_dma_num, rx_dma_chn;

    uint32_t rxDmaIrqn;
    uint8_t spiNum;

    void *mutex;      // SemaphoreHandle_t
    void *events;     // EventGroupHandle_t
    uint8_t *trCache; // DMA 缓冲区指针

    // SCK GPIO 信息 (用于初始化后向某些传感器发送唤醒脉冲)
    void *sckGpio; // GPIO_TypeDef*
    uint16_t sckPin;
    uint16_t sckAf; // 复用功能号 (恢复时需要)

    // 在 SCK 线上发出若干个时钟周期 (用于唤醒 SPL06 等需要时钟馈入的芯片)
    // 临时把 SCK 切为 GPIO 推挽输出，发送完后恢复为 SPI 复用模式
    void sendSckWakeupPulses(uint8_t count, double freqHz);
};

// 支持的SPI编号: 1~6 (SPI1~SPI6)

// 根据HardWare配置初始化所有SPI设备
void init_drv_spi();

// 获取指定SPI号的SPI_CLASS实例
// 未初始化的返回nullptr
SPI_CLASS *get_spi_instance(uint8_t spi_num);
