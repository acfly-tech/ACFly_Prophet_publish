#include "drv_spi.hpp"
#include "Basic.hpp"
#include "HardWareConfig.hpp"
#include "dma.hpp"

#include "AC_Math.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

#include "stm32h743xx.h"
#include <limits>

#define MAX_SPI_INDEX 7 // [0]未使用, [1]~[6]
#define SPI_TIMEOUT (1.0 * configTICK_RATE_HZ)
#define SPI_DEFAULT_FREQ 5e6

// --- 静态实例 ---
static SPI_CLASS spi_instances[MAX_SPI_INDEX];
static bool spi_inited[MAX_SPI_INDEX] = {};

// --- DMA 缓冲区 (每个SPI实例一个) ---
Static_AXIDMABuf uint8_t SpiDmaBufs[MAX_SPI_INDEX][256];

// ================================================================
// SPI 硬件参数查找表 (STM32H743)
// ================================================================

// SPI_TypeDef* 查找表
static SPI_TypeDef *const spiTable[MAX_SPI_INDEX] = {0, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};

// SPI RCC 使能表
static const HW_RccEnableInfo spiRccTable[MAX_SPI_INDEX] = {
    {0, 0},
    {&RCC->APB2ENR, (1u << 12)},  // SPI1
    {&RCC->APB1LENR, (1u << 14)}, // SPI2
    {&RCC->APB1LENR, (1u << 15)}, // SPI3
    {&RCC->APB2ENR, (1u << 13)},  // SPI4
    {&RCC->APB2ENR, (1u << 20)},  // SPI5
    {&RCC->APB4ENR, (1u << 5)},   // SPI6
};

// SPI 内核时钟频率表 (SPI1/4/5=APB2, SPI2/3=APB1, SPI6=APB4)
static const uint32_t spiClkTable[MAX_SPI_INDEX] = {0, APB2CLK, APB1CLK, APB1CLK, APB2CLK, APB2CLK, APB4CLK};

// DMAMUX TX 请求号
static const uint8_t spiDmamuxTxTable[MAX_SPI_INDEX] = {0, 38, 40, 42, 84, 86, 88};

// DMAMUX RX 请求号
static const uint8_t spiDmamuxRxTable[MAX_SPI_INDEX] = {0, 37, 39, 41, 83, 85, 87};

// HW_Device 映射 (spi_num -> HW_DEV_SPIx)
static const uint16_t spiDevTable[MAX_SPI_INDEX] = {0, HW_DEV_SPI1, HW_DEV_SPI2, HW_DEV_SPI3, HW_DEV_SPI4, HW_DEV_SPI5, HW_DEV_SPI6};

// ---- DMA 回调包装 ----
template <int N> static void dmaRxCallback(uint8_t isr, BaseType_t *HigherPriorityTaskWoken)
{
    spi_instances[N].handleRxDmaIRQ(isr, HigherPriorityTaskWoken);
}

// ================================================================
// SPI_CLASS 实现
// ================================================================

bool SPI_CLASS::init(uint8_t num)
{
    if (num < 1 || num > 6)
        return false;

    spiNum = num;
    trCache = SpiDmaBufs[num];

    SPI_TypeDef *s = spiTable[num];
    if (!s)
        return false;
    spi = s;

    uint16_t hw_dev = spiDevTable[num];

    // --- 从 InternalStorage 读取硬件配置 ---
    HW_ConfigEntry entry;

    // SCK引脚 (PIN1) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN1, &entry))
        return false;
    uint16_t sck_port = entry.value_high;
    uint16_t sck_pin = entry.value_low;

    // MISO引脚 (PIN2) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN2, &entry))
        return false;
    uint16_t miso_port = entry.value_high;
    uint16_t miso_pin = entry.value_low;

    // MOSI引脚 (PIN3) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN3, &entry))
        return false;
    uint16_t mosi_port = entry.value_high;
    uint16_t mosi_pin = entry.value_low;

    // AF (可选, 默认5)
    uint16_t sck_af = 5, miso_af = 5, mosi_af = 5;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF1, &entry))
        sck_af = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF2, &entry))
        miso_af = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF3, &entry))
        mosi_af = entry.value_low;

    // TX DMA (DMA1)
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_DMA1, &entry))
    {
        tx_dma_num = entry.value_high;
        tx_dma_chn = entry.value_low;
    }
    else
        return false;

    // RX DMA (DMA2)
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_DMA2, &entry))
    {
        rx_dma_num = entry.value_high;
        rx_dma_chn = entry.value_low;
    }
    else
        return false;

    // 端口号校验
    if (sck_port > 7 || miso_port > 7 || mosi_port > 7)
        return false;
    if (tx_dma_num < 1 || tx_dma_num > 2 || tx_dma_chn > 7)
        return false;
    if (rx_dma_num < 1 || rx_dma_num > 2 || rx_dma_chn > 7)
        return false;

    // --- SPI 时钟使能 ---
    if (spiRccTable[num].reg)
        *spiRccTable[num].reg |= spiRccTable[num].bit;
    vTaskDelay(1);

    // --- SPI 寄存器配置 ---
    uint32_t spi_clock = spiClkTable[num];
    uint32_t freq_splt = round(log2((double) spi_clock / SPI_DEFAULT_FREQ)) - 1;
    freq_splt &= 0x7f;
    s->CR1 = (1 << 12);
    s->CFG1 = (freq_splt << 28) | (1 << 15) | (1 << 14) | (7 << 0);
    s->CFG2 = (1 << 31) | (1 << 30) | (1 << 26) | (1 << 25) | (1 << 24) | (1 << 22);
    s->IFCR = (1 << 9);
    s->CR1 = (1 << 12);

    // --- DMA 初始化 ---
    DMA_TypeDef *txDma = tx_dma_num == 1 ? DMA1 : DMA2;
    DMA_TypeDef *rxDma = rx_dma_num == 1 ? DMA1 : DMA2;
    DMA_Stream_TypeDef *const *txStreams = tx_dma_num == 1 ? HW_DMA1Streams : HW_DMA2Streams;
    DMA_Stream_TypeDef *const *rxStreams = rx_dma_num == 1 ? HW_DMA1Streams : HW_DMA2Streams;
    DMA_Stream_TypeDef *txStr = txStreams[tx_dma_chn];
    DMA_Stream_TypeDef *rxStr = rxStreams[rx_dma_chn];
    txDmaStream = txStr;
    rxDmaStream = rxStr;
    rxDmaIrqn = HW_DmaIrqnTable[rx_dma_num - 1][rx_dma_chn];

    DMAMUX_Channel_TypeDef *txMuxCh = (DMAMUX_Channel_TypeDef *) (DMAMUX1_BASE + 0x0004UL * ((tx_dma_num - 1) * 8 + tx_dma_chn));
    DMAMUX_Channel_TypeDef *rxMuxCh = (DMAMUX_Channel_TypeDef *) (DMAMUX1_BASE + 0x0004UL * ((rx_dma_num - 1) * 8 + rx_dma_chn));

    // SPI TX DMA
    txStr->PAR = (uint32_t) &s->TXDR;
    txStr->NDTR = 2;
    txMuxCh->CCR = (spiDmamuxTxTable[num] << 0);
    txStr->CR = (3 << 16) | (0 << 13) | (1 << 10) | (0 << 9) | (0b01 << 6);
    txStr->FCR = (1 << 2) | (3 << 0);

    // SPI RX DMA
    rxStr->PAR = (uint32_t) &s->RXDR;
    rxStr->NDTR = 2;
    rxMuxCh->CCR = (spiDmamuxRxTable[num] << 0);
    rxStr->CR = (3 << 16) | (0 << 13) | (1 << 10) | (0 << 9) | (0b00 << 6) | (1 << 4) | (1 << 2);
    rxStr->FCR = (1 << 2) | (3 << 0);
    NVIC_SetPriority((IRQn_Type) rxDmaIrqn, IRQPrioriy_SensorSpi);

    // 注册DMA RX中断回调
    DMA_CallbackFunc dmaCb = 0;
    switch (num)
    {
        case 1:
            dmaCb = dmaRxCallback<1>;
            break;
        case 2:
            dmaCb = dmaRxCallback<2>;
            break;
        case 3:
            dmaCb = dmaRxCallback<3>;
            break;
        case 4:
            dmaCb = dmaRxCallback<4>;
            break;
        case 5:
            dmaCb = dmaRxCallback<5>;
            break;
        case 6:
            dmaCb = dmaRxCallback<6>;
            break;
    }
    DMA_CallBackFunc_Register(rx_dma_num, rx_dma_chn, dmaCb);

    // --- GPIO 初始化 ---
    GPIO_TypeDef *sckGpio = HW_GpioTable[sck_port];
    GPIO_TypeDef *misoGpio = HW_GpioTable[miso_port];
    GPIO_TypeDef *mosiGpio = HW_GpioTable[mosi_port];

    // 保存 SCK 引脚信息，供 sendSckWakeupPulses 使用
    this->sckGpio = sckGpio;
    this->sckPin = sck_pin;
    this->sckAf = sck_af;

    RCC->AHB4ENR |= (1 << sck_port) | (1 << miso_port) | (1 << mosi_port);
    vTaskDelay(1);

    // Moder = 复用功能
    set_register(sckGpio->MODER, 0b10, 2 * sck_pin, 2);
    set_register(misoGpio->MODER, 0b10, 2 * miso_pin, 2);
    set_register(mosiGpio->MODER, 0b10, 2 * mosi_pin, 2);

    // OTYPE: SCK+MOSI推挽, MISO开漏
    set_register(sckGpio->OTYPER, 0, sck_pin, 1);
    set_register(misoGpio->OTYPER, 1, miso_pin, 1);
    set_register(mosiGpio->OTYPER, 0, mosi_pin, 1);

    // PUPDR: SCK+MOSI无, MISO上拉
    set_register(sckGpio->PUPDR, 0, 2 * sck_pin, 2);
    set_register(misoGpio->PUPDR, 1, 2 * miso_pin, 2);
    set_register(mosiGpio->PUPDR, 0, 2 * mosi_pin, 2);

    // SPEED: medium
    set_register(sckGpio->OSPEEDR, 1, 2 * sck_pin, 2);
    set_register(misoGpio->OSPEEDR, 1, 2 * miso_pin, 2);
    set_register(mosiGpio->OSPEEDR, 1, 2 * mosi_pin, 2);

    // AF
    uint8_t afIdx, afOff;
    afIdx = sck_pin >= 8 ? 1 : 0;
    afOff = (sck_pin >= 8 ? sck_pin - 8 : sck_pin) * 4;
    set_register(sckGpio->AFR[afIdx], sck_af, afOff, 4);
    afIdx = miso_pin >= 8 ? 1 : 0;
    afOff = (miso_pin >= 8 ? miso_pin - 8 : miso_pin) * 4;
    set_register(misoGpio->AFR[afIdx], miso_af, afOff, 4);
    afIdx = mosi_pin >= 8 ? 1 : 0;
    afOff = (mosi_pin >= 8 ? mosi_pin - 8 : mosi_pin) * 4;
    set_register(mosiGpio->AFR[afIdx], mosi_af, afOff, 4);

    // --- 创建 FreeRTOS 对象 ---
    mutex = xSemaphoreCreateRecursiveMutex();
    events = xEventGroupCreate();

    // 向某些需要时钟馈入唤醒的传感器(如 SPL06)发送若干 SCK 周期脉冲
    sendSckWakeupPulses(32, 50000.0);

    return true;
}

// ================================================================
// SCK 唤醒脉冲
// ================================================================
void SPI_CLASS::sendSckWakeupPulses(uint8_t count, double freqHz)
{
    if (!sckGpio || count == 0 || freqHz <= 0)
        return;

    GPIO_TypeDef *gpio = (GPIO_TypeDef *) sckGpio;
    uint16_t pin = sckPin;
    double halfPeriod = 0.5 / freqHz; // 秒

    // 把 SCK 切换为 GPIO 推挽输出模式
    set_register(gpio->MODER, 0b01, 2 * pin, 2); // 01 = 通用输出
    set_register(gpio->OTYPER, 0, pin, 1);       // 推挽
    set_register(gpio->OSPEEDR, 1, 2 * pin, 2);  // 中速

    uint32_t cyclesPerHalfPeriod = (uint32_t) (halfPeriod * (double) SystemCoreClock);

    for (uint8_t i = 0; i < count; ++i)
    {
        // SCK 高
        gpio->BSRR = 1 << pin;
        uint32_t start = DWT->CYCCNT;
        delay(0.0001); // 确保至少延时 100us

        // SCK 低
        gpio->BSRR = 1 << (pin + 16);
        start = DWT->CYCCNT;
        delay(0.0001); // 确保至少延时 100us
    }

    // SCK 保持低
    gpio->BSRR = 1 << (pin + 16);

    // 恢复 SCK 为 SPI 复用功能
    set_register(gpio->MODER, 0b10, 2 * pin, 2); // 10 = 复用功能
    set_register(gpio->OSPEEDR, 1, 2 * pin, 2);  // 中速
    uint8_t afIdx = pin >= 8 ? 1 : 0;
    uint8_t afOff = (pin >= 8 ? pin - 8 : pin) * 4;
    set_register(gpio->AFR[afIdx], sckAf, afOff, 4);
}

// ================================================================
// 公开接口方法
// ================================================================

bool SPI_CLASS::lock(double timeout)
{
    uint32_t ticks;
    if (timeout >= 0)
        ticks = timeout * configTICK_RATE_HZ;
    else
        ticks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) mutex, ticks) == pdTRUE)
        return true;
    return false;
}

void SPI_CLASS::unlock()
{
    xSemaphoreGiveRecursive((SemaphoreHandle_t) mutex);
}

SPI_Result SPI_CLASS::write(const uint8_t *tx_data, uint8_t tx_size, uint8_t tx_buffer_size, double timeout)
{
    if (tx_data == 0 || tx_size == 0)
        return SPI_Result_InvalidArgs;

    if (!lock(timeout))
        return SPI_Result_timeout;

    SPI_TypeDef *s = (SPI_TypeDef *) spi;
    DMA_Stream_TypeDef *txStr = (DMA_Stream_TypeDef *) txDmaStream;
    DMA_Stream_TypeDef *rxStr = (DMA_Stream_TypeDef *) rxDmaStream;

    // 关闭DMA通道
    txStr->CR &= ~(1 << 0);
    rxStr->CR &= ~(1 << 0);
    xEventGroupClearBits((EventGroupHandle_t) events, 0xff);

    // 准备发送数据缓冲区
    const uint8_t *data_buff;
    uint32_t aligned_end = CACHELINE_ALIGN_UP((uint32_t) tx_data + tx_size);
    uint32_t aligned_size = aligned_end - (uint32_t) tx_data;
#ifdef DCACHE_SIZE
    if (isDMABuf((uint32_t) tx_data))
#endif
    {
        data_buff = tx_data;
    }
#ifdef DCACHE_SIZE
    else if ((uint32_t) tx_data % DCACHE_SIZE == 0 && aligned_size <= tx_buffer_size)
    {
        data_buff = tx_data;
        SCB_CleanDCache_by_Addr((uint32_t *) tx_data, aligned_size);
    }
    else
    {
        data_buff = trCache;
    }
#endif

    // 关闭SPI外设
    s->CR1 &= ~(1 << 0);
    s->CFG1 &= ~((1 << 15) | (1 << 14));
    // 清空DMA状态
    clear_DMA_flags(tx_dma_num, tx_dma_chn);
    clear_DMA_flags(rx_dma_num, rx_dma_chn);
    // 设置DMA地址
    txStr->M0AR = (uint32_t) data_buff;
    rxStr->M0AR = (uint32_t) trCache;
    // 清空SPI状态
    s->IFCR = (1 << 9) | (1 << 4);
    // 设置传输数量
    txStr->NDTR = rxStr->NDTR = s->CR2 = tx_size;
    // 使能RX DMA，打开RX DMA请求
    rxStr->CR |= (1 << 0);
    s->CFG1 |= (1 << 14);
    // 使能TX DMA，打开TX DMA请求
    txStr->CR |= (1 << 0);
    s->CFG1 |= (1 << 15);
    // 打开SPI开始传输
    s->CR1 |= (1 << 0);
    s->CR1 |= (1 << 9);
    // 开启中断
    NVIC_EnableIRQ((IRQn_Type) rxDmaIrqn);

    // 等待传输完成
    uint32_t r_status = xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0), pdTRUE, pdFALSE, SPI_TIMEOUT);
    NVIC_DisableIRQ((IRQn_Type) rxDmaIrqn);

    unlock();
    if (r_status & (1 << 4))
        return SPI_Result_Success;
    return SPI_Result_Fail;
}

SPI_Result SPI_CLASS::writeNread(const uint8_t *tx_data, uint8_t tx_size, uint8_t tx_buffer_size, uint8_t *rx_data, uint8_t rx_buffer_size, double timeout)
{
    if (tx_data == 0 || tx_size == 0 || tx_buffer_size < tx_size || rx_data == 0 || rx_buffer_size < tx_size)
        return SPI_Result_InvalidArgs;

    if (!lock(timeout))
        return SPI_Result_timeout;

    SPI_TypeDef *s = (SPI_TypeDef *) spi;
    DMA_Stream_TypeDef *txStr = (DMA_Stream_TypeDef *) txDmaStream;
    DMA_Stream_TypeDef *rxStr = (DMA_Stream_TypeDef *) rxDmaStream;

    // 关闭DMA通道
    txStr->CR &= ~(1 << 0);
    rxStr->CR &= ~(1 << 0);
    xEventGroupClearBits((EventGroupHandle_t) events, 0xff);

    // 准备TX数据缓冲区
    const uint8_t *tx_data_buff;
    uint32_t aligned_end = CACHELINE_ALIGN_UP((uint32_t) tx_data + tx_size);
    uint32_t aligned_size = aligned_end - (uint32_t) tx_data;
#ifdef DCACHE_SIZE
    if (isDMABuf((uint32_t) tx_data))
#endif
    {
        tx_data_buff = tx_data;
        SCB_CleanDCache_by_Addr((uint32_t *) tx_data, aligned_size);
    }
#ifdef DCACHE_SIZE
    else if ((uint32_t) tx_data % DCACHE_SIZE == 0 && aligned_size <= tx_buffer_size)
    {
        tx_data_buff = tx_data;
        SCB_CleanDCache_by_Addr((uint32_t *) tx_data, aligned_size);
    }
    else
    {
        tx_data_buff = trCache;
    }
#endif

    // 准备RX数据缓冲区
    const uint8_t *rx_data_buff;
    aligned_end = CACHELINE_ALIGN_UP((uint32_t) rx_data + tx_size);
    aligned_size = aligned_end - (uint32_t) rx_data;
#ifdef DCACHE_SIZE
    if (isDMABuf((uint32_t) rx_data))
#endif
    {
        rx_data_buff = rx_data;
        SCB_InvalidateDCache_by_Addr((uint32_t *) rx_data, aligned_size);
    }
#ifdef DCACHE_SIZE
    else if ((uint32_t) rx_data % DCACHE_SIZE == 0 && aligned_size <= rx_buffer_size)
    {
        rx_data_buff = rx_data;
        SCB_InvalidateDCache_by_Addr((uint32_t *) rx_data, aligned_size);
    }
    else
    {
        rx_data_buff = trCache;
    }
#endif

    // 关闭SPI外设
    s->CR1 &= ~(1 << 0);
    s->CFG1 &= ~((1 << 15) | (1 << 14));
    // 清空DMA状态
    clear_DMA_flags(tx_dma_num, tx_dma_chn);
    clear_DMA_flags(rx_dma_num, rx_dma_chn);
    // 设置DMA地址
    txStr->M0AR = (uint32_t) tx_data_buff;
    rxStr->M0AR = (uint32_t) rx_data_buff;
    // 清空SPI状态
    s->IFCR = (1 << 9) | (1 << 4);
    // 设置传输数量
    txStr->NDTR = rxStr->NDTR = s->CR2 = tx_size;
    // 使能RX DMA，打开RX DMA请求
    rxStr->CR |= (1 << 0);
    s->CFG1 |= (1 << 14);
    // 使能TX DMA，打开TX DMA请求
    txStr->CR |= (1 << 0);
    s->CFG1 |= (1 << 15);
    // 打开SPI开始传输
    s->CR1 |= (1 << 0);
    s->CR1 |= (1 << 9);
    // 开启中断
    NVIC_EnableIRQ((IRQn_Type) rxDmaIrqn);

    // 等待传输完成
    uint32_t r_status = xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0), pdTRUE, pdFALSE, SPI_TIMEOUT);
    NVIC_DisableIRQ((IRQn_Type) rxDmaIrqn);

    // 如数据在noncache缓冲区则复制回来
    if (r_status & (1 << 4))
    {
        if (rx_data_buff != rx_data)
            memcpy(rx_data, rx_data_buff, tx_size);
    }

    unlock();
    if (r_status & (1 << 4))
        return SPI_Result_Success;
    return SPI_Result_Fail;
}

void SPI_CLASS::handleRxDmaIRQ(uint8_t isr, BaseType_t *HigherPriorityTaskWoken)
{
    // 关闭DMA中断
    NVIC_DisableIRQ((IRQn_Type) rxDmaIrqn);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR((EventGroupHandle_t) events, isr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ================================================================
// 初始化接口
// ================================================================

void init_drv_spi()
{
    for (uint8_t num = 1; num <= 6; num++)
    {
        HW_ConfigEntry entry;
        if (!HW_ReadConfigEntry(spiDevTable[num], HW_CFG_PIN1, &entry))
            continue;
        if (!spi_instances[num].init(num))
            continue;
        spi_inited[num] = true;
    }
}

SPI_CLASS *get_spi_instance(uint8_t spi_num)
{
    if (spi_num >= 1 && spi_num < MAX_SPI_INDEX && spi_inited[spi_num])
        return &spi_instances[spi_num];
    return 0;
}
