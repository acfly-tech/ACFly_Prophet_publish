#include "drv_i2c.hpp"
#include "Basic.hpp"
#include "HardWareConfig.hpp"
#include "dma.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include <string.h>

// IIC传输配置
#define IIC_AutoEND (1 << 25)
#define IIC_Reload (1 << 24)
#define IIC_NBytes(x) (x << 16)
#define IIC_NACK (1 << 15)
#define IIC_STOP (1 << 14)
#define IIC_START (1 << 13)
#define IIC_HEAD10R (1 << 12)
#define IIC_ADD10 (1 << 11)
#define IIC_READ (1 << 10)
#define IIC_ADDR7(x) (x << 1)
// IIC状态
#define IIC_ARLO (1 << 9)
#define IIC_BERR (1 << 8)
#define IIC_TCR (1 << 7)
#define IIC_TC (1 << 6)
#define IIC_STOPF (1 << 5)
#define IIC_NACKF (1 << 4)
#define IIC_TIMEOUT 1.0 * configTICK_RATE_HZ

#include "TimeBase.h"

/**
 * 计算 I2C TIMINGR 值（支持不同 kernel clock）
 * @param f_i2cclk   I2C kernel clock 频率 (Hz)，必须准确！（例如 100000000）
 * @param speed      目标速度
 * @return           TIMINGR 寄存器值，失败返回 0
 */
static uint32_t calc_i2c_timing(uint32_t f_i2cclk, I2C_SPEED speed)
{
    // ====================== 1. 特殊处理你已验证稳定的 100MHz 配置 ======================
    if (f_i2cclk == 100000000UL)
    {
        switch (speed)
        {
            case I2C_SPEED_400k:
                return 0x9034B6; // 你原来的 400kHz（SCLH=52）
            case I2C_SPEED_50k:
                return 0xC0901E71;
            default:
                return 0x10C0ECFF; // 100kHz（SCLH=236，最稳定）
        }
    }

    // ====================== 2. 其他频率使用动态保守计算 ======================
    uint32_t target_freq, scll_min_ns, sclh_min_ns, sdadel_min_ns, scldel_min_ns;

    switch (speed)
    {
        case I2C_SPEED_400k:
            target_freq = 400000;
            scll_min_ns = 650; // 保守
            sclh_min_ns = 340; // 比你原来的 52 ticks 更接近实际稳定值
            sdadel_min_ns = 100;
            scldel_min_ns = 200;
            break;

        case I2C_SPEED_50k:
            target_freq = 50000;
            scll_min_ns = 5200;
            sclh_min_ns = 4300;
            sdadel_min_ns = 350;
            scldel_min_ns = 350;
            break;

        default: // 100kHz
            target_freq = 100000;
            scll_min_ns = 4900;
            sclh_min_ns = 4200;
            sdadel_min_ns = 280;
            scldel_min_ns = 320;
            break;
    }

    for (uint32_t presc = 0; presc <= 15; ++presc)
    {
        uint32_t f_presc = f_i2cclk / (presc + 1);

        uint32_t scll_min = ((uint64_t) scll_min_ns * f_presc + 999999999ULL) / 1000000000ULL;
        uint32_t sclh_min = ((uint64_t) sclh_min_ns * f_presc + 999999999ULL) / 1000000000ULL;
        uint32_t sdadel = ((uint64_t) sdadel_min_ns * f_presc + 999999999ULL) / 1000000000ULL;
        uint32_t scldel = ((uint64_t) scldel_min_ns * f_presc + 999999999ULL) / 1000000000ULL;

        if (scll_min < 2)
            scll_min = 2;
        if (sclh_min < 2)
            sclh_min = 2;

        uint32_t period = f_presc / target_freq;

        if (period < scll_min + sclh_min + 8)
            continue; // 留更多裕量

        uint32_t sclh = sclh_min;
        uint32_t scll = period - sclh - 6; // 低电平留足够时间

        if (scll < scll_min || scll > 255 || sclh > 255)
            continue;

        if (sdadel > 15)
            sdadel = 15;
        if (scldel > 15)
            scldel = 15;

        return ((presc << 28) | (scldel << 20) | (sdadel << 16) | (sclh << 8) | scll);
    }

    // 兜底：返回 100kHz 的安全值
    return 0x10C0ECFF;
}

#define MAX_I2C_INDEX 5 // index 0 unused, 1=I2C1, 2=I2C2, 3=I2C3, 4=I2C4

// I2C外设查找表
static I2C_TypeDef *const i2cTable[MAX_I2C_INDEX] = {0, I2C1, I2C2, I2C3, I2C4};
static const IRQn_Type i2cEvIrqnTable[MAX_I2C_INDEX] = {(IRQn_Type) 0, I2C1_EV_IRQn, I2C2_EV_IRQn, I2C3_EV_IRQn, I2C4_EV_IRQn};
static const IRQn_Type i2cErIrqnTable[MAX_I2C_INDEX] = {(IRQn_Type) 0, I2C1_ER_IRQn, I2C2_ER_IRQn, I2C3_ER_IRQn, I2C4_ER_IRQn};
static const uint8_t dmamuxTxTable[MAX_I2C_INDEX] = {0, 34, 36, 74, 0}; // I2C4 走 BDMA/DMAMUX2，init() 中单独处理，此处占位
static const uint8_t dmamuxRxTable[MAX_I2C_INDEX] = {0, 33, 35, 73, 0}; // I2C4 走 BDMA/DMAMUX2，init() 中单独处理，此处占位
static const HW_RccEnableInfo rccTable[MAX_I2C_INDEX] = {
    {0, 0},
    {&RCC->APB1LENR, (1u << 21)}, // I2C1
    {&RCC->APB1LENR, (1u << 22)}, // I2C2
    {&RCC->APB1LENR, (1u << 23)}, // I2C3
    {&RCC->APB4ENR, (1u << 7)},   // I2C4
};
static const uint16_t hwDevTable[MAX_I2C_INDEX] = {0, HW_DEV_I2C1, HW_DEV_I2C2, HW_DEV_I2C3, HW_DEV_I2C4};

// 实例和状态
static I2C_CLASS i2c_instances[MAX_I2C_INDEX] = {};
static bool i2c_inited[MAX_I2C_INDEX] = {};
static uint8_t i2cInt_num = 0;
static bool i2cInt_ext = false;

// 每个实例独立的非cache缓冲区
Static_AXIDMABuf uint8_t noncache_buf1[256];
Static_AXIDMABuf uint8_t noncache_buf2[256];
Static_AXIDMABuf uint8_t noncache_buf3[256];
Static_SRAM4Buf uint8_t noncache_buf4[256]; // BDMA (I2C4) 只能访问 SRAM4
static uint8_t *const noncacheBufTable[MAX_I2C_INDEX] = {0, noncache_buf1, noncache_buf2, noncache_buf3, noncache_buf4};

/*======================== I2C_CLASS 实现 ========================*/

bool I2C_CLASS::init(uint8_t num)
{
    if (num < 1 || num >= MAX_I2C_INDEX)
        return false;

    uint16_t hw_dev = hwDevTable[num];
    HW_ConfigEntry entry;

    // SCL引脚 (PIN1) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN1, &entry))
        return false;
    uint16_t scl_port = entry.value_high;
    uint16_t scl_pin = entry.value_low;

    // SDA引脚 (PIN2) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN2, &entry))
        return false;
    uint16_t sda_port = entry.value_high;
    uint16_t sda_pin = entry.value_low;

    // AF (必须)
    uint16_t scl_af = 4;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF1, &entry))
        scl_af = entry.value_low;
    uint16_t sda_af = 4;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF2, &entry))
        sda_af = entry.value_low;

    // TX DMA (必须)
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_DMA1, &entry))
        return false;
    uint16_t tx_dma_num = entry.value_high;
    uint16_t tx_stream_num = entry.value_low;

    // RX DMA (必须)
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_DMA2, &entry))
        return false;
    uint16_t rx_dma_num = entry.value_high;
    uint16_t rx_stream_num = entry.value_low;

    // OTYPE (可选, 默认开漏)
    uint16_t scl_otype = HW_OTYPE_OPEN_DRAIN;
    uint16_t sda_otype = HW_OTYPE_OPEN_DRAIN;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_OTYPE1, &entry))
        scl_otype = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_OTYPE2, &entry))
        sda_otype = entry.value_low;

    // PULL (可选, 默认无上拉，I2C外部需接上拉电阻)
    uint16_t scl_pull = HW_PULL_NONE;
    uint16_t sda_pull = HW_PULL_NONE;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_PULL1, &entry))
        scl_pull = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_PULL2, &entry))
        sda_pull = entry.value_low;

    // SPEED (可选, 默认very_high)
    uint16_t scl_speed = HW_SPEED_VERY_HIGH;
    uint16_t sda_speed = HW_SPEED_VERY_HIGH;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_SPEED1, &entry))
        scl_speed = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_SPEED2, &entry))
        sda_speed = entry.value_low;

    // DMA参数校验 (dma_num: 1=DMA1, 2=DMA2, 3=BDMA)
    if (tx_dma_num == 0 || tx_dma_num > 3 || tx_stream_num > 7)
        return false;
    if (rx_dma_num == 0 || rx_dma_num > 3 || rx_stream_num > 7)
        return false;

    // 保存成员变量
    i2cNum = num;
    i2c = i2cTable[num];
    noncacheBuf = noncacheBufTable[num];
    useBDMA = (tx_dma_num == 3); // I2C4 使用 BDMA

    if (useBDMA)
    {
        // BDMA 通道指针: BDMA_Channel0 + n (BDMA_Channel_TypeDef 每项 20 字节, 与 DMA_Stream_TypeDef 前 5 字项内存布局相同)
        dmaTxStream = (DMA_Stream_TypeDef *) (BDMA_Channel0 + tx_stream_num);
        dmaTxIfcr = (volatile uint32_t *) &BDMA->IFCR;
        dmaTxIfcrVal = 0xFu << (tx_stream_num * 4);

        dmaRxStream = (DMA_Stream_TypeDef *) (BDMA_Channel0 + rx_stream_num);
        dmaRxIfcr = (volatile uint32_t *) &BDMA->IFCR;
        dmaRxIfcrVal = 0xFu << (rx_stream_num * 4);
    }
    else
    {
        // TX DMA stream
        dmaTxStream = (tx_dma_num == 1) ? HW_DMA1Streams[tx_stream_num] : HW_DMA2Streams[tx_stream_num];
        if (tx_stream_num < 4)
        {
            DMA_TypeDef *txDmaBase = (tx_dma_num == 1) ? DMA1 : DMA2;
            dmaTxIfcr = &txDmaBase->LIFCR;
        }
        else
        {
            DMA_TypeDef *txDmaBase = (tx_dma_num == 1) ? DMA1 : DMA2;
            dmaTxIfcr = &txDmaBase->HIFCR;
        }
        dmaTxIfcrVal = HW_DmaClearBits[tx_stream_num & 3];

        // RX DMA stream
        dmaRxStream = (rx_dma_num == 1) ? HW_DMA1Streams[rx_stream_num] : HW_DMA2Streams[rx_stream_num];
        if (rx_stream_num < 4)
        {
            DMA_TypeDef *rxDmaBase = (rx_dma_num == 1) ? DMA1 : DMA2;
            dmaRxIfcr = &rxDmaBase->LIFCR;
        }
        else
        {
            DMA_TypeDef *rxDmaBase = (rx_dma_num == 1) ? DMA1 : DMA2;
            dmaRxIfcr = &rxDmaBase->HIFCR;
        }
        dmaRxIfcrVal = HW_DmaClearBits[rx_stream_num & 3];
    }

    // 创建FreeRTOS同步对象
    mutex = xSemaphoreCreateRecursiveMutex();
    events = xEventGroupCreate();

    /*配置GPIO*/
    GPIO_TypeDef *scl_gpio = HW_GpioTable[scl_port];
    GPIO_TypeDef *sda_gpio = HW_GpioTable[sda_port];
    RCC->AHB4ENR |= (1 << scl_port);
    if (sda_port != scl_port)
        RCC->AHB4ENR |= (1 << sda_port);
    os_delay(0.01);

    // SCL引脚
    if (scl_otype == HW_OTYPE_OPEN_DRAIN)
        scl_gpio->OTYPER |= (1 << scl_pin);
    else
        scl_gpio->OTYPER &= ~(1u << scl_pin);
    set_register(scl_gpio->PUPDR, scl_pull, scl_pin * 2, 2);
    set_register(scl_gpio->OSPEEDR, scl_speed, scl_pin * 2, 2);
    if (scl_pin < 8)
        set_register(scl_gpio->AFR[0], scl_af, scl_pin * 4, 4);
    else
        set_register(scl_gpio->AFR[1], scl_af, (scl_pin - 8) * 4, 4);
    set_register(scl_gpio->MODER, 0b10, scl_pin * 2, 2);

    // SDA引脚
    if (sda_otype == HW_OTYPE_OPEN_DRAIN)
        sda_gpio->OTYPER |= (1 << sda_pin);
    else
        sda_gpio->OTYPER &= ~(1u << sda_pin);
    set_register(sda_gpio->PUPDR, sda_pull, sda_pin * 2, 2);
    set_register(sda_gpio->OSPEEDR, sda_speed, sda_pin * 2, 2);
    if (sda_pin < 8)
        set_register(sda_gpio->AFR[0], sda_af, sda_pin * 4, 4);
    else
        set_register(sda_gpio->AFR[1], sda_af, (sda_pin - 8) * 4, 4);
    set_register(sda_gpio->MODER, 0b10, sda_pin * 2, 2);
    /*配置GPIO*/

    /*配置I2C外设*/
    // 打开I2C时钟
    *rccTable[num].reg |= rccTable[num].bit;
    os_delay(0.01);

    // 复位
    i2c->CR1 = 0;
    os_delay(0.005);

    // 根据所在时钟域预计算各速率 TIMINGR
    // I2C1-3 挂在 APB1 (D2域)；I2C4 挂在 APB4 (D3域，D3PCLK)
    uint32_t f_i2cclk = (num == 4) ? (uint32_t) D3PCLK : (uint32_t) APB1CLK;
    timingR[I2C_SPEED_50k] = calc_i2c_timing(f_i2cclk, I2C_SPEED_50k);
    timingR[I2C_SPEED_100k] = calc_i2c_timing(f_i2cclk, I2C_SPEED_100k);
    timingR[I2C_SPEED_400k] = calc_i2c_timing(f_i2cclk, I2C_SPEED_400k);
    i2c->TIMINGR = timingR[I2C_SPEED_100k];
    // 使能I2C
    i2c->CR1 = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 1;
    NVIC_SetPriority(i2cEvIrqnTable[num], 5);
    NVIC_EnableIRQ(i2cEvIrqnTable[num]);
    NVIC_SetPriority(i2cErIrqnTable[num], 5);
    NVIC_EnableIRQ(i2cErIrqnTable[num]);
    /*配置I2C外设*/

    /*DMA初始化*/
    if (useBDMA)
    {
        // 使能 BDMA 时钟 (AHB4ENR bit21)
        RCC->AHB4ENR |= (1u << 21);
        os_delay(0.005);

        // TX BDMA 通道
        // PAR/CPAR 在相同偏移 0x08, 可直接用 dmaTxStream->PAR
        dmaTxStream->PAR = (uint32_t) &i2c->TXDR;
        // DMAMUX2: I2C4_TX = 请求 ID 14 (BDMA_REQUEST_I2C4_TX)
        (DMAMUX2_Channel0 + tx_stream_num)->CCR = 14u;
        // BDMA CCR: DIR=1(内存→外设), MINC=1
        dmaTxStream->CR = (1u << 4) | (1u << 7);

        // RX BDMA 通道
        dmaRxStream->PAR = (uint32_t) &i2c->RXDR;
        // DMAMUX2: I2C4_RX = 请求 ID 13 (BDMA_REQUEST_I2C4_RX)
        (DMAMUX2_Channel0 + rx_stream_num)->CCR = 13u;
        // BDMA CCR: DIR=0(外设→内存), MINC=1
        dmaRxStream->CR = (1u << 7);
        // BDMA 无 FIFO，不写 FCR
    }
    else
    {
        // 打开 DMA1/DMA2 时钟
        if (tx_dma_num == 1 || rx_dma_num == 1)
            RCC->AHB1ENR |= (1 << 0);
        if (tx_dma_num == 2 || rx_dma_num == 2)
            RCC->AHB1ENR |= (1 << 1);
        os_delay(0.005);

        // TX DMA
        dmaTxStream->PAR = (uint32_t) &i2c->TXDR;
        uint8_t txMuxIdx = (tx_dma_num - 1) * 8 + tx_stream_num;
        (DMAMUX1_Channel0 + txMuxIdx)->CCR = (dmamuxTxTable[num] << 0);
        dmaTxStream->CR = (1 << 16) | (0 << 13) | (1 << 10) | (0 << 9) | (0b01 << 6);
        dmaTxStream->FCR = (1 << 2) | (3 << 0);

        // RX DMA
        dmaRxStream->PAR = (uint32_t) &i2c->RXDR;
        uint8_t rxMuxIdx = (rx_dma_num - 1) * 8 + rx_stream_num;
        (DMAMUX1_Channel0 + rxMuxIdx)->CCR = (dmamuxRxTable[num] << 0);
        dmaRxStream->CR = (1 << 16) | (0 << 13) | (1 << 10) | (0 << 9) | (0b00 << 6);
        dmaRxStream->FCR = (1 << 2) | (3 << 0);
    }
    /*DMA初始化*/

    initialized = true;
    return true;
}

bool I2C_CLASS::lock(double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive(mutex, Sync_waitTicks) == pdTRUE)
        return true;
    return false;
}

void I2C_CLASS::unlock()
{
    xSemaphoreGiveRecursive(mutex);
}

I2C_Result I2C_CLASS::write(const uint16_t address, const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size, I2C_SPEED speed, double TIMEOUT)
{
    if (tx_size == 0)
        return I2C_Result_ERROR;

    if (lock(TIMEOUT))
    {
        // 确定DMA缓冲区
        const uint8_t *data_buff;
#ifdef DCACHE_SIZE
        if (useBDMA)
        {
            // BDMA 只能访问 SRAM4，始终使用内置 noncacheBuf
            memcpy(noncacheBuf, tx_data, tx_size);
            data_buff = noncacheBuf;
        }
        else if (isDMABuf((uint32_t) tx_data))
        {
            data_buff = tx_data;
        }
        else if (((uint32_t) tx_data & (DCACHE_SIZE - 1)) == 0)
        {
            uint32_t aligned_size = CACHELINE_ALIGN_UP((uint32_t) tx_data + tx_size) - (uint32_t) tx_data;
            if (aligned_size <= tx_buffer_size)
            {
                data_buff = tx_data;
                SCB_CleanDCache_by_Addr((uint32_t *) tx_data, aligned_size);
            }
            else
            {
                memcpy(noncacheBuf, tx_data, tx_size);
                data_buff = noncacheBuf;
            }
        }
        else
        {
            memcpy(noncacheBuf, tx_data, tx_size);
            data_buff = noncacheBuf;
        }
#else
        data_buff = tx_data;
#endif

        // 计算当次发送长度
        uint16_t length;
        rmTxLength = tx_size;
        if (rmTxLength > 255)
            length = 255;
        else
            length = rmTxLength;
        rmTxLength -= length;

        // 清空状态
        xEventGroupClearBits(events, IIC_STOPF | IIC_NACKF | IIC_ARLO | IIC_BERR);
        i2c->CR1 &= ~((1 << 15) | (1 << 14));
        i2c->TIMINGR = timingR[speed];

        i2c->CR1 |= (1 << 0);
        /*配置DMA*/
        dmaTxStream->CR &= ~(1 << 0);
        *dmaTxIfcr = dmaTxIfcrVal;
        dmaTxStream->M0AR = (uint32_t) data_buff;
        dmaTxStream->NDTR = rmTxLength + length;
        dmaTxStream->CR |= (1 << 0);
        /*配置DMA*/
        i2c->CR1 |= (1 << 14);

        // 开始传输
        if (rmTxLength == 0)
            i2c->CR2 = IIC_AutoEND | IIC_NBytes(length) | IIC_ADDR7(address) | IIC_START;
        else
            i2c->CR2 = IIC_Reload | IIC_NBytes(length) | IIC_ADDR7(address) | IIC_START;

        // 等待传输完成
        uint32_t revents = xEventGroupWaitBits(events, IIC_STOPF | IIC_NACKF | IIC_ARLO | IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT);
        i2c->CR1 &= ~(1 << 0);
        unlock();

        if (revents & IIC_ARLO)
            return I2C_Result_ARBLOSE;
        else if (revents & IIC_BERR)
            return I2C_Result_ERROR;
        else if (revents & IIC_NACKF)
            return I2C_Result_NAK;
        else if (revents & IIC_STOPF)
            return I2C_Result_Success;
        return I2C_Result_timeout;
    }
    return I2C_Result_timeout;
}

I2C_Result I2C_CLASS::read(const uint16_t address, uint8_t *rx_data, const uint8_t rx_size, const uint8_t rx_buffer_size, I2C_SPEED speed, double TIMEOUT)
{
    if (rx_size == 0)
        return I2C_Result_ERROR;

    if (lock(TIMEOUT))
    {
        // 确定DMA缓冲区
        uint8_t *data_buff;
#ifdef DCACHE_SIZE
        if (useBDMA)
        {
            // BDMA 只能访问 SRAM4，始终使用内置 noncacheBuf
            data_buff = noncacheBuf;
        }
        else if (isDMABuf((uint32_t) rx_data))
        {
            data_buff = rx_data;
        }
        else if (((uint32_t) rx_data & (DCACHE_SIZE - 1)) == 0)
        {
            uint32_t aligned_size = CACHELINE_ALIGN_UP((uint32_t) rx_data + rx_size) - (uint32_t) rx_data;
            if (aligned_size <= rx_buffer_size)
            {
                data_buff = rx_data;
                SCB_CleanInvalidateDCache_by_Addr((uint32_t *) rx_data, aligned_size);
            }
            else
            {
                data_buff = noncacheBuf;
            }
        }
        else
        {
            data_buff = noncacheBuf;
        }
#else
        data_buff = rx_data;
#endif

        // 计算当次接收长度
        uint16_t rx_length = rx_size;
        rmRxLength = rx_length;
        if (rmRxLength > 255)
            rx_length = 255;
        else
            rx_length = rmRxLength;
        rmRxLength -= rx_length;
        rmTxLength = 0;

        // 清空状态
        xEventGroupClearBits(events, IIC_STOPF | IIC_NACKF | IIC_ARLO | IIC_BERR);
        i2c->CR1 &= ~((1 << 15) | (1 << 14));
        i2c->TIMINGR = timingR[speed];

        i2c->CR1 |= (1 << 0);
        /*配置DMA*/
        dmaRxStream->CR &= ~(1 << 0);
        *dmaRxIfcr = dmaRxIfcrVal;
        dmaRxStream->M0AR = (uint32_t) data_buff;
        dmaRxStream->NDTR = rmRxLength + rx_length;
        dmaRxStream->CR |= (1 << 0);
        /*配置DMA*/

        i2c->CR1 |= (1 << 15);

        // 开始传输
        if (rmRxLength == 0)
            i2c->CR2 = IIC_READ | IIC_AutoEND | IIC_NBytes(rx_length) | IIC_ADDR7(address) | IIC_START;
        else
            i2c->CR2 = IIC_READ | IIC_Reload | IIC_NBytes(rx_length) | IIC_ADDR7(address) | IIC_START;

        // 等待传输完成
        uint32_t revents = xEventGroupWaitBits(events, IIC_STOPF | IIC_NACKF | IIC_ARLO | IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT);
        i2c->CR1 &= ~(1 << 0);

        if (data_buff != rx_data)
            memcpy(rx_data, data_buff, rx_size);

        unlock();

        if (revents & IIC_ARLO)
            return I2C_Result_ARBLOSE;
        else if (revents & IIC_BERR)
            return I2C_Result_ERROR;
        else if (revents & IIC_NACKF)
            return I2C_Result_NAK;
        else if (revents & IIC_STOPF)
            return I2C_Result_Success;
        return I2C_Result_timeout;
    }
    return I2C_Result_timeout;
}

I2C_Result I2C_CLASS::writeNread(const uint16_t address, const uint8_t *tx_data, const uint8_t tx_size, const uint8_t tx_buffer_size, uint8_t *rx_data, const uint8_t rx_size,
                                 const uint8_t rx_buffer_size, I2C_SPEED speed, double TIMEOUT)
{
    if (tx_size == 0 || rx_size == 0)
        return I2C_Result_ERROR;

    if (lock(TIMEOUT))
    {
        // 确定TX DMA缓冲区
        const uint8_t *tx_buff;
#ifdef DCACHE_SIZE
        if (useBDMA)
        {
            // BDMA 只能访问 SRAM4，始终使用内置 noncacheBuf
            memcpy(noncacheBuf, tx_data, tx_size);
            tx_buff = noncacheBuf;
        }
        else if (isDMABuf((uint32_t) tx_data))
        {
            tx_buff = tx_data;
        }
        else if (((uint32_t) tx_data & (DCACHE_SIZE - 1)) == 0)
        {
            uint32_t aligned_size = CACHELINE_ALIGN_UP((uint32_t) tx_data + tx_size) - (uint32_t) tx_data;
            if (aligned_size <= tx_buffer_size)
            {
                tx_buff = tx_data;
                SCB_CleanDCache_by_Addr((uint32_t *) tx_data, aligned_size);
            }
            else
            {
                memcpy(noncacheBuf, tx_data, tx_size);
                tx_buff = noncacheBuf;
            }
        }
        else
        {
            memcpy(noncacheBuf, tx_data, tx_size);
            tx_buff = noncacheBuf;
        }
#else
        tx_buff = tx_data;
#endif

        // 确定RX DMA缓冲区
        uint8_t *rx_buff;
#ifdef DCACHE_SIZE
        if (useBDMA)
        {
            // BDMA 只能访问 SRAM4，始终使用内置 noncacheBuf + 128
            rx_buff = noncacheBuf + 128;
        }
        else if (isDMABuf((uint32_t) rx_data))
        {
            rx_buff = rx_data;
        }
        else if (((uint32_t) rx_data & (DCACHE_SIZE - 1)) == 0)
        {
            uint32_t aligned_size = CACHELINE_ALIGN_UP((uint32_t) rx_data + rx_size) - (uint32_t) rx_data;
            if (aligned_size <= rx_buffer_size)
            {
                rx_buff = rx_data;
                SCB_CleanInvalidateDCache_by_Addr((uint32_t *) rx_data, aligned_size);
            }
            else
            {
                rx_buff = noncacheBuf + 128;
            }
        }
        else
        {
            rx_buff = noncacheBuf + 128;
        }
#else
        rx_buff = rx_data;
#endif

        // 计算当次发送长度
        uint16_t length;
        rmTxLength = tx_size;
        if (rmTxLength > 255)
            length = 255;
        else
            length = rmTxLength;
        rmTxLength -= length;

        // 计算当次接收长度
        rmRxLength = rx_size;

        // 清空状态
        xEventGroupClearBits(events, IIC_STOPF | IIC_NACKF | IIC_ARLO | IIC_BERR);
        i2c->CR1 &= ~((1 << 15) | (1 << 14));
        i2c->TIMINGR = timingR[speed];

        i2c->CR1 |= (1 << 0);
        /*配置DMA*/
        // TX DMA
        dmaTxStream->CR &= ~(1 << 0);
        *dmaTxIfcr = dmaTxIfcrVal;
        dmaTxStream->M0AR = (uint32_t) tx_buff;
        dmaTxStream->NDTR = rmTxLength + length;
        dmaTxStream->CR |= (1 << 0);

        // RX DMA
        dmaRxStream->CR &= ~(1 << 0);
        *dmaRxIfcr = dmaRxIfcrVal;
        dmaRxStream->M0AR = (uint32_t) rx_buff;
        dmaRxStream->NDTR = rmRxLength;
        /*配置DMA*/
        i2c->CR1 |= (1 << 14);

        // 开始传输
        if (rmTxLength == 0)
            i2c->CR2 = IIC_NBytes(length) | IIC_ADDR7(address) | IIC_START;
        else
            i2c->CR2 = IIC_Reload | IIC_NBytes(length) | IIC_ADDR7(address) | IIC_START;

        // 等待传输完成
        uint32_t revents = xEventGroupWaitBits(events, IIC_STOPF | IIC_NACKF | IIC_ARLO | IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT);
        i2c->CR1 &= ~(1 << 0);

        if (rx_buff != rx_data)
            memcpy(rx_data, rx_buff, rx_size);

        unlock();

        if (revents & IIC_ARLO)
            return I2C_Result_ARBLOSE;
        else if (revents & IIC_BERR)
            return I2C_Result_ERROR;
        else if (revents & IIC_NACKF)
            return I2C_Result_NAK;
        else if (revents & IIC_STOPF)
            return I2C_Result_Success;
        return I2C_Result_timeout;
    }
    return I2C_Result_timeout;
}

void I2C_CLASS::handleEvIRQ()
{
    BaseType_t HigherPriorityTaskWoken = pdFALSE;
    uint8_t ISR = i2c->ISR;

    if (ISR & IIC_TC)
    { // 需要发送Restart
        if (rmRxLength > 0)
        {
            uint8_t length;
            if (rmRxLength > 255)
                length = 255;
            else
                length = rmRxLength;
            rmRxLength -= length;

            dmaRxStream->CR |= (1 << 0);
            i2c->CR1 |= (1 << 15);

            uint32_t addr = i2c->CR2 & 0x3ff;
            if (rmRxLength == 0)
                i2c->CR2 = IIC_READ | IIC_AutoEND | IIC_NBytes(length) | addr | IIC_START;
            else
                i2c->CR2 = IIC_READ | IIC_Reload | IIC_NBytes(length) | addr | IIC_START;
        }
        else
            i2c->CR2 = IIC_STOP;
    }
    else if (ISR & IIC_TCR)
    { // 继续发送
        if (rmTxLength > 0)
        {
            uint8_t length;
            if (rmTxLength > 255)
                length = 255;
            else
                length = rmTxLength;
            rmTxLength -= length;

            uint32_t CR2 = i2c->CR2 & (~(0xff << 16));
            i2c->CR2 = CR2 | IIC_NBytes(length);
        }
        else if (rmRxLength > 0)
        {
            uint8_t length;
            if (rmRxLength > 255)
                length = 255;
            else
                length = rmRxLength;
            rmRxLength -= length;

            uint32_t CR2 = i2c->CR2 & (~(0xff << 16));
            i2c->CR2 = CR2 | IIC_NBytes(length);
        }
        else
            i2c->CR2 = IIC_STOP;
    }
    else
        xEventGroupSetBitsFromISR(events, ISR, &HigherPriorityTaskWoken);

    i2c->ICR = ISR & ((1 << 5) | (1 << 4) | (1 << 3));

    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

void I2C_CLASS::handleErIRQ()
{
    BaseType_t HigherPriorityTaskWoken = pdFALSE;
    uint32_t ISR = i2c->ISR;

    i2c->ICR = ISR & ((1 << 13) | (1 << 12) | (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
    xEventGroupSetBitsFromISR(events, ISR, &HigherPriorityTaskWoken);

    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

/*======================== IRQ 中断分发 ========================*/

extern "C" void I2C1_EV_IRQHandler()
{
    if (i2c_inited[1])
        i2c_instances[1].handleEvIRQ();
}
extern "C" void I2C1_ER_IRQHandler()
{
    if (i2c_inited[1])
        i2c_instances[1].handleErIRQ();
}
extern "C" void I2C2_EV_IRQHandler()
{
    if (i2c_inited[2])
        i2c_instances[2].handleEvIRQ();
}
extern "C" void I2C2_ER_IRQHandler()
{
    if (i2c_inited[2])
        i2c_instances[2].handleErIRQ();
}
extern "C" void I2C3_EV_IRQHandler()
{
    if (i2c_inited[3])
        i2c_instances[3].handleEvIRQ();
}
extern "C" void I2C3_ER_IRQHandler()
{
    if (i2c_inited[3])
        i2c_instances[3].handleErIRQ();
}
extern "C" void I2C4_EV_IRQHandler()
{
    if (i2c_inited[4])
        i2c_instances[4].handleEvIRQ();
}
extern "C" void I2C4_ER_IRQHandler()
{
    if (i2c_inited[4])
        i2c_instances[4].handleErIRQ();
}

/*======================== 全局接口 ========================*/

void init_drv_i2c()
{
    // 读取i2cInt配置
    HW_ConfigEntry entry;
    if (HW_ReadConfigEntry(HW_DEV_I2CINT, HW_CFG_I2C_REF, &entry))
        i2cInt_num = (uint8_t) entry.value_low;
    if (HW_ReadConfigEntry(HW_DEV_I2CINT, HW_CFG_I2C_EXT, &entry))
        i2cInt_ext = (entry.value_low != 0);

    // 遍历所有I2C总线，根据硬件配置文件初始化
    for (uint8_t n = 1; n < MAX_I2C_INDEX; ++n)
    {
        if (HW_ReadConfigEntry(hwDevTable[n], HW_CFG_PIN1, &entry))
        {
            if (i2c_instances[n].init(n))
                i2c_inited[n] = true;
        }
    }
}

I2C_CLASS *get_i2c_instance(uint8_t i2c_num)
{
    if (i2c_num >= 1 && i2c_num < MAX_I2C_INDEX && i2c_inited[i2c_num])
        return &i2c_instances[i2c_num];
    return 0;
}

uint8_t get_i2cInt_num()
{
    return i2cInt_num;
}

bool get_i2cInt_ext()
{
    return i2cInt_ext;
}
