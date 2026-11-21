#include "dma.hpp"
#include "HardWareConfig.hpp"

#include "FreeRTOS.h"
#include "stm32h743xx.h"

// ==== 硬件通用查找表定义 ====
DMA_Stream_TypeDef *const HW_DMA1Streams[8] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
DMA_Stream_TypeDef *const HW_DMA2Streams[8] = {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};

// MDMA 通道表: MDMA_Channel0~15 (H743 共16个MDMA通道, 间距0x40)
MDMA_Channel_TypeDef *const HW_MDMAChannels[16] = {MDMA_Channel0, MDMA_Channel1, MDMA_Channel2,  MDMA_Channel3,  MDMA_Channel4,  MDMA_Channel5,  MDMA_Channel6,  MDMA_Channel7,
                                                   MDMA_Channel8, MDMA_Channel9, MDMA_Channel10, MDMA_Channel11, MDMA_Channel12, MDMA_Channel13, MDMA_Channel14, MDMA_Channel15};

const IRQn_Type HW_DmaIrqnTable[2][8] = {
    {DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn},
    {DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn},
};

const uint32_t HW_DmaClearBits[4] = {
    (1u << 5) | (1u << 4) | (1u << 3) | (1u << 2) | (1u << 0),      // Stream 0/4
    (1u << 11) | (1u << 10) | (1u << 9) | (1u << 8) | (1u << 6),    // Stream 1/5
    (1u << 21) | (1u << 20) | (1u << 19) | (1u << 18) | (1u << 16), // Stream 2/6
    (1u << 27) | (1u << 26) | (1u << 25) | (1u << 24) | (1u << 22), // Stream 3/7
};

// dma标志位位置表 (LISR/HISR寄存器中每4位对应一个stream的6个标志位)
static const uint8_t DMA_FLAG_POS[] = {0, 6, 16, 22};

// 清除DMA中断标志位
void clear_DMA_flags(uint8_t dma_num, uint8_t stream_num)
{
    if (dma_num < 1 || dma_num > 2 || stream_num > 7)
        return;
    if (stream_num < 4)
    {
        DMA_TypeDef *dma = dma_num == 1 ? DMA1 : DMA2;
        dma->LIFCR = 0x3D << DMA_FLAG_POS[stream_num];
    }
    else
    {
        DMA_TypeDef *dma = dma_num == 1 ? DMA1 : DMA2;
        dma->HIFCR = 0x3D << DMA_FLAG_POS[stream_num - 4];
    }
}

// 回调数组: [0..7] = DMA1_Stream0..7, [8..15] = DMA2_Stream0..7
static DMA_CallbackFunc dmaCallBacks[16] = {};

bool DMA_CallBackFunc_Register(uint8_t dma_num, uint8_t stream_num, DMA_CallbackFunc callback)
{
    if (dma_num < 1 || dma_num > 2 || stream_num > 7 || !callback)
        return false;
    uint8_t idx = (dma_num - 1) * 8 + stream_num;
    if (dmaCallBacks[idx])
        return false; // 已被占用
    dmaCallBacks[idx] = callback;
    return true;
}

// ---- 宏：生成 DMA ISR handler ----
// dma_n: 1或2  stream: 0~7  idx: (dma_n-1)*8+stream
// LIFCR: stream 0~3  HIFCR: stream 4~7
#define DMA_IRQ_HANDLER(dma_n, stream, idx)                                                                                                                                                            \
    extern "C" void DMA##dma_n##_Stream##stream##_IRQHandler()                                                                                                                                         \
    {                                                                                                                                                                                                  \
        BaseType_t xHigher = pdFALSE;                                                                                                                                                                  \
        uint32_t isr;                                                                                                                                                                                  \
        if (stream < 4)                                                                                                                                                                                \
        {                                                                                                                                                                                              \
            isr = (DMA##dma_n->LISR >> DMA_FLAG_POS[stream]) & 0x3D;                                                                                                                                   \
            DMA##dma_n->LIFCR = isr << DMA_FLAG_POS[stream];                                                                                                                                           \
        }                                                                                                                                                                                              \
        else                                                                                                                                                                                           \
        {                                                                                                                                                                                              \
            isr = (DMA##dma_n->HISR >> DMA_FLAG_POS[stream - 4]) & 0x3D;                                                                                                                               \
            DMA##dma_n->HIFCR = isr << DMA_FLAG_POS[stream - 4];                                                                                                                                       \
        }                                                                                                                                                                                              \
        if (dmaCallBacks[idx])                                                                                                                                                                         \
            dmaCallBacks[idx](isr, &xHigher);                                                                                                                                                          \
        portYIELD_FROM_ISR(xHigher);                                                                                                                                                                   \
    }

// DMA1 Stream 0~7
DMA_IRQ_HANDLER(1, 0, 0)
DMA_IRQ_HANDLER(1, 1, 1)
DMA_IRQ_HANDLER(1, 2, 2)
DMA_IRQ_HANDLER(1, 3, 3)
DMA_IRQ_HANDLER(1, 4, 4)
DMA_IRQ_HANDLER(1, 5, 5)
DMA_IRQ_HANDLER(1, 6, 6)
DMA_IRQ_HANDLER(1, 7, 7)

// DMA2 Stream 0~7
DMA_IRQ_HANDLER(2, 0, 8)
DMA_IRQ_HANDLER(2, 1, 9)
DMA_IRQ_HANDLER(2, 2, 10)
DMA_IRQ_HANDLER(2, 3, 11)
DMA_IRQ_HANDLER(2, 4, 12)
DMA_IRQ_HANDLER(2, 5, 13)
DMA_IRQ_HANDLER(2, 6, 14)
DMA_IRQ_HANDLER(2, 7, 15)

// MDMA 回调数组: 每个 MDMA 通道最多一个回调 (channel 0~15)
static MDMA_CallbackFunc mdmaCallBacks[16] = {};

// MDMA CIFCR 一次清除本通道全部5个标志位 (TEIF/CTCIF/BRTIF/BTIF/CLTCIF)
static inline uint32_t MDMA_ClearAllFlagsBits()
{
    return MDMA_CIFCR_CTEIF | MDMA_CIFCR_CCTCIF | MDMA_CIFCR_CBRTIF | MDMA_CIFCR_CBTIF | MDMA_CIFCR_CLTCIF;
}

bool MDMA_CallBackFunc_Register(uint8_t channel_num, MDMA_CallbackFunc callback)
{
    if (channel_num > 15 || !callback)
        return false;
    if (mdmaCallBacks[channel_num])
        return false; // 已被占用
    mdmaCallBacks[channel_num] = callback;
    return true;
}

// MDMA 中断服务 (H743 MDMA 全部16个通道共用单一 MDMA_IRQn)
// 处理流程: 读GISR0找到pending通道→读该通道CISR→清CIFCR→调用注册的回调
extern "C" void MDMA_IRQHandler()
{
    BaseType_t xHigher = pdFALSE;
    // GISR0 每位对应一个通道的全局中断标志 (GIF0~GIF15)
    uint32_t gisr = MDMA->GISR0;
    while (gisr)
    {
        // 找到最低位的pending通道
        uint32_t ch = __CLZ(__RBIT(gisr)); // trailing-zero → channel index
        if (ch < 16)
        {
            MDMA_Channel_TypeDef *chan = HW_MDMAChannels[ch];
            uint32_t isr = chan->CISR;
            chan->CIFCR = MDMA_ClearAllFlagsBits(); // 清除本通道所有标志位
            if (mdmaCallBacks[ch])
                mdmaCallBacks[ch]((uint8_t) isr, &xHigher);
            gisr &= ~(1UL << ch); // 清除已处理的位
        }
        else
        {
            break;
        }
    }
    portYIELD_FROM_ISR(xHigher);
}

void init_DMA(void)
{
    // 打开DMA1/DMA2时钟
    RCC->AHB1ENR |= (1 << 0) | (1 << 1);
    // 打开MDMA时钟 (AHB3, bit31=MDMAEN) 并使能 MDMA_IRQn
    RCC->AHB3ENR |= (1u << 31);
    NVIC_SetPriority(MDMA_IRQn, 6);
    NVIC_EnableIRQ(MDMA_IRQn);
}
