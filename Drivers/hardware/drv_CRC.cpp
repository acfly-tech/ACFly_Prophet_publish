#include "drv_CRC.hpp"
#include "Basic.hpp"
#include "dma.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stm32h743xx.h"
#include "stream_buffer.h"
#include "task.h"

#define CRC_REV_IN_None (0 << 5)
#define CRC_REV_IN8 (1 << 5)
#define CRC_REV_IN16 (2 << 5)
#define CRC_REV_IN32 (3 << 5)

#define CRC_REV_OUT_None (0 << 7)
#define CRC_REV_OUT (1 << 7)

#define CRC_POLYSIZE_32bit (0 << 3)
#define CRC_POLYSIZE_16bit (1 << 3)
#define CRC_POLYSIZE_8bit (2 << 3)
#define CRC_POLYSIZE_7bit (3 << 3)
#define CRC_RESET (1 << 0)

#define CRC_DR8 (*(uint8_t *) &CRC->DR)
#define CRC_DR16 (*(uint16_t *) &CRC->DR)
#define CRC_DR32 (*(uint32_t *) &CRC->DR)

// CRC 复用的 MDMA 通道 (与 drv_Flash 共享 MDMA_Channel0; 用户需确保两者互斥访问)
// 同时给出 通道号常量, 供 MDMA_CallBackFunc_Register 注册使用
#define MDMA_CRC_CHANNEL_IDX 1
#define MDMA_CRC_CHANNEL HW_MDMAChannels[MDMA_CRC_CHANNEL_IDX]

/*CRC多项式配置*/
// 默认的CRC-32校验-比如以太网
const CRC_Cfg cg_CRC_CONFIG_CRC32 = {CRC_POLY_SIZE_32bit, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true};

// mavlink CRC-32
const CRC_Cfg cg_CRC_CONFIG_CRC32_MAVLINK = {CRC_POLY_SIZE_32bit, 0x04C11DB7, 0, 0, true, true};

// CRC-32/MPEG-2校验
const CRC_Cfg cg_CRC_CONFIG_CRC32_MPEG2 = {CRC_POLY_SIZE_32bit, 0x04C11DB7, 0xFFFFFFFF, 0x00000000, false, false};

// CRC-16/XMODEM校验
const CRC_Cfg cg_CRC_CONFIG_CRC16_XMODEM = {CRC_POLY_SIZE_16bit, 0x1021, 0x0000, 0x0000, false, false};

// CRC-16/CCITT校验
const CRC_Cfg cg_CRC_CONFIG_CRC16_CCITT = {CRC_POLY_SIZE_16bit, 0x1021, 0x0000, 0x0000, true, true};

// CRC-16/MODBUS校验
const CRC_Cfg cg_CRC_CONFIG_CRC16_MODBUS = {CRC_POLY_SIZE_16bit, 0x8005, 0xffff, 0x0000, true, true};

// CRC-16/USB校验
const CRC_Cfg cg_CRC_CONFIG_CRC16_USB = {CRC_POLY_SIZE_16bit, 0x8005, 0xffff, 0xffff, true, true};

// CRC-16/IBM校验
const CRC_Cfg cg_CRC_CONFIG_CRC16_IBM = {CRC_POLY_SIZE_16bit, 0x8005, 0x0000, 0x0000, true, true};

// CRC-8
const CRC_Cfg cg_CRC_CONFIG_CRC8 = {CRC_POLY_SIZE_8bit, 0x07, 0x00, 0x00, false, false};

// CRC-8/MAXIM校验
const CRC_Cfg cg_CRC_CONFIG_CRC8_MAXIM = {CRC_POLY_SIZE_8bit, 0x31, 0x00, 0x00, true, true};

// CRC-8/ROHC校验
const CRC_Cfg cg_CRC_CONFIG_CRC8_ROHC = {CRC_POLY_SIZE_8bit, 0x07, 0xFF, 0x00, true, true};

// CRC-8/ITU校验
const CRC_Cfg cg_CRC_CONFIG_CRC8_ITU = {CRC_POLY_SIZE_8bit, 0x07, 0x00, 0x55, false, false};

// CRC-7/MMC校验
const CRC_Cfg cg_CRC_CONFIG_CRC7_MMC = {CRC_POLY_SIZE_7bit, 0x09, 0x00, 0x00, false, false};
/*CRC多项式配置*/

// 发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();
// CRC互斥锁
static SemaphoreHandle_t CRCSemphr = xSemaphoreCreateRecursiveMutex();

static inline void crc_calc_start(const CRC_Cfg &cfg, uint32_t crcLastRs = 0, bool crcConinue = false)
{
    // 配置CRC
    uint32_t CRC_CR = 0;
    if (cfg.isREFOUT)
        CRC_CR |= CRC_REV_OUT;
    if (cfg.isREFIN)
        CRC_CR |= CRC_REV_IN8;
    switch (cfg.Size)
    {
        case CRC_POLY_SIZE_7bit:
            CRC_CR |= CRC_POLYSIZE_7bit;
            break;
        case CRC_POLY_SIZE_8bit:
            CRC_CR |= CRC_POLYSIZE_8bit;
            break;
        case CRC_POLY_SIZE_16bit:
            CRC_CR |= CRC_POLYSIZE_16bit;
            break;
        case CRC_POLY_SIZE_32bit:
            CRC_CR |= CRC_POLYSIZE_32bit;
            break;
    }
    CRC->CR = CRC_CR | CRC_RESET;
    CRC->POL = cfg.Poly;
    if (crcConinue)
    {
        if (cfg.isREFOUT)
        { // 输出反转时
            // 需要先反转上次输出到本次输入
            crcLastRs = __RBIT((uint32_t) crcLastRs);
            switch (cfg.Size)
            {
                case CRC_POLY_SIZE_7bit:
                case CRC_POLY_SIZE_8bit:
                    crcLastRs = crcLastRs >> 24;
                    break;
                case CRC_POLY_SIZE_16bit:
                    crcLastRs = crcLastRs >> 16;
                    break;
                default:
                    break;
            }
        }
        CRC->INIT = crcLastRs;
    }
    else
        CRC->INIT = cfg.Init;
}
static inline bool crc_calcPart(const uint8_t *data, uint16_t length)
{
    // 使用MDMA将数据搬送到CRC->DR (MDMA可访问DTCM/AXI/SRAM等所有内存域)
    // 等待上一次传输结束 (channel ready)
    // while ((MDMA_CRC_CHANNEL->CISR & MDMA_CISR_CRQA) != 0)
    // {
    // }
    // 清除可能残留的 MDMA 通道标志
    MDMA_CRC_CHANNEL->CIFCR = MDMA_CIFCR_CTEIF | MDMA_CIFCR_CCTCIF | MDMA_CIFCR_CBRTIF | MDMA_CIFCR_CBTIF | MDMA_CIFCR_CLTCIF;

    // 配置MDMA传输参数
    MDMA_CRC_CHANNEL->CBNDTR = (length & MDMA_CBNDTR_BNDT); // 传输字节数 (单 block)
    MDMA_CRC_CHANNEL->CSAR = (uint32_t) data;               // 源地址
    MDMA_CRC_CHANNEL->CDAR = (uint32_t) &CRC->DR;           // 目的地址 (固定)

    // 总线选择 (CTBR): 仿 STM32 HAL MDMA 逻辑 — 决定源/目分别走 AXI 还是 AHB
    // 经验规则: DTCM(0x20000000)/ITCM(0x00000000) 走 AHB(SBUS/DBUS=1), 其余走 AXI(=0)
    // 注: CRC外设位于AHB4(0x5802_4C00), 其DRAM访问通过MDMA的AXI端口访问 (默认 SBUS/DBUS=0),
    //     与HAL一致 (本规则对所有使用场景——含DTCM源、AXI SRAM源——均正确)
    uint32_t ctbr = 0;
    {
        uint32_t srcMask = ((uint32_t) data) & 0xFF000000U;
        if (srcMask == 0x20000000U || srcMask == 0x00000000U)
            ctbr |= MDMA_CTBR_SBUS; // 源在 DTCM/ITCM → AHB总线读
        uint32_t dstMask = ((uint32_t) &CRC->DR) & 0xFF000000U;
        if (dstMask == 0x20000000U || dstMask == 0x00000000U)
            ctbr |= MDMA_CTBR_DBUS; // 目在 DTCM/ITCM → AHB总线写 (CRC外设不在该范围, 故DBUS=0走AXI)
    }
    MDMA_CRC_CHANNEL->CTBR = ctbr;

    // 等待传输完成事件
    xEventGroupClearBits(events, 1);
    // 使能中断并使能通道、启动软件请求
    MDMA_CRC_CHANNEL->CCR = MDMA_CCR_BTIE | MDMA_CCR_CTCIE | MDMA_CCR_TEIE;
    MDMA_CRC_CHANNEL->CCR |= MDMA_CCR_EN;
    MDMA_CRC_CHANNEL->CCR |= MDMA_CCR_SWRQ;

    EventBits_t rtbits = xEventGroupWaitBits(events, (1 << 0), pdFALSE, pdTRUE, 1 * configTICK_RATE_HZ);
    if (rtbits == (1 << 0))
        return true;
    else
        return false;
}
static inline void calc_crcResult(const CRC_Cfg &cfg, uint32_t *res)
{
    if (res)
    {
        switch (cfg.Size)
        {
            case CRC_POLY_SIZE_7bit:
                *res = CRC_DR8;
                *res ^= (uint8_t) cfg.XOROUT;
                break;
            case CRC_POLY_SIZE_8bit:
                *res = CRC_DR8;
                *res ^= (uint8_t) cfg.XOROUT;
                break;
            case CRC_POLY_SIZE_16bit:
                *res = CRC_DR16;
                *res ^= (uint16_t) cfg.XOROUT;
                break;
            case CRC_POLY_SIZE_32bit:
                *res = CRC_DR32;
                *res ^= (uint32_t) cfg.XOROUT;
                break;
        }
    }
}

static inline bool calc_crc(const CRC_Cfg &cfg, const uint8_t *data, uint16_t length, uint32_t *res, uint32_t crcLastRs = 0, bool crcConinue = false)
{
    crc_calc_start(cfg, crcLastRs, crcConinue);
    if (crc_calcPart(data, length))
    {
        calc_crcResult(cfg, res);
        return true;
    }
    return false;
}

static inline bool lockCRC(double waitTime)
{
    uint32_t waitTicks;
    if (waitTime >= 0)
        waitTicks = waitTime * configTICK_RATE_HZ;
    else
        waitTicks = portMAX_DELAY;

    // 获取信号量
    if (xSemaphoreTakeRecursive(CRCSemphr, waitTicks) == pdTRUE)
        return true;
    return false;
}

static inline void unlockCRC()
{
    xSemaphoreGiveRecursive(CRCSemphr);
}

static bool crcStarted = false;
#ifdef DCACHE_SIZE
#define CRC_BUF_SIZE (2 * DCACHE_SIZE)
Static_AXIDMABuf uint8_t crcBuf[CRC_BUF_SIZE];
#endif

// 前置声明: MDMA channel0 中断回调 (定义在本文件末尾)
static void crcMdmaCallback(uint8_t isr, BaseType_t *HigherPriorityTaskWoken);
bool CRC_Calc(CRC_Cfg cfg, const uint8_t *data, uint16_t length, uint32_t *res, uint32_t buffer_size, uint32_t crcLastRs, bool crcConinue, double waitTime)
{
    // 获取信号量
    if (lockCRC(waitTime))
    {
        if (crcStarted)
        {
            unlockCRC();
            return false;
        }

        // 操作结果
        bool result = false;

#ifdef DCACHE_SIZE
        // MDMA 可访问所有内存域(DTCM/AXI/SRAM4等)，因此数据区域限制取消；
        // 仅需处理 D-Cache 一致性：非 DMA 缓冲区(可cache) 区域需 Clean 后再 MDMA，
        // 或拷贝进内部crcBuf分块MDMA。
        if (isDMABuf((uint32_t) data))
        { // 缓冲区为非cache区域
          // 直接用 MDMA 发送
            result = calc_crc(cfg, data, length, res, crcLastRs, crcConinue);
        }
        else
        {
            // 校验 buffer_size 是否足够覆盖对齐后的cache行
            uint32_t aligned_size = CACHELINE_ALIGN_UP((uint32_t) data + length) - (uint32_t) data;
            if (((uint32_t) data & (DCACHE_SIZE - 1)) == 0 && aligned_size <= buffer_size)
            { // 缓冲区cacheline对齐
                SCB_CleanDCache_by_Addr((uint32_t *) data, aligned_size);
                result = calc_crc(cfg, data, length, res, crcLastRs, crcConinue);
            }
            else
            { // 拷贝进内部crcBuf分块传送
                crc_calc_start(cfg, crcLastRs, crcConinue);
                int32_t current_length = length;
                while (current_length > 0)
                {
                    uint16_t current_cpyLength;
                    if (CRC_BUF_SIZE < current_length)
                        current_cpyLength = CRC_BUF_SIZE;
                    else
                        current_cpyLength = current_length;
                    memcpy(crcBuf, &data[length - current_length], current_cpyLength);
                    current_length -= current_cpyLength;
                    if (crc_calcPart(crcBuf, current_cpyLength) == false)
                    {
                        unlockCRC();
                        return false;
                    }
                }
                calc_crcResult(cfg, res);
                result = true;
            }
        }
#else
        result = calc_crc(cfg, data, length, res, crcLastRs, crcConinue);
#endif
        unlockCRC();
        return result;
    }
    return false;
}

void init_drv_CRC()
{
    // 打开CRC时钟 (AHB4). MDMA 时钟与 MDMA_IRQn 使能统一由 init_DMA() 负责
    RCC->AHB4ENR |= (1 << 19);
    os_delay(0.01);

    CRC->CR = CRC_REV_OUT | CRC_REV_IN8 | CRC_POLYSIZE_16bit | CRC_RESET;
    CRC->POL = 0X1021;
    CRC->INIT = 0;

    // MDMA channel0 初始化 (与 drv_Flash 共享, 两驱动都仅在各自锁保护下使用)
    // 先禁用 channel0 并清狅可能残留的通道标志
    MDMA_CRC_CHANNEL->CCR = 0;
    MDMA_CRC_CHANNEL->CIFCR = MDMA_CIFCR_CTEIF | MDMA_CIFCR_CCTCIF | MDMA_CIFCR_CBRTIF | MDMA_CIFCR_CBTIF | MDMA_CIFCR_CLTCIF;

    // 配置 MDMA 传输 (源递增 byte, 目的固定 CRC->DR byte访问)
    // 与原 DMA2 方案保持一致：源/目均按字节访问, MDMA 可访问所有内存域 (含DTCM/SRAM4)
    // CRC 的 DR 在字节访问下会自动按字节累加 (适用于 7/8/16/32bit 多项式)
    // CTCR 关键位:
    //   SINC=01 (源按 SSIZE 步进, SSIZE=byte → +1)
    //   DINC=00 (目地址固定不增量, 始终写到 &CRC->DR)
    //   SSIZE=00 / DSIZE=00 (源/目皆按字节访问)
    //   TRGM=01 (BLOCK_TRANSFER: 单次软件请求完成整个 block, 即 CBNDTR.BNDT 字节)
    //            ※ 若用 TRGM=00(BUFFER_TRANSFER), MDMA 每次请求只传 TLEN+1 字节然后停止,
    //              会造成 CRC 仅收到部分字节算出错误结果。
    //   SWRM=1  (软件请求模式)
    MDMA_CRC_CHANNEL->CTCR = (0b10 << MDMA_CTCR_SINC_Pos)  // SINC=01 → 源按字节递增
                             | (0u << MDMA_CTCR_DINC_Pos)  // DINC=00 → 目的固定
                             | (0u << MDMA_CTCR_SSIZE_Pos) // SSIZE=00 → byte
                             | (0u << MDMA_CTCR_DSIZE_Pos) // DSIZE=00 → byte
                             | MDMA_CTCR_TRGM_0            // TRGM=01 → BLOCK_TRANSFER 模式
                             | MDMA_CTCR_SWRM;             // SWRM=1 → 软件请求
    MDMA_CRC_CHANNEL->CBRUR = 0;
    MDMA_CRC_CHANNEL->CLAR = 0;
    MDMA_CRC_CHANNEL->CMAR = 0;
    MDMA_CRC_CHANNEL->CMDR = 0;

    // 注册 channel0 的 MDMA 中断回调 (由 dma.cpp MDMA_IRQHandler 统一分派)
    MDMA_CallBackFunc_Register(MDMA_CRC_CHANNEL_IDX, crcMdmaCallback);
}

// MDMA channel0 中断回调 (由 dma.cpp MDMA_IRQHandler 统一分派、自动清聋 CISR 标志)
// isr: 本通道 CISR 原始值 (已清除, 此处仅读出使用)
static void crcMdmaCallback(uint8_t isr, BaseType_t *HigherPriorityTaskWoken)
{
    // 禁用通道并清除使能位 (避免再次触发), 软件请求已完成
    MDMA_CRC_CHANNEL->CCR = 0;
    // 传输完成或错误均唤醒等待任务
    if (isr & (MDMA_CISR_CTCIF | MDMA_CISR_TEIF))
    {
        xEventGroupSetBitsFromISR(events, 1, HigherPriorityTaskWoken);
    }
}