/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    sd_diskio.c
 * @brief   SD Disk I/O driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_rtos_template.c v2.0.2 as FreeRTOS is enabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_sd.h"
#include "stm32h7xx_hal.h"

#include "Basic.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define QUEUE_SIZE (uint32_t) 10
#define READ_CPLT_MSG (uint32_t) 1
#define WRITE_CPLT_MSG (uint32_t) 2
/* SD/DMA错误或Abort回调上报的消息，等待侧收到后立即终止本次传输 */
#define RW_FAIL_MSG (uint32_t) 3
/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT 30

#define SD_DEFAULT_BLOCK_SIZE 512

static uint32_t INTERNAL_SD_BLOCK_COUNT = 0;
static uint32_t USER_SD_BLOCK_COUNT = 0;
static uint32_t EXTERNAL_SD_BLOCK_COUNT = 0;

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
#define DISABLE_SD_INIT
/* USER CODE END disableSDInit */

/*
 * when using cachable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* USER CODE BEGIN enableSDDmaCacheMaintenance */
#ifdef DCACHE_SIZE
#define ENABLE_SD_DMA_CACHE_MAINTENANCE 1
Static_AXIDMABuf uint8_t scratch[BLOCKSIZE];
#else
__attribute__((aligned(4))) uint8_t scratch[BLOCKSIZE];
#endif
/* USER CODE BEGIN enableSDDmaCacheMaintenance */
extern uint8_t Lock_SD(double TIMEOUT);
extern void UnLock_SD(void);
extern uint8_t Check_USB_SD_Busy();
extern void Update_USB_SD_Busy_Time();
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static osMessageQId SDQueueID;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize(BYTE);
DSTATUS SD_status(BYTE);
DRESULT SD_read(BYTE, BYTE *, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write(BYTE, const BYTE *, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE, BYTE, void *);
#endif /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef SD_Driver = {
    SD_initialize, SD_status, SD_read,
#if _USE_WRITE == 1
    SD_write,
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
    SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
#define DMA_AVAI_BUF(addr)                                                                                                                                                                             \
    ((((uintptr_t) (addr) >= 0x24000000UL) && ((uintptr_t) (addr) < 0x24080000UL)) || /* AXI SRAM */                                                                                                   \
     (((uintptr_t) (addr) >= 0x30000000UL) && ((uintptr_t) (addr) < 0x30020000UL)) || /* SRAM1 */                                                                                                      \
     (((uintptr_t) (addr) >= 0x30040000UL) && ((uintptr_t) (addr) < 0x30048000UL)) || /* SRAM3 */                                                                                                      \
     (((uintptr_t) (addr) >= 0x08000000UL) && ((uintptr_t) (addr) < 0x08200000UL))    /* 内部 Flash */                                                                                                 \
    )
/* (((uintptr_t) (addr) >= 0x30020000UL) && ((uintptr_t) (addr) < 0x30040000UL)) || SRAM2 已排除: SD DMA 直写不可靠 */

/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE pdrv)
{
    // Stat = STA_NOINIT 必须在 Lock 内部，避免与 SD_read/write 并发时
    // 短暂写入 NOINIT 导致其他线程误判磁盘未就绪
    if (Lock_SD(-1))
    {
        Stat = STA_NOINIT;
        if (BSP_SD_GetCardState() == MSD_OK)
        {
            Stat &= ~STA_NOINIT;
        }
        UnLock_SD();
    }
    return Stat;
}

/**
 * @brief  Initializes a Drive
 * @param  lun : not used
 * @retval DSTATUS: Operation status
 */
DSTATUS SD_initialize(BYTE pdrv)
{
    BSP_SD_CardInfo CardInfo;
    /*
     * check that the kernel has been started before continuing
     * as the osMessage API will fail otherwise
     */
    if (osKernelRunning())
    {
        if (Lock_SD(-1))
        {
            // 移入 Lock 内部：避免与 SD_read/write 并发时写入 NOINIT 造成竞态
            Stat = STA_NOINIT;
#if !defined(DISABLE_SD_INIT)

            if (BSP_SD_Init() == MSD_OK)
            {
                Stat = SD_CheckStatus(lun);
            }

#else
            Stat = SD_CheckStatus(pdrv);
#endif

            /*
             * if the SD is correctly initialized, create the operation queue
             */

            static uint8_t SD_Queue_initialized = 0;
            if (Stat != STA_NOINIT && !SD_Queue_initialized)
            {
                SD_Queue_initialized = 1;
                osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
                SDQueueID = osMessageCreate(osMessageQ(SD_Queue), NULL);
                if (SDQueueID == NULL)
                { /* 队列创建失败(heap不足), 标记未初始化以阻止后续读写 */
                    Stat = STA_NOINIT;
                    SD_Queue_initialized = 0;
                }
                else
                {
                    BSP_SD_GetCardInfo(&CardInfo);
                    INTERNAL_SD_BLOCK_COUNT = INTERNAL_SD_SIZE / CardInfo.LogBlockSize;
                    USER_SD_BLOCK_COUNT = USER_SD_SIZE / CardInfo.LogBlockSize;
                    EXTERNAL_SD_BLOCK_COUNT = CardInfo.LogBlockNbr - INTERNAL_SD_BLOCK_COUNT - USER_SD_BLOCK_COUNT;
                }
            }

            UnLock_SD();
        }
    }
    return Stat;
}

/**
 * @brief  Gets Disk Status
 * @param  lun : not used
 * @retval DSTATUS: Operation status
 */
DSTATUS SD_status(BYTE pdrv)
{
    return SD_CheckStatus(pdrv);
}

static DRESULT SD_pdrv_map(BYTE pdrv, DWORD *sector, UINT *count)
{
    if (pdrv != DEV_SD && pdrv != DEV_SD_INT && pdrv != DEV_SD_USR && pdrv != DEV_SD_EXT)
    {
        return RES_PARERR;
    }
    switch (pdrv)
    {
        case DEV_SD_INT:
        {
            if (*sector + *count > INTERNAL_SD_BLOCK_COUNT)
                return RES_ERROR;
            *sector = *sector + EXTERNAL_SD_BLOCK_COUNT + USER_SD_BLOCK_COUNT;
            break;
        }
        case DEV_SD_USR:
        {
            if (*sector + *count > USER_SD_BLOCK_COUNT)
                return RES_ERROR;
            *sector = *sector + EXTERNAL_SD_BLOCK_COUNT;
            break;
        }
        case DEV_SD_EXT:
        {
            if (*sector + *count > EXTERNAL_SD_BLOCK_COUNT)
                return RES_ERROR;
            break;
        }
    }
    return RES_OK;
}

DRESULT SD_erase(BYTE pdrv, DWORD sector, UINT count)
{
    DRESULT res = RES_ERROR;
    osEvent event;
    uint32_t timer;
    BSP_SD_CardInfo CardInfo;

    // 扇区映射
    if (count == 0)
        return RES_ERROR;
    res = SD_pdrv_map(pdrv, &sector, &count);
    if (res != RES_OK)
        return res;
    res = RES_ERROR;

    if (Lock_SD(SD_TIMEOUT))
    {
        BSP_SD_GetCardInfo(&CardInfo);
        res = BSP_SD_Erase(sector, count * (CardInfo.BlockSize / CardInfo.LogBlockSize));

        timer = osKernelSysTick() + SD_TIMEOUT * 1000;
        /* block until SDIO IP is ready or a timeout occur */
        while (timer > osKernelSysTick())
        {
            vTaskDelay(1);
            if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
            {
                res = RES_OK;
                Stat &= ~STA_NOINIT;
                break;
            }
        }

        UnLock_SD();
    }
    return res;
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* 排空队列中残留的完成/错误消息(上次传输超时后迟到到达的)，防止干扰本次传输的等待判断。
 * 调用前提：已持有 Lock_SD，同一时刻仅存在一次传输 */
static void SD_DMA_FlushPendingMsgs(void)
{
    while (osMessageGet(SDQueueID, 0).status == osEventMessage)
        ;
}

/* 传输异常终止：停止SDMMC传输并排空可能残留的DMA消息，防止脏状态影响后续扇区/后续调用。
 * 若不停止，超时后残留的DMA状态会导致后续所有扇区连锁失败 */
static void SD_AbortTransfer(void)
{
    SDMMC_CmdStopTransfer(SDMMC1);
    SD_DMA_FlushPendingMsgs();
}
/* USER CODE END beforeReadSection */
/**
 * @brief  Reads Sector(s)
 * @param  lun : not used
 * @param  *buff: Data buffer to store read data
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to read (1..128)
 * @retval DRESULT: Operation result
 */
DRESULT SD_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_ERROR;
    osEvent event;
    uint32_t timer;

    // 扇区映射
    if (count == 0)
        return RES_ERROR;
    res = SD_pdrv_map(pdrv, &sector, &count);
    if (res != RES_OK)
        return res;
    res = RES_ERROR;

#ifdef DCACHE_SIZE
    if (DMA_AVAI_BUF(buff) && isDMABuf((uint32_t) buff) && (((uint32_t) buff & 0x3) == 0))
    { // 缓冲区为非cache区域且四字节对齐
#else
    if (((uint32_t) buff & 0x3) == 0)
    { // 缓冲区四字节对齐
#endif
        if (Lock_SD(SD_TIMEOUT))
        {
            // 直接dma送进缓冲区
            SD_DMA_FlushPendingMsgs();
            if (BSP_SD_ReadBlocks_DMA((uint32_t *) buff, (uint32_t) (sector), count) == MSD_OK)
            {
                /* wait for a message from the queue or a timeout */
                event = osMessageGet(SDQueueID, SD_TIMEOUT * 1000);

                if (event.status == osEventMessage && event.value.v == READ_CPLT_MSG)
                {
                    if (count > 1)
                        SDMMC_CmdStopTransfer(SDMMC1);
                    timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                    /* block until SDIO IP is ready or a timeout occur */
                    while (timer > osKernelSysTick())
                    {
                        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                        {
                            res = RES_OK;
                            Stat &= ~STA_NOINIT;
                            break;
                        }
                    }
                }
                else
                { /* 等待超时或收到错误消息: 终止传输并返回错误，不将残留状态留给下次调用 */
                    SD_AbortTransfer();
                }
            }
            UnLock_SD();
        }
    }
#ifdef DCACHE_SIZE
    else if (DMA_AVAI_BUF(buff) && ((uint32_t) buff & 0x1f) == 0)
    { // 缓冲区32字节对齐
        // 直接dma送进缓冲区
        // 将对应Cache数据清除
        if (Lock_SD(SD_TIMEOUT))
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *) buff, count * BLOCKSIZE);
            SD_DMA_FlushPendingMsgs();
            if (BSP_SD_ReadBlocks_DMA((uint32_t *) buff, (uint32_t) (sector), count) == MSD_OK)
            {
                /* wait for a message from the queue or a timeout */
                event = osMessageGet(SDQueueID, SD_TIMEOUT * 1000);

                if (event.status == osEventMessage && event.value.v == READ_CPLT_MSG)
                {
                    if (count > 1)
                        SDMMC_CmdStopTransfer(SDMMC1);
                    timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                    /* block until SDIO IP is ready or a timeout occur */
                    while (timer > osKernelSysTick())
                    {
                        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                        {
                            res = RES_OK;
                            Stat &= ~STA_NOINIT;
                            break;
                        }
                    }
                }
                else
                { /* 等待超时或收到错误消息: 终止传输并返回错误，不将残留状态留给下次调用 */
                    SD_AbortTransfer();
                }
            }
            UnLock_SD();
        }
    }
#endif
    else
    {
        // 缓冲区非32字节对齐或非4字节对齐
        // 把数据用dma传送至内置缓冲区在复制过去
        int i = 0;
        uint8_t ret = 1;
        bool sector_ok = true; // 所有已启动的扇区传输均成功收到完成消息且卡就绪
        if (Lock_SD(SD_TIMEOUT))
        {
            SD_DMA_FlushPendingMsgs();
            for (i = 0; i < count; i++)
            {
                ret = BSP_SD_ReadBlocks_DMA((uint32_t *) scratch, (uint32_t) sector++, 1);
                if (ret != MSD_OK)
                    break;

                /* wait for a message from the queue or a timeout */
                event = osMessageGet(SDQueueID, SD_TIMEOUT * 1000);
                if (event.status != osEventMessage || event.value.v != READ_CPLT_MSG)
                { /* 等待超时或错误消息: 终止整个读取，不把scratch中未更新的旧数据复制给调用者 */
                    SD_AbortTransfer();
                    sector_ok = false;
                    break;
                }

                timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                /* block until SDIO IP is ready or a timeout occur */
                bool ready = false;
                while (timer > osKernelSysTick())
                {
                    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                    {
                        ready = true;
                        break;
                    }
                }
                if (!ready)
                { /* 卡就绪等待超时: scratch内容不可信，终止并返回错误 */
                    SD_AbortTransfer();
                    sector_ok = false;
                    break;
                }

                // 把数据从内置缓冲区复制到buf
                memcpy((uint32_t *) buff, (uint32_t *) scratch, BLOCKSIZE);
                buff += BLOCKSIZE;
            }

            if (sector_ok && (i == count) && (ret == MSD_OK))
            {
                timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                /* block until SDIO IP is ready or a timeout occur */
                while (timer > osKernelSysTick())
                {
                    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                    {
                        res = RES_OK;
                        Stat &= ~STA_NOINIT;
                        break;
                    }
                }
            }

            UnLock_SD();
        }
    }
    return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
 * @brief  Writes Sector(s)
 * @param  lun : not used
 * @param  *buff: Data to be written
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to write (1..128)
 * @retval DRESULT: Operation result
 */
#if _USE_WRITE == 1
DRESULT SD_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_ERROR;
    osEvent event;
    uint32_t timer;

    // 扇区映射
    if (count == 0)
        return RES_ERROR;
    res = SD_pdrv_map(pdrv, &sector, &count);
    if (res != RES_OK)
        return res;
    res = RES_ERROR;

#ifdef DCACHE_SIZE
    if (DMA_AVAI_BUF(buff) && isDMABuf((uint32_t) buff) && (((uint32_t) buff & 0x3) == 0))
    { // 缓冲区为非cache区域或非四字节对齐
#else
    if (((uint32_t) buff & 0x3) == 0)
    { // 缓冲区四字节对齐
#endif
        if (Lock_SD(SD_TIMEOUT))
        {
            // 直接用dma发送
            SD_DMA_FlushPendingMsgs();
            if (BSP_SD_WriteBlocks_DMA((uint32_t *) buff, (uint32_t) (sector), count) == MSD_OK)
            {
                /* Get the message from the queue */
                event = osMessageGet(SDQueueID, SD_TIMEOUT * 1000);

                if (event.status == osEventMessage && event.value.v == WRITE_CPLT_MSG)
                {
                    if (count > 1)
                        SDMMC_CmdStopTransfer(SDMMC1);

                    timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                    /* block until SDIO IP is ready or a timeout occur */

                    while (timer > osKernelSysTick())
                    {
                        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                        {
                            res = RES_OK;
                            Stat &= ~STA_NOINIT;
                            break;
                        }
                    }
                }
                else
                { /* 等待超时或收到错误消息: 终止传输并返回错误 */
                    SD_AbortTransfer();
                }
            }
            UnLock_SD();
        }
    }
#ifdef DCACHE_SIZE
    else if (DMA_AVAI_BUF(buff) && ((uint32_t) buff & 0x1f) == 0)
    { // 缓冲区32字节对齐或非4字节对齐
        // 直接用dma发送
        if (Lock_SD(SD_TIMEOUT))
        {
            // 把Cache内容写入内存
            SCB_CleanDCache_by_Addr((uint32_t *) buff, count * BLOCKSIZE);
            SD_DMA_FlushPendingMsgs();
            if (BSP_SD_WriteBlocks_DMA((uint32_t *) buff, (uint32_t) (sector), count) == MSD_OK)
            {
                /* Get the message from the queue */
                event = osMessageGet(SDQueueID, SD_TIMEOUT * 1000);

                if (event.status == osEventMessage && event.value.v == WRITE_CPLT_MSG)
                {
                    if (count > 1)
                        SDMMC_CmdStopTransfer(SDMMC1);
                    timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                    /* block until SDIO IP is ready or a timeout occur */
                    while (timer > osKernelSysTick())
                    {
                        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                        {
                            res = RES_OK;
                            Stat &= ~STA_NOINIT;
                            break;
                        }
                    }
                }
                else
                { /* 等待超时或收到错误消息: 终止传输并返回错误 */
                    SD_AbortTransfer();
                }
            }
            UnLock_SD();
        }
    }
#endif
    else
    { // 缓冲区非32字节对齐
        // 把数据先复制到内置缓冲区再发送
        int i;
        uint8_t ret;
        if (Lock_SD(SD_TIMEOUT))
        {
            SD_DMA_FlushPendingMsgs();
            for (i = 0; i < count; i++)
            {
                // 数据从buf拷贝到scratch
                memcpy((void *) scratch, (void *) buff, BLOCKSIZE);
                ret = BSP_SD_WriteBlocks_DMA((uint32_t *) scratch, (uint32_t) sector++, 1);
                if (ret == MSD_OK)
                {
                    /* wait for a message from the queue or a timeout */
                    event = osMessageGet(SDQueueID, SD_TIMEOUT * 1000);
                    if (event.status == osEventMessage)
                    {
                        if (event.value.v == WRITE_CPLT_MSG)
                        {
                            timer = osKernelSysTick() + SD_TIMEOUT * 1000;
                            buff += BLOCKSIZE;

                            /* block until SDIO IP is ready or a timeout occur */
                            bool ready = false;
                            while (timer > osKernelSysTick())
                            {
                                if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                                {
                                    ready = true;
                                    break;
                                }
                            }
                            if (!ready)
                            { /* 卡就绪等待超时: 复位SDMMC防止脏状态影响后续扇区 */
                                SD_AbortTransfer();
                                res = RES_ERROR;
                                break;
                            }
                        }
                        else
                        { /* 收到错误消息类型, 终止传输 */
                            SD_AbortTransfer();
                            res = RES_ERROR;
                            break;
                        }
                    }
                    else
                    { /* 等待DMA完成超时, 终止传输防止后续扇区数据错位 */
                        SD_AbortTransfer();
                        res = RES_ERROR;
                        break;
                    }
                }
                else
                {
                    res = MSD_ERROR;
                    break;
                }
            }

            if ((i == count) && (ret == MSD_OK))
            {
                res = RES_OK;
                Stat &= ~STA_NOINIT;
            }

            UnLock_SD();
        }
    }
    return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
 * @brief  I/O control operation
 * @param  lun : not used
 * @param  cmd: Control code
 * @param  *buff: Buffer to send/receive control data
 * @retval DRESULT: Operation result
 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;
    BSP_SD_CardInfo CardInfo;

    // 在 Lock 保护下读取 Stat，避免与 SD_read/write 并发时读到瞬态 NOINIT
    {
        DSTATUS s;
        if (!Lock_SD(-1))
            return RES_NOTRDY;
        s = Stat;
        UnLock_SD();
        if (s & STA_NOINIT)
            return RES_NOTRDY;
    }
    if ((pdrv != DEV_SD && pdrv != DEV_SD_INT && pdrv != DEV_SD_USR && pdrv != DEV_SD_EXT) || ((cmd != CTRL_SYNC) && (buff == NULL)))
        return RES_PARERR;

    switch (cmd)
    {
        /* Make sure that no pending write process */
        case CTRL_SYNC:
            res = RES_OK;
            break;

        /* Get number of sectors on the disk (DWORD) */
        case GET_SECTOR_COUNT:
            switch (pdrv)
            {
                case DEV_SD:
                    if (Lock_SD(-1))
                    {
                        BSP_SD_GetCardInfo(&CardInfo);
                        UnLock_SD();
                        *(uint32_t *) buff = CardInfo.LogBlockNbr;
                    }
                    break;
                case DEV_SD_INT:
                    *(uint32_t *) buff = INTERNAL_SD_BLOCK_COUNT;
                    break;
                case DEV_SD_USR:
                    *(uint32_t *) buff = USER_SD_BLOCK_COUNT;
                    break;
                case DEV_SD_EXT:
                    *(uint32_t *) buff = EXTERNAL_SD_BLOCK_COUNT;
                    break;
            }
            res = RES_OK;
            break;

        /* Get R/W sector size (WORD) */
        case GET_SECTOR_SIZE:
            if (Lock_SD(-1))
            {
                BSP_SD_GetCardInfo(&CardInfo);
                UnLock_SD();
                *(uint32_t *) buff = CardInfo.LogBlockSize;
                res = RES_OK;
            }
            break;

        /* Get erase block size in unit of sector (DWORD) */
        case GET_BLOCK_SIZE:
            if (Lock_SD(-1))
            {
                BSP_SD_GetCardInfo(&CardInfo);
                UnLock_SD();
                *(uint32_t *) buff = CardInfo.BlockSize / CardInfo.LogBlockSize;
                res = RES_OK;
            }
            break;

        default:
            res = RES_PARERR;
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
 * @brief Tx Transfer completed callbacks
 * @param hsd: SD handle
 * @retval None
 */

/*
  ===============================================================================
   Select the correct function signature depending on your platform.
   please refer to the file "stm32xxxx_eval_sd.h" to verify the correct function
   prototype
  ===============================================================================
 */
// void BSP_SD_WriteCpltCallback(uint32_t SdCard)
void BSP_SD_WriteCpltCallback(void)
{
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
     */
    osMessagePut(SDQueueID, WRITE_CPLT_MSG, osWaitForever);
}

/**
 * @brief Rx Transfer completed callbacks
 * @param hsd: SD handle
 * @retval None
 */

/*
 ===============================================================================
  Select the correct function signature depending on your platform.
  please refer to the file "stm32xxxx_eval_sd.h" to verify the correct function
  prototype
 ===============================================================================
*/
// void BSP_SD_ReadCpltCallback(uint32_t SdCard)
void BSP_SD_ReadCpltCallback(void)
{
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
     */
    osMessagePut(SDQueueID, READ_CPLT_MSG, osWaitForever);
}

/**
 * @brief SD传输Abort回调(覆盖bsp_driver_sd.c中的__weak版本)
 * 上报RW_FAIL_MSG，等待侧收到后立即终止本次传输
 */
void BSP_SD_AbortCallback(void)
{
    if (SDQueueID)
        osMessagePut(SDQueueID, RW_FAIL_MSG, 0);
}

/**
 * @brief SD/DMA错误回调(覆盖HAL中的__weak版本)，在中断上下文被调用
 * 此前错误路径不上报任何消息，超时是唯一检测手段；现在主动上报，
 * 等待侧可立即收到失败消息并终止，而无需等满30s超时
 */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    (void) hsd;
    if (SDQueueID)
        osMessagePut(SDQueueID, RW_FAIL_MSG, 0);
}

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
