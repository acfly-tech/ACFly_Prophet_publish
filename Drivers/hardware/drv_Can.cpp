#include "drv_can.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "HardWareConfig.hpp"
#include "Parameters.hpp"
#include "event_groups.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stream_buffer.h"
#include <list>

using namespace std;

// ===========================================================================
// FDCAN FIFO 尺寸配置
// STM32H743 Message RAM 总大小: 10KB = 2560 words
// 每个FDCAN_DATA_BYTES_64元素 = 2(头) + 16(数据) = 18 words
// 单路CAN (全部RAM): RxFifo0=64, TxFifo=32 → 1728 words < 2560 ✓
// 双路CAN (各占一半): RxFifo0=32, TxFifo=16 → 每路864 words, 共1728 < 2560 ✓
// ===========================================================================
#define CAN_ELMTS_WORDS 18 // 每个64字节FD帧元素占用的Message RAM words数
// 单路时使用较大FIFO (硬件上限: RxFifo0≤64, TxFifo≤32)
#define CAN_SINGLE_RXFIFO 64
#define CAN_SINGLE_TXFIFO 32
// 双路时各自使用较小FIFO
#define CAN_DUAL_RXFIFO 32
#define CAN_DUAL_TXFIFO 16

// ===========================================================================
// 每路CAN总线状态结构体
// ===========================================================================
struct CanBusState
{
    FDCAN_HandleTypeDef hfdcan;
    list<CanBusMailBox *> mailBoxes;
    SemaphoreHandle_t mailBoxesMutex;
    SemaphoreHandle_t TxSemphr;
    EventGroupHandle_t events;
    bool inited;
    uint8_t can_num; // 1 or 2

    // GPIO配置 (从INI读取)
    GPIO_TypeDef *tx_gpio;
    GPIO_TypeDef *rx_gpio;
    uint16_t tx_pin;
    uint16_t rx_pin;
    uint32_t tx_af;
    uint32_t rx_af;
};

static CanBusState g_can[3]; // index 0 unused, 1=CAN1, 2=CAN2

// ===========================================================================
// 辅助函数
// ===========================================================================
static inline bool lock_mailBoxes(CanBusState *drv, double waitTime)
{
    uint32_t ticks = (waitTime > 0) ? (uint32_t) (waitTime * configTICK_RATE_HZ) : portMAX_DELAY;
    return xSemaphoreTakeRecursive(drv->mailBoxesMutex, ticks) == pdTRUE;
}
static inline void unlock_mailBoxes(CanBusState *drv)
{
    xSemaphoreGiveRecursive(drv->mailBoxesMutex);
}
static inline bool Lock_Can(CanBusState *drv, double waitTime)
{
    uint32_t ticks = (waitTime > 0) ? (uint32_t) (waitTime * configTICK_RATE_HZ) : portMAX_DELAY;
    return xSemaphoreTakeRecursive(drv->TxSemphr, ticks) == pdTRUE;
}
static inline void Unlock_Can(CanBusState *drv)
{
    xSemaphoreGiveRecursive(drv->TxSemphr);
}

// ===========================================================================
// 波特率自动求解
// ===========================================================================
static bool solve_req_baud(double pclk, double req_baud, uint16_t *Prescaler, uint16_t *TimeSeg1, uint16_t *TimeSeg2, uint16_t *SyncJumpWidth)
{
    double rp = pclk / req_baud;
    double min_err = 1;
    int16_t best = -1;
    for (uint16_t i = 48; i >= 4; --i)
    {
        double p = rp / (double) (i + 1);
        double err = p - (int32_t) p;
        if (err > 0.5)
            err = 1.0 - err;
        if (err < 0.00001)
        {
            min_err = err;
            best = i;
            break;
        }
        if (err < min_err)
        {
            min_err = err;
            best = i;
        }
    }
    if (min_err > 0.1 || best < 0)
        return false;
    *Prescaler = (uint16_t) round(rp / (double) (best + 1));
    *TimeSeg1 = (uint16_t) round((double) (best + 1) * 0.7) - 1;
    if (*TimeSeg1 == 0)
        *TimeSeg1 = 1;
    else if (*TimeSeg1 == (uint16_t) best)
        (*TimeSeg1)--;
    *TimeSeg2 = (uint16_t) best - *TimeSeg1;
    *SyncJumpWidth = (uint16_t) round(200.0 / *Prescaler);
    if (*SyncJumpWidth < 1)
        *SyncJumpWidth = 1;
    else if (*SyncJumpWidth > 127)
        *SyncJumpWidth = 127;
    return true;
}

// ===========================================================================
// MspInit — 读INI配置GPIO引脚并初始化
// ===========================================================================
static void can_msp_init(CanBusState *drv)
{
    uint8_t n = drv->can_num;

    // 读取TX/RX引脚 (从INI: tx_pin, rx_pin, tx_af, rx_af)
    HW_Device hw_dev = (n == 1) ? HW_DEV_CAN1 : HW_DEV_CAN2;
    HW_ConfigEntry entry;

    uint16_t tx_port = 0, tx_pin = 0, rx_port = 0, rx_pin = 0;
    uint32_t tx_af = 9, rx_af = 9; // FDCAN默认AF

    if (HW_ReadConfigEntry(hw_dev, HW_CFG_PIN1, &entry))
    {
        tx_port = entry.value_high;
        tx_pin = entry.value_low;
    }
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_PIN2, &entry))
    {
        rx_port = entry.value_high;
        rx_pin = entry.value_low;
    }
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF1, &entry))
        tx_af = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_AF2, &entry))
        rx_af = entry.value_low;

    // 使能GPIO时钟
    __HAL_RCC_FDCAN_CLK_ENABLE();
    RCC->AHB4ENR |= (1u << tx_port);
    if (rx_port != tx_port)
        RCC->AHB4ENR |= (1u << rx_port);
    os_delay(1e-3);

    GPIO_TypeDef *tx_gpio = HW_GpioTable[tx_port];
    GPIO_TypeDef *rx_gpio = HW_GpioTable[rx_port];

    // TX引脚
    tx_gpio->OTYPER &= ~(1u << tx_pin);
    set_register(tx_gpio->PUPDR, 0b01, tx_pin * 2, 2);
    set_register(tx_gpio->OSPEEDR, 0b11, tx_pin * 2, 2);
    if (tx_pin < 8)
        set_register(tx_gpio->AFR[0], tx_af, tx_pin * 4, 4);
    else
        set_register(tx_gpio->AFR[1], tx_af, (tx_pin - 8) * 4, 4);
    set_register(tx_gpio->MODER, 0b10, tx_pin * 2, 2);

    // RX引脚
    rx_gpio->OTYPER &= ~(1u << rx_pin);
    set_register(rx_gpio->PUPDR, 0b01, rx_pin * 2, 2);
    set_register(rx_gpio->OSPEEDR, 0b11, rx_pin * 2, 2);
    if (rx_pin < 8)
        set_register(rx_gpio->AFR[0], rx_af, rx_pin * 4, 4);
    else
        set_register(rx_gpio->AFR[1], rx_af, (rx_pin - 8) * 4, 4);
    set_register(rx_gpio->MODER, 0b10, rx_pin * 2, 2);

    // NVIC
    IRQn_Type it0, it1;
    if (n == 1)
    {
        it0 = FDCAN1_IT0_IRQn;
        it1 = FDCAN1_IT1_IRQn;
    }
    else
    {
        it0 = FDCAN2_IT0_IRQn;
        it1 = FDCAN2_IT1_IRQn;
    }
    NVIC_SetPriority(it0, 5);
    NVIC_EnableIRQ(it0);
    NVIC_SetPriority(it1, 5);
    NVIC_EnableIRQ(it1);
}

// ===========================================================================
// 单路CAN总线初始化
// ===========================================================================
static void init_can_bus(uint8_t n, uint8_t rx_cnt, uint8_t tx_cnt, uint32_t ram_offset)
{
    if (n < 1 || n > 2)
        return;
    CanBusState *drv = &g_can[n];
    if (drv->inited)
        return;

    // 检查INI中是否有该CAN口的引脚配置
    HW_Device hw_dev = (n == 1) ? HW_DEV_CAN1 : HW_DEV_CAN2;
    HW_ConfigEntry entry;
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN1, &entry))
        return; // INI中无此CAN配置，不初始化

    drv->can_num = n;
    drv->TxSemphr = xSemaphoreCreateRecursiveMutex();
    drv->mailBoxesMutex = xSemaphoreCreateRecursiveMutex();
    drv->events = xEventGroupCreate();

    FDCAN_HandleTypeDef *hfdcan = &drv->hfdcan;
    // 不调用 HAL_FDCAN_DeInit: g_can 为全局零初始化，State=HAL_FDCAN_STATE_RESET，
    // 直接调用 HAL_FDCAN_Init 即可。若先调用 DeInit 会在 Instance 未赋值时触发
    // HAL_FDCAN_MspDeInit → __HAL_RCC_FDCAN_FORCE_RESET，导致已运行的另一路CAN被复位。
    hfdcan->Instance = (n == 1) ? FDCAN1 : FDCAN2;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = DISABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = ENABLE;

#ifdef SYSFREQ480
    uint32_t pclk = 240000000;
#else
    uint32_t pclk = 200000000;
#endif
    double req_NominalFreq = 500000.0;
    double req_DataFreq = 5000000.0;

    char canGrpName[8];
    sprintf(canGrpName, "CAN%d", n);
    CANConfig can_cfg;
    if (ReadParamGroup(canGrpName, (uint64_t *) &can_cfg, 0) == PR_OK)
    {
        req_NominalFreq = can_cfg.Baud[0];
        req_DataFreq = can_cfg.DataBaud[0];
    }

    uint16_t P, S1, S2, SJW;
    if (!solve_req_baud(pclk, req_NominalFreq, &P, &S1, &S2, &SJW))
        solve_req_baud(pclk, 500000.0, &P, &S1, &S2, &SJW);
    hfdcan->Init.NominalPrescaler = P;
    hfdcan->Init.NominalSyncJumpWidth = SJW;
    hfdcan->Init.NominalTimeSeg1 = S1;
    hfdcan->Init.NominalTimeSeg2 = S2;

    if (!solve_req_baud(pclk, req_DataFreq, &P, &S1, &S2, &SJW))
        solve_req_baud(pclk, 5000000.0, &P, &S1, &S2, &SJW);
    hfdcan->Init.DataPrescaler = P;
    hfdcan->Init.DataSyncJumpWidth = SJW;
    hfdcan->Init.DataTimeSeg1 = S1;
    hfdcan->Init.DataTimeSeg2 = S2;

    hfdcan->Init.StdFiltersNbr = 0;
    hfdcan->Init.ExtFiltersNbr = 0;
    hfdcan->Init.MessageRAMOffset = ram_offset;
    hfdcan->Init.RxFifo0ElmtsNbr = rx_cnt;
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan->Init.RxFifo1ElmtsNbr = 0;
    hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan->Init.RxBuffersNbr = 0;
    hfdcan->Init.TxEventsNbr = 0;
    hfdcan->Init.TxBuffersNbr = 0;
    hfdcan->Init.TxFifoQueueElmtsNbr = tx_cnt;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_64;

    HAL_FDCAN_Init(hfdcan); // 触发 HAL_FDCAN_MspInit

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_WATERMARK, rx_cnt);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0xffffffff);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 1);
    HAL_FDCAN_Start(hfdcan);

    drv->inited = true;
}

// ===========================================================================
// HAL_FDCAN_MspInit 全局回调
// ===========================================================================
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    for (uint8_t n = 1; n <= 2; ++n)
    {
        if (hfdcan->Instance == g_can[n].hfdcan.Instance && !g_can[n].inited)
        {
            can_msp_init(&g_can[n]);
            return;
        }
    }
}

extern "C" void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    // 只有在两路CAN都未运行时才复位FDCAN时钟块，避免复位已运行的另一路
    for (uint8_t n = 1; n <= 2; ++n)
        if (g_can[n].inited)
            return;
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();
}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    BaseType_t hp = pdFALSE;
    for (uint8_t n = 1; n <= 2; ++n)
        if (g_can[n].inited && hfdcan->Instance == g_can[n].hfdcan.Instance)
            xEventGroupSetBitsFromISR(g_can[n].events, (1 << 0), &hp);
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_WATERMARK)) == 0)
        return;
    BaseType_t hp = pdFALSE;
    for (uint8_t n = 1; n <= 2; ++n)
    {
        if (g_can[n].inited && hfdcan->Instance == g_can[n].hfdcan.Instance)
        {
            xLPDaemonTaskCallFromISR(CanBusMailBox::CanRxTCB, &g_can[n], 0, &hp);
            break;
        }
    }
    portYIELD_FROM_ISR(hp);
}

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_can[1].hfdcan);
}
extern "C" void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_can[1].hfdcan);
}
extern "C" void FDCAN2_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_can[2].hfdcan);
}
extern "C" void FDCAN2_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_can[2].hfdcan);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
        hfdcan->Instance->CCCR |= FDCAN_CCCR_CCE;
        hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    }
}

// ===========================================================================
// init_drv_can — 根据INI配置初始化各CAN总线
// 动态分配 Message RAM: 先统计启用的CAN数, 再按比例分配 FIFO 大小
// ===========================================================================
void init_drv_can(void)
{
    // 统计INI中已配置的CAN总线
    bool can_enabled[3] = {false, false, false};
    uint8_t can_count = 0;
    HW_ConfigEntry _entry;
    for (uint8_t n = 1; n <= 2; ++n)
    {
        HW_Device hw_dev = (n == 1) ? HW_DEV_CAN1 : HW_DEV_CAN2;
        if (HW_ReadConfigEntry(hw_dev, HW_CFG_PIN1, &_entry))
        {
            can_enabled[n] = true;
            ++can_count;
        }
    }

    // 根据CAN数量决定每路的FIFO规格
    uint8_t rx_cnt = (can_count <= 1) ? CAN_SINGLE_RXFIFO : CAN_DUAL_RXFIFO;
    uint8_t tx_cnt = (can_count <= 1) ? CAN_SINGLE_TXFIFO : CAN_DUAL_TXFIFO;
    uint32_t words_per_bus = (uint32_t) (rx_cnt + tx_cnt) * CAN_ELMTS_WORDS;

    // 按顺序分配RAM偏移
    uint32_t offset = 0;
    for (uint8_t n = 1; n <= 2; ++n)
    {
        if (can_enabled[n])
        {
            init_can_bus(n, rx_cnt, tx_cnt, offset);
            offset += words_per_bus;
        }
    }
}

bool can_is_inited(uint8_t can_num)
{
    if (can_num < 1 || can_num > 2)
        return false;
    return g_can[can_num].inited;
}

CanMailBox *create_can_mailbox(uint8_t can_num)
{
    if (!can_is_inited(can_num))
        return nullptr;
    return new CanBusMailBox(can_num);
}

// ===========================================================================
// CanBusMailBox 实现
// ===========================================================================
bool CanBusMailBox::checkMail(CanId id)
{
    for (uint16_t i = 0; i < this->rx_idCount; ++i)
    {
        if (id.IdType)
        {
            if (this->ids[i] < 0x20000000)
                if ((id.Identifier & this->id_masks[i]) == (this->ids[i] & this->id_masks[i]))
                    return true;
        }
        else
        {
            if (this->ids[i] >= 0x20000000)
                if ((id.Identifier & this->id_masks[i]) == ((this->ids[i] - 0x20000000) & this->id_masks[i]))
                    return true;
        }
    }
    return false;
}

bool CanBusMailBox::init(uint16_t Capacity, CanIdWithMask *Ids, uint16_t Ids_count)
{
    if (this->rx_idCount)
        return false;
    if (!drv || !drv->inited)
    {
        this->rx_idCount = 0;
        this->packet_queue = 0;
        return false;
    }

    if (Ids && Ids_count)
    {
        this->rx_idCount = Ids_count;
        this->ids = new uint32_t[Ids_count];
        this->id_masks = new uint32_t[Ids_count];
        for (uint16_t i = 0; i < Ids_count; ++i)
        {
            uint32_t mask = Ids[i].mask ? Ids[i].mask : 0xffffffff;
            if (Ids[i].IdType)
                this->ids[i] = Ids[i].Identifier & mask;
            else
                this->ids[i] = (Ids[i].Identifier & mask) + 0x20000000;
            this->id_masks[i] = mask;
        }
        this->packet_queue = xQueueCreate(Capacity, sizeof(CanPacket));
        lock_mailBoxes(drv, -1);
        drv->mailBoxes.push_back(this);
        unlock_mailBoxes(drv);
        this->mail_overflow_cnt = 0;
        return true;
    }
    this->rx_idCount = 0;
    this->packet_queue = 0;
    return false;
}

bool CanBusMailBox::init(uint16_t Capacity, CanId *Ids, uint16_t Ids_count)
{
    if (this->rx_idCount)
        return false;
    if (!drv || !drv->inited)
    {
        this->rx_idCount = 0;
        this->packet_queue = 0;
        return false;
    }

    if (Ids && Ids_count)
    {
        this->rx_idCount = Ids_count;
        this->ids = new uint32_t[Ids_count];
        this->id_masks = new uint32_t[Ids_count];
        for (uint16_t i = 0; i < Ids_count; ++i)
        {
            this->ids[i] = (Ids[i].IdType ? Ids[i].Identifier : Ids[i].Identifier + 0x20000000);
            this->id_masks[i] = 0xffffffff;
        }
        this->packet_queue = xQueueCreate(Capacity, sizeof(CanPacket));
        lock_mailBoxes(drv, -1);
        drv->mailBoxes.push_back(this);
        unlock_mailBoxes(drv);
        this->mail_overflow_cnt = 0;
        return true;
    }
    this->rx_idCount = 0;
    this->packet_queue = 0;
    return false;
}

CanBusMailBox::CanBusMailBox(uint8_t can_num)
    : drv(nullptr)
    , rx_idCount(0)
    , id_masks(nullptr)
    , ids(nullptr)
    , packet_queue(nullptr)
    , mail_overflow_cnt(0)
{
    if (can_num >= 1 && can_num <= 2 && g_can[can_num].inited)
        drv = &g_can[can_num];
}

CanBusMailBox::CanBusMailBox(uint8_t can_num, uint16_t Capacity, CanIdWithMask *Ids, uint16_t Ids_count)
    : drv(nullptr)
    , rx_idCount(0)
    , id_masks(nullptr)
    , ids(nullptr)
    , packet_queue(nullptr)
    , mail_overflow_cnt(0)
{
    if (can_num >= 1 && can_num <= 2 && g_can[can_num].inited)
        drv = &g_can[can_num];
    init(Capacity, Ids, Ids_count);
}

CanBusMailBox::CanBusMailBox(uint8_t can_num, uint16_t Capacity, CanId *Ids, uint16_t Ids_count)
    : drv(nullptr)
    , rx_idCount(0)
    , id_masks(nullptr)
    , ids(nullptr)
    , packet_queue(nullptr)
    , mail_overflow_cnt(0)
{
    if (can_num >= 1 && can_num <= 2 && g_can[can_num].inited)
        drv = &g_can[can_num];
    init(Capacity, Ids, Ids_count);
}

bool CanBusMailBox::receiveMail(CanPacket *mail, double waitTime)
{
    if (!packet_queue)
        return false;
    uint32_t ticks = (waitTime >= 0) ? (uint32_t) (waitTime * configTICK_RATE_HZ) : portMAX_DELAY;
    return xQueueReceive(packet_queue, mail, ticks) == pdTRUE;
}

bool CanBusMailBox::SendMail(CanPacket mail, double Send_waitTime, double Sync_waitTime)
{
    if (!drv || !drv->inited)
        return false;

    static const uint32_t dlc_table[] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
                                         FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
    uint32_t mail_DLC;
    if (mail.DataLength <= 8)
        mail_DLC = dlc_table[mail.DataLength];
    else if (mail.DataLength == 12)
        mail_DLC = FDCAN_DLC_BYTES_12;
    else if (mail.DataLength == 16)
        mail_DLC = FDCAN_DLC_BYTES_16;
    else if (mail.DataLength == 20)
        mail_DLC = FDCAN_DLC_BYTES_20;
    else if (mail.DataLength == 24)
        mail_DLC = FDCAN_DLC_BYTES_24;
    else if (mail.DataLength == 32)
        mail_DLC = FDCAN_DLC_BYTES_32;
    else if (mail.DataLength == 48)
        mail_DLC = FDCAN_DLC_BYTES_48;
    else if (mail.DataLength == 64)
        mail_DLC = FDCAN_DLC_BYTES_64;
    else
        return false;

    uint32_t waitTicks = (Send_waitTime >= 0) ? (uint32_t) (Send_waitTime * configTICK_RATE_HZ) : portMAX_DELAY;
    FDCAN_HandleTypeDef *hfdcan = &drv->hfdcan;

    if (Lock_Can(drv, Sync_waitTime))
    {
        while ((hfdcan->Instance->TXFQS & (1u << 21)) && waitTicks)
        {
            vTaskDelay(1);
            --waitTicks;
        }
        bool res = false;
        if (!(hfdcan->Instance->TXFQS & (1u << 21)))
        {
            FDCAN_TxHeaderTypeDef TxHeader;
            TxHeader.Identifier = mail.IdType == 0 ? (mail.Identifier & 0x7ff) : (mail.Identifier & 0x1FFFFFFF);
            TxHeader.IdType = mail.IdType == 0 ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
            TxHeader.TxFrameType = mail.FrameType == 0 ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME;
            TxHeader.DataLength = mail_DLC;
            TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            TxHeader.BitRateSwitch = FDCAN_BRS_ON;
            TxHeader.FDFormat = mail.FDFormat == 0 ? FDCAN_FRAME_CLASSIC : FDCAN_FRAME_FD_BRS;
            TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            TxHeader.MessageMarker = 0;
            res = (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, mail.data) == HAL_OK);
        }
        Unlock_Can(drv);
        return res;
    }
    return false;
}

// ISR回调: pvParameter1 指向 CanBusState*
void CanBusMailBox::CanRxTCB(void *pvParameter1, uint32_t /*ulParameter2*/)
{
    CanBusState *drv = (CanBusState *) pvParameter1;
    FDCAN_RxHeaderTypeDef RxHeader;
    CanPacket packet;
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (HAL_FDCAN_GetRxMessage(&drv->hfdcan, FDCAN_RX_FIFO0, &RxHeader, packet.data) == HAL_ERROR)
            break;
        CanId canId;
        canId.Identifier = RxHeader.Identifier;
        canId.IdType = (RxHeader.IdType == FDCAN_STANDARD_ID) ? 0 : 1;

        packet.IdType = canId.IdType;
        packet.DataLength = (uint8_t) (RxHeader.DataLength >> 16);
        packet.FrameType = (RxHeader.RxFrameType == FDCAN_DATA_FRAME) ? 0 : 1;
        packet.FDFormat = (RxHeader.FDFormat == FDCAN_CLASSIC_CAN) ? 0 : 1;
        packet.Identifier = RxHeader.Identifier;

        lock_mailBoxes(drv, -1);
        for (auto *mb : drv->mailBoxes)
        {
            if (mb->checkMail(canId))
            {
                if (xQueueSend(mb->packet_queue, &packet, 0) != pdTRUE)
                    ++mb->mail_overflow_cnt;
            }
        }
        unlock_mailBoxes(drv);
    }
}
