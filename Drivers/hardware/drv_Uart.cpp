#include "drv_Uart.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "HardWareConfig.hpp"
#include "dma.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "message_buffer.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "timers.h"

#include "stm32h743xx.h"

#define TxStreamBufferSize 1024
#define TxBufferSize 64
#define RxStreamBufferSize 1024

// 支持的 UART 编号 1~8
#define MAX_UART_INDEX 9

// --- 静态实例 ---
static UART_CLASS uart_instances[MAX_UART_INDEX]; // 索引 = uart_num
static bool uart_inited[MAX_UART_INDEX] = {};

// USART 外设指针表 [uart_num], 索引 1~8, [0]未使用
USART_TypeDef *const HW_UsartTable[9] = {0, USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8};

// USART 中断号表 [uart_num]
const IRQn_Type HW_UsartIrqnTable[9] = {(IRQn_Type) 0, USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn, USART6_IRQn, UART7_IRQn, UART8_IRQn};

// USART 波特率时钟表 [uart_num]
const uint32_t HW_UsartBaudClkTable[9] = {0, USART16CLK, USART234578CLK, USART234578CLK, USART234578CLK, USART234578CLK, USART16CLK, USART234578CLK, USART234578CLK};

// USART RCC 使能表 [uart_num]
const HW_RccEnableInfo HW_UsartRccTable[9] = {
    {0, 0},
    {&RCC->APB2ENR, (1u << 4)},   // USART1
    {&RCC->APB1LENR, (1u << 17)}, // USART2
    {&RCC->APB1LENR, (1u << 18)}, // USART3
    {&RCC->APB1LENR, (1u << 19)}, // UART4
    {&RCC->APB1LENR, (1u << 20)}, // UART5
    {&RCC->APB2ENR, (1u << 5)},   // USART6
    {&RCC->APB1LENR, (1u << 30)}, // UART7
    {&RCC->APB1LENR, (1u << 31)}, // UART8
};

// ==== USART 中断回调系统 ====
static USART_CallbackFunc usartCallBacks[9] = {};

bool USART_CallBackFunc_Register(uint8_t usart_num, USART_CallbackFunc callback)
{
    if (usart_num < 1 || usart_num > 8 || !callback)
        return false;
    if (usartCallBacks[usart_num])
        return false; // 该UART已被占用
    usartCallBacks[usart_num] = callback;
    return true;
}

extern "C" void USART1_IRQHandler()
{
    if (usartCallBacks[1])
        usartCallBacks[1]();
}
extern "C" void USART2_IRQHandler()
{
    if (usartCallBacks[2])
        usartCallBacks[2]();
}
extern "C" void USART3_IRQHandler()
{
    if (usartCallBacks[3])
        usartCallBacks[3]();
}
extern "C" void UART4_IRQHandler()
{
    if (usartCallBacks[4])
        usartCallBacks[4]();
}
extern "C" void UART5_IRQHandler()
{
    if (usartCallBacks[5])
        usartCallBacks[5]();
}
extern "C" void USART6_IRQHandler()
{
    if (usartCallBacks[6])
        usartCallBacks[6]();
}
extern "C" void UART7_IRQHandler()
{
    if (usartCallBacks[7])
        usartCallBacks[7]();
}
extern "C" void UART8_IRQHandler()
{
    if (usartCallBacks[8])
        usartCallBacks[8]();
}

// --- USART ISR 调度表 ---
static UART_CLASS *usartMap[MAX_UART_INDEX] = {};

// --- DMA TX 缓冲区 (AXI DMA RAM) ---
Static_AXIDMABuf uint8_t DmaTxBufs[MAX_UART_INDEX][TxBufferSize];

// ================================================================
// UART 硬件参数查找表 (init函数开头使用)
// ================================================================

// USART_TypeDef* 查找表, 索引=uart_num(1~8), [0]未使用
static USART_TypeDef *const usartTable[MAX_UART_INDEX] = {0, USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8};

// USART IRQn 查找表
static const IRQn_Type usartIrqnTable[MAX_UART_INDEX] = {(IRQn_Type) 0, USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn, USART6_IRQn, UART7_IRQn, UART8_IRQn};

// DMAMUX TX 请求号查找表 (uart_num -> mux value)
static const uint8_t dmamuxTxTable[MAX_UART_INDEX] = {0, 42, 44, 46, 64, 66, 72, 80, 82};

// 波特率时钟查找表
static const uint32_t baudClkTable[MAX_UART_INDEX] = {0, USART16CLK, USART234578CLK, USART234578CLK, USART234578CLK, USART234578CLK, USART16CLK, USART234578CLK, USART234578CLK};

// RCC 时钟使能: [uart_num] -> {寄存器偏移相对RCC基地址, bit位}
// APB2ENR offset = 0xF0, APB1LENR offset = 0xE8
struct RccEnableInfo
{
    volatile uint32_t *reg;
    uint32_t bit;
};
static const RccEnableInfo rccTable[MAX_UART_INDEX] = {
    {0, 0},
    {&RCC->APB2ENR, (1u << 4)},   // USART1
    {&RCC->APB1LENR, (1u << 17)}, // USART2
    {&RCC->APB1LENR, (1u << 18)}, // USART3
    {&RCC->APB1LENR, (1u << 19)}, // UART4
    {&RCC->APB1LENR, (1u << 20)}, // UART5
    {&RCC->APB2ENR, (1u << 5)},   // USART6
    {&RCC->APB1LENR, (1u << 30)}, // UART7
    {&RCC->APB1LENR, (1u << 31)}, // UART8
};

// HW_Device 映射 (uart_num -> HW_DEV_UARTx)
static const uint16_t hwDevTable[MAX_UART_INDEX] = {0, HW_DEV_UART1, HW_DEV_UART2, HW_DEV_UART3, HW_DEV_UART4, HW_DEV_UART5, HW_DEV_UART6, HW_DEV_UART7, HW_DEV_UART8};

// ---- DMA 回调包装 ----
template <int N> static void dmaTxCallback(uint8_t isr, BaseType_t *HigherPriorityTaskWoken)
{
    uart_instances[N].handleDmaIRQ();
}

// ---- USART 中断回调包装 (注册到 HardWareConfig 的 USART_CallBackFunc_Register) ----
template <int N> static void usartIrqCallback()
{
    if (usartMap[N])
        usartMap[N]->handleUsartIRQ();
}

// ================================================================
// UART_CLASS 实现
// ================================================================

bool UART_CLASS::init(uint8_t num)
{
    if (num < 1 || num > 8)
        return false;

    uartNum = num;

    USART_TypeDef *u = usartTable[num];
    if (!u)
        return false;
    usart = u;
    baudClk = baudClkTable[num];
    TxBuffer = DmaTxBufs[num];
    TxCompleted = true;
    parityEnabled = false;
    parityWildcard = 0xFF;

    uint16_t hw_dev = hwDevTable[num];

    // --- 从 InternalStorage 读取硬件配置 ---
    HW_ConfigEntry entry;

    // TX pin (PIN1) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN1, &entry))
        return false;
    uint16_t tx_port = entry.value_high;
    uint16_t tx_pin = entry.value_low;

    // RX pin (PIN2) - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_PIN2, &entry))
        return false;
    uint16_t rx_port = entry.value_high;
    uint16_t rx_pin = entry.value_low;

    // AF - 从 INI 读取（AF1=TX_AF, AF2=RX_AF）
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_AF1, &entry))
        return false;
    uint16_t tx_af = entry.value_low;
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_AF2, &entry))
        return false;
    uint16_t rx_af = entry.value_low;

    // TX DMA - 必须
    if (!HW_ReadConfigEntry(hw_dev, HW_CFG_DMA1, &entry))
        return false;
    uint16_t dma_num = entry.value_high;
    uint16_t stream_num = entry.value_low;

    // OTYPE (可选, 默认 TX=push_pull RX=open_drain)
    uint16_t tx_otype = HW_OTYPE_PUSH_PULL;
    uint16_t rx_otype = HW_OTYPE_OPEN_DRAIN;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_OTYPE1, &entry))
        tx_otype = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_OTYPE2, &entry))
        rx_otype = entry.value_low;

    // PULL (可选, 默认 TX=none RX=pull_up)
    uint16_t tx_pull = HW_PULL_NONE;
    uint16_t rx_pull = HW_PULL_UP;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_PULL1, &entry))
        tx_pull = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_PULL2, &entry))
        rx_pull = entry.value_low;

    // SPEED (可选, 默认 very_high)
    uint16_t tx_speed = HW_SPEED_VERY_HIGH;
    uint16_t rx_speed = HW_SPEED_VERY_HIGH;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_SPEED1, &entry))
        tx_speed = entry.value_low;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_SPEED2, &entry))
        rx_speed = entry.value_low;

    // TR_SWAP (可选)
    bool tr_swap = false;
    if (HW_ReadConfigEntry(hw_dev, HW_CFG_TR_SWAP, &entry))
        tr_swap = (entry.value_low != 0);

    // --- 预先验证范围 (避免在 FreeRTOS 对象已分配后才发现无效配置导致泄漏) ---
    if (tx_port > 7 || rx_port > 7)
        return false;
    if (dma_num < 1 || dma_num > 2 || stream_num > 7)
        return false;

    // --- 创建 FreeRTOS 对象 (逐一检查 NULL, 失败时释放已分配资源) ---
    TxSemphr = xSemaphoreCreateRecursiveMutex();
    if (!TxSemphr)
        return false;
    RxSemphr = xSemaphoreCreateMutex();
    if (!RxSemphr)
    {
        vSemaphoreDelete((SemaphoreHandle_t) TxSemphr);
        TxSemphr = nullptr;
        return false;
    }
    TxStreamBuffer = xTxMessageBufferCreate(TxStreamBufferSize, 1);
    if (!TxStreamBuffer)
    {
        vSemaphoreDelete((SemaphoreHandle_t) TxSemphr);
        TxSemphr = nullptr;
        vSemaphoreDelete((SemaphoreHandle_t) RxSemphr);
        RxSemphr = nullptr;
        return false;
    }
    RxStreamBuffer = xStreamBufferCreate(RxStreamBufferSize, 1);
    if (!RxStreamBuffer)
    {
        vSemaphoreDelete((SemaphoreHandle_t) TxSemphr);
        TxSemphr = nullptr;
        vSemaphoreDelete((SemaphoreHandle_t) RxSemphr);
        RxSemphr = nullptr;
        vStreamBufferDelete((StreamBufferHandle_t) TxStreamBuffer);
        TxStreamBuffer = nullptr;
        return false;
    }
    events = xEventGroupCreate();
    if (!events)
    {
        vSemaphoreDelete((SemaphoreHandle_t) TxSemphr);
        TxSemphr = nullptr;
        vSemaphoreDelete((SemaphoreHandle_t) RxSemphr);
        RxSemphr = nullptr;
        vStreamBufferDelete((StreamBufferHandle_t) TxStreamBuffer);
        TxStreamBuffer = nullptr;
        vStreamBufferDelete((StreamBufferHandle_t) RxStreamBuffer);
        RxStreamBuffer = nullptr;
        return false;
    }

    // --- GPIO 初始化 ---
    GPIO_TypeDef *tx_gpio = HW_GpioTable[tx_port];
    GPIO_TypeDef *rx_gpio = HW_GpioTable[rx_port];

    // 开启 GPIO 时钟
    RCC->AHB4ENR |= (1 << tx_port);
    if (rx_port != tx_port)
        RCC->AHB4ENR |= (1 << rx_port);
    os_delay(1e-2);

    // TX pin
    if (tx_otype == HW_OTYPE_OPEN_DRAIN)
        tx_gpio->OTYPER |= (1 << tx_pin);
    else
        tx_gpio->OTYPER &= ~(1u << tx_pin);
    set_register(tx_gpio->PUPDR, tx_pull, tx_pin * 2, 2);
    set_register(tx_gpio->OSPEEDR, tx_speed, tx_pin * 2, 2);
    if (tx_pin < 8)
        set_register(tx_gpio->AFR[0], (uint32_t) tx_af, tx_pin * 4, 4);
    else
        set_register(tx_gpio->AFR[1], (uint32_t) tx_af, (tx_pin - 8) * 4, 4);
    set_register(tx_gpio->MODER, 0b10, tx_pin * 2, 2);

    // RX pin
    if (rx_otype == HW_OTYPE_OPEN_DRAIN)
        rx_gpio->OTYPER |= (1 << rx_pin);
    else
        rx_gpio->OTYPER &= ~(1u << rx_pin);
    set_register(rx_gpio->PUPDR, rx_pull, rx_pin * 2, 2);
    set_register(rx_gpio->OSPEEDR, rx_speed, rx_pin * 2, 2);
    if (rx_pin < 8)
        set_register(rx_gpio->AFR[0], (uint32_t) rx_af, rx_pin * 4, 4);
    else
        set_register(rx_gpio->AFR[1], (uint32_t) rx_af, (rx_pin - 8) * 4, 4);
    set_register(rx_gpio->MODER, 0b10, rx_pin * 2, 2);

    // --- USART 时钟使能 ---
    if (rccTable[num].reg)
        *rccTable[num].reg |= rccTable[num].bit;
    os_delay(1e-2);

    // --- USART 寄存器配置 ---
    u->CR1 = (1 << 29) | (1 << 26) | (1 << 3) | (1 << 2);
    u->RTOR = 10;
    u->CR2 = (1 << 23) | (tr_swap ? (1 << 15) : 0);
    u->CR3 = (1 << 28) | (0b011 << 25);
    u->CR1 |= (1 << 0);
    setBaudRateInternal(115200);
    NVIC_SetPriority(usartIrqnTable[num], 5);
    NVIC_EnableIRQ(usartIrqnTable[num]);
    xEventGroupSetBits((EventGroupHandle_t) events, (1 << 0));

    // --- DMA 初始化 ---
    DMA_TypeDef *dmaBase = (dma_num == 1) ? DMA1 : DMA2;
    DMA_Stream_TypeDef *dmaStr = (dma_num == 1) ? HW_DMA1Streams[stream_num] : HW_DMA2Streams[stream_num];
    dmaStream = dmaStr;

    // DMA flag clear 信息 (handleDmaIRQ中仍需要，startSend中也需要)
    if (stream_num < 4)
        dmaIfcr = &dmaBase->LIFCR;
    else
        dmaIfcr = &dmaBase->HIFCR;
    dmaIfcrVal = HW_DmaClearBits[stream_num & 3];

    dmaStr->PAR = (uint32_t) &u->TDR;

    // DMAMUX 配置
    uint8_t dmamux_ch_idx = (dma_num - 1) * 8 + stream_num;
    DMAMUX_Channel_TypeDef *dmamux_ch = DMAMUX1_Channel0 + dmamux_ch_idx;
    dmamux_ch->CCR = (dmamuxTxTable[num] << 0);

    dmaStr->CR = (1 << 20) | (0 << 16) | (0 << 13) | (1 << 10) | (0 << 9) | (0b01 << 6) | (1 << 4);
    dmaStr->FCR = (1 << 2) | (3 << 0);
    NVIC_SetPriority(HW_DmaIrqnTable[dma_num - 1][stream_num], 5);
    NVIC_EnableIRQ(HW_DmaIrqnTable[dma_num - 1][stream_num]);

    // --- 注册 DMA 回调 ---
    DMA_CallbackFunc dmaCallback = 0;
    switch (num)
    {
        case 1:
            dmaCallback = dmaTxCallback<1>;
            break;
        case 2:
            dmaCallback = dmaTxCallback<2>;
            break;
        case 3:
            dmaCallback = dmaTxCallback<3>;
            break;
        case 4:
            dmaCallback = dmaTxCallback<4>;
            break;
        case 5:
            dmaCallback = dmaTxCallback<5>;
            break;
        case 6:
            dmaCallback = dmaTxCallback<6>;
            break;
        case 7:
            dmaCallback = dmaTxCallback<7>;
            break;
        case 8:
            dmaCallback = dmaTxCallback<8>;
            break;
    }
    DMA_CallBackFunc_Register(dma_num, stream_num, dmaCallback);

    // --- 注册 USART 中断回调 ---
    USART_CallbackFunc usartCb = 0;
    switch (num)
    {
        case 1:
            usartCb = usartIrqCallback<1>;
            break;
        case 2:
            usartCb = usartIrqCallback<2>;
            break;
        case 3:
            usartCb = usartIrqCallback<3>;
            break;
        case 4:
            usartCb = usartIrqCallback<4>;
            break;
        case 5:
            usartCb = usartIrqCallback<5>;
            break;
        case 6:
            usartCb = usartIrqCallback<6>;
            break;
        case 7:
            usartCb = usartIrqCallback<7>;
            break;
        case 8:
            usartCb = usartIrqCallback<8>;
            break;
    }
    USART_CallBackFunc_Register(num, usartCb);

    // --- 注册到 USART ISR 调度表 ---
    usartMap[num] = this;

    return true;
}

void UART_CLASS::startSend(const uint8_t *buf, uint16_t len)
{
    USART_TypeDef *u = (USART_TypeDef *) usart;
    DMA_Stream_TypeDef *dmaStr = (DMA_Stream_TypeDef *) dmaStream;

    TxCompleted = false;
    xEventGroupClearBits((EventGroupHandle_t) events, (1 << 0));
    // 关闭串口DMA发送
    u->CR3 &= ~(1 << 7);
    u->CR1 &= ~(1 << 6);
    // 清空DMA状态
    *dmaIfcr = dmaIfcrVal;
    // 设置DMA存储器地址
    dmaStr->M0AR = (uint32_t) buf;
    // 设置DMA传输数量
    dmaStr->NDTR = len;
    // 使能DMA
    dmaStr->CR |= (1 << 0);
    // 清空Uart状态
    u->ICR = (1 << 6);
    // 打开串口DMA发送
    u->CR3 |= (1 << 7);
}

void UART_CLASS::setBaudRateInternal(uint32_t baud)
{
    USART_TypeDef *u = (USART_TypeDef *) usart;
    u->CR1 &= ~(1 << 3);
    u->CR1 &= ~(1 << 0);
    u->BRR = baudClk / baud;
    u->CR1 |= (1 << 3) | (1 << 0);
}

void UART_CLASS::txCallbackWrapper(void *pvParameter1, uint32_t ulParameter2)
{
    UART_CLASS *inst = (UART_CLASS *) pvParameter1;
    uint16_t length = xStreamBufferReceive((StreamBufferHandle_t) inst->TxStreamBuffer, inst->TxBuffer, TxBufferSize, 0);
    if (length > 0)
        inst->startSend(inst->TxBuffer, length);
    else
    {
        inst->TxCompleted = true;
        xEventGroupSetBits((EventGroupHandle_t) inst->events, (1 << 0));
    }
}

void UART_CLASS::handleDmaIRQ()
{
    // DMA标志已由dma.cpp清除
    // 打开串口TC中断
    USART_TypeDef *u = (USART_TypeDef *) usart;
    u->CR1 |= (1 << 6);
}

void UART_CLASS::handleUsartIRQ()
{
    USART_TypeDef *u = (USART_TypeDef *) usart;

    if (u->ISR & (1 << 3))
    { // 接收fifo溢出
        u->ICR = 1 << 3;
    }

    BaseType_t HigherPriorityTaskWoken = pdFALSE;
    if ((u->ISR & (1 << 11)) || (u->ISR & (1 << 26)))
    { // 接收中断
        u->ICR = 1 << 11;

        uint8_t buf[36];
        uint8_t len = 0;
        while ((u->ISR & (1 << 5)) != 0 && len < 34)
        {
            uint32_t isr = u->ISR;
            uint8_t rdata = u->RDR;
            if (parityEnabled)
            {
                bool pe = (isr & (1 << 0)) != 0; // PE flag
                if (pe)
                {
                    u->ICR = (1 << 0); // 清除PE标志
                    buf[len++] = parityWildcard;
                    buf[len++] = UART_PARITY_ESC_ERROR;
                }
                else if (rdata == parityWildcard)
                {
                    buf[len++] = parityWildcard;
                    buf[len++] = UART_PARITY_ESC_LITERAL;
                }
                else
                {
                    buf[len++] = rdata;
                }
            }
            else
            {
                buf[len++] = rdata;
            }
        }
        if (len > 0)
            xStreamBufferSendFromISR((StreamBufferHandle_t) RxStreamBuffer, buf, len, &HigherPriorityTaskWoken);
    }
    if (u->ISR & (1 << 6))
    { // 发送完成中断
        u->CR3 &= ~(1 << 7);
        u->CR1 &= ~(1 << 6);
        u->ICR = (1 << 6);
        BaseType_t res = xTimerPendFunctionCallFromISR(txCallbackWrapper, (void *) this, 0, &HigherPriorityTaskWoken);
        if (res != pdPASS)
            TxCompleted = true;
    }
    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

// ================================================================
// 公开接口方法
// ================================================================

uint16_t UART_CLASS::Read(uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    uint32_t Rc_waitTicks;
    if (Rc_waitTime >= 0)
        Rc_waitTicks = Rc_waitTime * configTICK_RATE_HZ;
    else
        Rc_waitTicks = portMAX_DELAY;

    if (xSemaphoreTake((SemaphoreHandle_t) RxSemphr, Sync_waitTicks) == pdTRUE)
    {
        uint16_t rc_length = xStreamBufferReceive((StreamBufferHandle_t) RxStreamBuffer, data, length, Rc_waitTicks);
        xSemaphoreGive((SemaphoreHandle_t) RxSemphr);
        return rc_length;
    }
    return 0;
}

uint16_t UART_CLASS::Write(const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime)
{
    if (length == 0)
        return 0;

    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    uint32_t Send_waitTicks;
    if (Send_waitTime >= 0)
        Send_waitTicks = Send_waitTime * configTICK_RATE_HZ;
    else
        Send_waitTicks = portMAX_DELAY;

    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) TxSemphr, Sync_waitTicks) == pdTRUE)
    {
        uint16_t data_sent = xStreamBufferSend((StreamBufferHandle_t) TxStreamBuffer, data, length, Send_waitTicks);
        if (TxCompleted)
        {
            TxCompleted = false;
            xTimerPendFunctionCall(txCallbackWrapper, (void *) this, 0, 0);
        }
        xSemaphoreGiveRecursive((SemaphoreHandle_t) TxSemphr);
        return data_sent;
    }
    return 0;
}

bool UART_CLASS::Lock(double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime > 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) TxSemphr, Sync_waitTicks) == pdTRUE)
        return true;
    return false;
}

void UART_CLASS::Unlock()
{
    xSemaphoreGiveRecursive((SemaphoreHandle_t) TxSemphr);
}

bool UART_CLASS::ResetRx(double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    if (xSemaphoreTake((SemaphoreHandle_t) RxSemphr, Sync_waitTicks) == pdTRUE)
    {
        xStreamBufferReset((StreamBufferHandle_t) RxStreamBuffer);
        xSemaphoreGive((SemaphoreHandle_t) RxSemphr);
        return true;
    }
    return false;
}

bool UART_CLASS::WaitSent(double waitTime)
{
    uint32_t waitTicks;
    if (waitTime >= 0)
        waitTicks = waitTime * configTICK_RATE_HZ;
    else
        waitTicks = portMAX_DELAY;
    EventBits_t rtbits = xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 0), pdFALSE, pdTRUE, waitTicks);
    if (rtbits == (1 << 0))
        return true;
    return false;
}

bool UART_CLASS::SetBaudRate(uint32_t baud_rate, double Send_waitTime, double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    uint32_t Send_waitTicks;
    if (Send_waitTime >= 0)
        Send_waitTicks = Send_waitTime * configTICK_RATE_HZ;
    else
        Send_waitTicks = portMAX_DELAY;

    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) TxSemphr, Sync_waitTicks) == pdTRUE)
    {
        EventBits_t rtbits = xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 0), pdFALSE, pdTRUE, Send_waitTicks);
        bool result = false;
        if (rtbits == (1 << 0))
        {
            setBaudRateInternal(baud_rate);
            result = true;
        }
        xSemaphoreGiveRecursive((SemaphoreHandle_t) TxSemphr);
        return result;
    }
    return false;
}

uint16_t UART_CLASS::TxSpacesAvailable()
{
    return xStreamBufferSpacesAvailable((StreamBufferHandle_t) TxStreamBuffer);
}

bool UART_CLASS::SetTxInvert(bool invert)
{
    USART_TypeDef *u = (USART_TypeDef *) usart;
    bool current = (u->CR2 & (1u << 17)) != 0;
    if (current == invert)
        return true;
    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) TxSemphr, portMAX_DELAY) == pdTRUE)
    {
        xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 0), pdFALSE, pdTRUE, portMAX_DELAY);
        u->CR1 &= ~(1 << 0);
        if (invert)
            u->CR2 |= (1u << 17);
        else
            u->CR2 &= ~(1u << 17);
        u->CR1 |= (1 << 0);
        xSemaphoreGiveRecursive((SemaphoreHandle_t) TxSemphr);
        return true;
    }
    return false;
}

bool UART_CLASS::SetRxInvert(bool invert)
{
    USART_TypeDef *u = (USART_TypeDef *) usart;
    bool current = (u->CR2 & (1u << 16)) != 0;
    if (current == invert)
        return true;
    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) TxSemphr, portMAX_DELAY) == pdTRUE)
    {
        xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 0), pdFALSE, pdTRUE, portMAX_DELAY);
        u->CR1 &= ~(1 << 0);
        if (invert)
            u->CR2 |= (1u << 16);
        else
            u->CR2 &= ~(1u << 16);
        u->CR1 |= (1 << 0);
        xSemaphoreGiveRecursive((SemaphoreHandle_t) TxSemphr);
        return true;
    }
    return false;
}

bool UART_CLASS::SetParity(UartParityMode parity_mode, uint8_t wildcard_byte, double Send_waitTime, double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    uint32_t Send_waitTicks;
    if (Send_waitTime >= 0)
        Send_waitTicks = Send_waitTime * configTICK_RATE_HZ;
    else
        Send_waitTicks = portMAX_DELAY;

    if (xSemaphoreTakeRecursive((SemaphoreHandle_t) TxSemphr, Sync_waitTicks) == pdTRUE)
    {
        EventBits_t rtbits = xEventGroupWaitBits((EventGroupHandle_t) events, (1 << 0), pdFALSE, pdTRUE, Send_waitTicks);
        bool result = false;
        if (rtbits == (1 << 0))
        {
            USART_TypeDef *u = (USART_TypeDef *) usart;
            // 关闭USART
            u->CR1 &= ~(1 << 0);

            if (parity_mode == UartParity_None)
            {
                // 关闭奇偶校验: PCE=0, M0=0 (8位数据)
                u->CR1 &= ~((1u << 10) | (1u << 12));
                parityEnabled = false;
            }
            else
            {
                // 开启奇偶校验: PCE=1, M0=1 (9位字长=8数据+1校验)
                u->CR1 |= (1u << 10) | (1u << 12);
                if (parity_mode == UartParity_Even)
                    u->CR1 &= ~(1u << 9); // PS=0 偶校验
                else
                    u->CR1 |= (1u << 9); // PS=1 奇校验
                parityWildcard = wildcard_byte;
                parityEnabled = true;
            }

            // 重新使能USART
            u->CR1 |= (1 << 0);
            result = true;
        }
        xSemaphoreGiveRecursive((SemaphoreHandle_t) TxSemphr);
        return result;
    }
    return false;
}

// ================================================================
// 初始化接口
// ================================================================

// ---- Port 注册辅助 (模板生成唯一函数指针) ----
template <int N> static uint16_t UartWrite(const uint8_t *d, uint16_t l, double s, double w)
{
    return uart_instances[N].Write(d, l, s, w);
}
template <int N> static uint16_t UartRead(uint8_t *d, uint16_t l, double r, double s)
{
    return uart_instances[N].Read(d, l, r, s);
}
template <int N> static bool UartLock(double s)
{
    return uart_instances[N].Lock(s);
}
template <int N> static void UartUnlock()
{
    uart_instances[N].Unlock();
}
template <int N> static bool UartResetRx(double s)
{
    return uart_instances[N].ResetRx(s);
}
template <int N> static bool UartWaitSent(double w)
{
    return uart_instances[N].WaitSent(w);
}
template <int N> static bool UartSetBaud(uint32_t b, double s, double w)
{
    return uart_instances[N].SetBaudRate(b, s, w);
}
template <int N> static uint16_t UartTxSpaces()
{
    return uart_instances[N].TxSpacesAvailable();
}
template <int N> static bool UartSetTxInvert(bool inv)
{
    return uart_instances[N].SetTxInvert(inv);
}
template <int N> static bool UartSetRxInvert(bool inv)
{
    return uart_instances[N].SetRxInvert(inv);
}
template <int N> static bool UartSetParity(UartParityMode m, uint8_t w, double s, double t)
{
    return uart_instances[N].SetParity(m, w, s, t);
}

#define REGISTER_UART_PORT_CASE(N)                                                                                                                                                                     \
    case N:                                                                                                                                                                                            \
        p.write = UartWrite<N>;                                                                                                                                                                        \
        p.read = UartRead<N>;                                                                                                                                                                          \
        p.lock = UartLock<N>;                                                                                                                                                                          \
        p.unlock = UartUnlock<N>;                                                                                                                                                                      \
        p.reset_rx = UartResetRx<N>;                                                                                                                                                                   \
        p.wait_sent = UartWaitSent<N>;                                                                                                                                                                 \
        p.SetBaudRate = UartSetBaud<N>;                                                                                                                                                                \
        p.txSpacesAvailable = UartTxSpaces<N>;                                                                                                                                                         \
        p.setTxInvert = UartSetTxInvert<N>;                                                                                                                                                            \
        p.setRxInvert = UartSetRxInvert<N>;                                                                                                                                                            \
        p.SetParity = UartSetParity<N>;                                                                                                                                                                \
        break;

void init_drv_uart()
{
    // 在硬件配置文件中查找每个 UART 端口的配置，成功找到并初始化后注册到 PortManager
    for (uint8_t num = 1; num <= 8; num++)
    {
        HW_ConfigEntry entry;
        if (!HW_ReadConfigEntry(hwDevTable[num], HW_CFG_PIN1, &entry))
            continue;
        if (!uart_instances[num].init(num))
            continue;

        uart_inited[num] = true;

        // 注册端口
        Port p = {};
        switch (num)
        {
            REGISTER_UART_PORT_CASE(1)
            REGISTER_UART_PORT_CASE(2)
            REGISTER_UART_PORT_CASE(3)
            REGISTER_UART_PORT_CASE(4)
            REGISTER_UART_PORT_CASE(5)
            REGISTER_UART_PORT_CASE(6)
            REGISTER_UART_PORT_CASE(7)
            REGISTER_UART_PORT_CASE(8)
            default:
                continue;
        }
        PortRegister(num, p);
    }
}

UART_CLASS *get_uart_instance(uint8_t uart_num)
{
    if (uart_num >= 1 && uart_num < MAX_UART_INDEX && uart_inited[uart_num])
        return &uart_instances[uart_num];
    return 0;
}
