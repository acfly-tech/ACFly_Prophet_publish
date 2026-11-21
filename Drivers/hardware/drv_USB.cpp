#include "drv_USB.hpp"
#include "Basic.hpp"
#include "CommuLink.hpp"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "stm32h7xx_hal.h"
#include "stream_buffer.h"

#define USB_BUS_ID 0
#define GLOBAL_BASE(_port) ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)

// 发送互斥锁
static SemaphoreHandle_t TxSemphr;
// USB发送消息缓冲区
#define TxBufferSize 2048
#define TxStreamBufferSize 1024

Static_AXIDMABuf uint8_t TxBuffer[TxBufferSize];
static StreamBufferHandle_t TxStreamBuffer;
static volatile bool TxCompleted = true;
// 发送超时检测：记录 TxCompleted 变为 false 的时间
static volatile TickType_t TxStartTime = 0;
// 超时阈值以tick为单位的换算（避免整除精度损失）
#define TX_TIMEOUT_TICKS pdMS_TO_TICKS(2000)
// 发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();
// 接收互斥锁
static SemaphoreHandle_t RxSemphr;
// 接收缓冲区
#define RxStreamBufferSize 1024
static StreamBufferHandle_t RxStreamBuffer;

extern "C" void usbd_vcom_resetCallBack()
{
    TxCompleted = true;
    // 重置时确保 events 位已置位，避免 WaitSent 调用方永久阻塞
    if (xPortIsInsideInterrupt())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(events, (1 << 0), &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        xEventGroupSetBits(events, (1 << 0));
    }
}

extern "C" void usbd_vcom_rxCallBack(uint8_t *buffer, uint32_t nbytes)
{
    if (xPortIsInsideInterrupt())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        int ret = pdFAIL;
        if (nbytes)
            ret = xStreamBufferSendFromISR(RxStreamBuffer, buffer, nbytes, &xHigherPriorityTaskWoken);
        if (ret == pdPASS)
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        if (nbytes)
            xStreamBufferSend(RxStreamBuffer, buffer, nbytes, 0);
    }
}

/*串口接收函数（将要发送的数据压入缓冲区）
  data:接收数据指针
  length:要接收的数据长度（字节）
  Sync_waitTime:线程同步最大等待时间（s）
  Rc_waitTime:等待数据的最大等待时间（s）
  返回值：实际接收到的字节数
  解释:如果在指定等待时间内接收不到足够数据（接收缓冲区里没这么多数据）就接收尽量多的数据
*/
uint16_t Read_USBD_VCOM(uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime)
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
    // 获取信号量
    if (xSemaphoreTake(RxSemphr, Sync_waitTicks) == pdTRUE)
    {
        uint16_t rc_length = xStreamBufferReceive(RxStreamBuffer, data, length, Rc_waitTicks);

        // 释放信号量
        xSemaphoreGive(RxSemphr);
        return rc_length;
    }
    else
        return 0;
}

extern "C" int cdc_startSend(const uint8_t *write_buffer, uint32_t len);

static void TxTCB(void *pvParameter1, uint32_t ulParameter2)
{
    uint16_t length = xStreamBufferReceive(TxStreamBuffer, TxBuffer, TxBufferSize, 0);
    if (length > 0)
    {
        // 如果底层的 cdc_startSend 会受中断影响，保持临界区保护
        NVIC_DisableIRQ(OTG_FS_IRQn);
        int res = cdc_startSend(TxBuffer, length);
        if (res != 0)
        {
            xEventGroupSetBits(events, (1 << 0));
            TxCompleted = true;
        }
        else
        {
            // 发送启动成功，清除events位使WaitSent_USBD_VCOM阻塞直到发送完成
            xEventGroupClearBits(events, (1 << 0));
            TxCompleted = false;
            TxStartTime = xTaskGetTickCount();
        }
        NVIC_EnableIRQ(OTG_FS_IRQn);
    }
    else
    {
        xEventGroupSetBits(events, (1 << 0));
        TxCompleted = true;
    }
}

/*  串口发送函数（将要发送的数据压入缓冲区）
    data:要发送的数据指针
    length:要发送的数据长度（字节）
    Sync_waitTime:线程同步最大等待时间（s）
    Send_waitTime:等待缓冲区有空间的最大等待时间（s）
    返回值：实际发送的字节数
    解释：如果在指定等待时间内接收不到足够数据（发送缓冲区位置不足）将只发送前半段数据（只将前半段数据压入缓冲区）
*/
extern "C" bool usb_device_is_configured(uint8_t busid);
uint16_t Write_USBD_VCOM(const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime)
{
    if (length == 0)
        return 0;

    // Skip write when USB is not connected to avoid blocking callers
    if (!usb_device_is_configured(0))
        return length;

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
    // 获取信号量
    if (xSemaphoreTakeRecursive(TxSemphr, Sync_waitTicks) == pdTRUE)
    {
        uint16_t data_sent = xStreamBufferSend(TxStreamBuffer, data, length, Send_waitTicks);

        // 超时恢复：如果 TxCompleted 长时间为 false，说明 USB 回调丢失
        if (!TxCompleted)
        {
            if ((xTaskGetTickCount() - TxStartTime) > TX_TIMEOUT_TICKS)
            {
                TxCompleted = true;
                xEventGroupSetBits(events, (1 << 0));
            }
        }

        if (TxCompleted && usb_device_is_configured(0))
        {
            // USB空闲可发送：投递 TxTCB 到 timer daemon 任务统一执行
            // 保证 TxStreamBuffer 的唯一消费者始终是 timer daemon（单任务上下文）
            // TxCompleted 由 TxTCB 内部根据 cdc_startSend 结果自行设置
            // 若定时器队列满则置events位，数据留在TxStreamBuffer等下次调度（避免永久卡死）
            BaseType_t res = xTimerPendFunctionCall(TxTCB, NULL, 0, 0);
            if (res != pdPASS)
            {
                xEventGroupSetBits(events, (1 << 0));
            }
        }

        // 释放信号量
        xSemaphoreGiveRecursive(TxSemphr);
        return data_sent;
    }
    else
        return 0;
}

// USB 发送完成中断回调
extern "C" void usbd_vcom_txCallBack()
{
    if (xPortIsInsideInterrupt())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t res = xTimerPendFunctionCallFromISR(TxTCB, NULL, 0, &xHigherPriorityTaskWoken);
        if (res != pdPASS)
        {
            // timer命令队列满时，标记完成并置events位，避免WaitSent永久阻塞
            TxCompleted = true;
            xEventGroupSetBitsFromISR(events, (1 << 0), &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        BaseType_t res = xTimerPendFunctionCall(TxTCB, NULL, 0, 0);
        if (res != pdPASS)
        {
            TxCompleted = true;
            xEventGroupSetBits(events, (1 << 0));
        }
    }
}

/*发送上锁
  上锁保证数据连续性
  上锁之后必须解锁
*/
bool Lock_USBD_VCOM(double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime > 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    if (xSemaphoreTakeRecursive(TxSemphr, Sync_waitTicks) == pdTRUE)
        return true;
    return false;
}
void Unlock_USBD_VCOM()
{
    xSemaphoreGiveRecursive(TxSemphr);
}

/*清空接收缓冲区
        Sync_waitTime:线程同步最大等待时间（s）
*/
bool ResetRx_USBD_VCOM(double Sync_waitTime)
{
    uint32_t Sync_waitTicks;
    if (Sync_waitTime >= 0)
        Sync_waitTicks = Sync_waitTime * configTICK_RATE_HZ;
    else
        Sync_waitTicks = portMAX_DELAY;
    // 获取信号量
    if (xSemaphoreTake(RxSemphr, Sync_waitTicks) == pdTRUE)
    {
        xStreamBufferReset(RxStreamBuffer);
        xSemaphoreGive(RxSemphr);
        return true;
    }
    return false;
}

/*等待发送完成
        waitTime:最大等待时间（s）
*/
bool WaitSent_USBD_VCOM(double waitTime)
{
    uint32_t waitTicks;
    if (waitTime >= 0)
        waitTicks = waitTime * configTICK_RATE_HZ;
    else
        waitTicks = portMAX_DELAY;
    EventBits_t rtbits = xEventGroupWaitBits(events, (1 << 0), pdFALSE, pdTRUE, waitTicks);
    if (rtbits == (1 << 0))
        return true;
    return false;
}

extern "C" void usbd_vcom_configuredCallBack()
{
    TxCompleted = true;
    // USB重新枚举完成后，主动唤醒 TxTCB 以排空缓冲区中残留数据
    // 统一通过 timer daemon 任务执行 TxTCB，保证 TxStreamBuffer 单消费者
    if (xPortIsInsideInterrupt())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t res = xTimerPendFunctionCallFromISR(TxTCB, NULL, 0, &xHigherPriorityTaskWoken);
        if (res != pdPASS)
        {
            // timer命令队列满时，置events位，避免WaitSent永久阻塞
            xEventGroupSetBitsFromISR(events, (1 << 0), &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        BaseType_t res = xTimerPendFunctionCall(TxTCB, NULL, 0, 0);
        if (res != pdPASS)
            xEventGroupSetBits(events, (1 << 0));
    }
}

// 获取缓冲区剩余空间
uint16_t USBD_VCOM_TxSpacesAvailable()
{
    return xStreamBufferSpacesAvailable(TxStreamBuffer);
}

extern "C" void cdc_acm_hid_msc_descriptor_init(uint8_t busid, uint32_t reg_base, uint8_t id);
void init_drv_USB()
{
    // 注册互斥锁
    TxSemphr = xSemaphoreCreateRecursiveMutex();
    RxSemphr = xSemaphoreCreateMutex();
    // 发送缓冲区
    TxStreamBuffer = xTxMessageBufferCreate(TxStreamBufferSize, 1);
    // 接收缓冲区
    RxStreamBuffer = xStreamBufferCreate(RxStreamBufferSize, 1);
    xEventGroupSetBits(events, (1 << 0));

    // 注册端口
    Port USBDVCOM;
    USBDVCOM.write = Write_USBD_VCOM;
    USBDVCOM.wait_sent = 0;
    USBDVCOM.lock = Lock_USBD_VCOM;
    USBDVCOM.unlock = Unlock_USBD_VCOM;
    USBDVCOM.read = Read_USBD_VCOM;
    USBDVCOM.reset_rx = ResetRx_USBD_VCOM;
    USBDVCOM.SetBaudRate = 0;
    USBDVCOM.txSpacesAvailable = USBD_VCOM_TxSpacesAvailable;
    PortRegister(MAXPorts - 1, USBDVCOM);
}
extern "C" void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    // 开启usb中断
    NVIC_SetPriority(OTG_FS_IRQn, IRQPrioriy_USB);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void start_USB(uint8_t id)
{
    cdc_acm_hid_msc_descriptor_init(USB_BUS_ID, USB_OTG_FS_PERIPH_BASE, id);
}

TIME usbLastIRQTIME(false);
extern "C" bool get_is_usb_connected()
{
    return usbLastIRQTIME.get_pass_time() < 2;
}

extern "C" void usbIRQ_CallBack(uint8_t busid)
{
    usbLastIRQTIME = TIME::now();
}

// USB中断
// extern "C" void OTG_FS_IRQHandler(void)
//{
//	void usbIRQ_CallBack(uint8_t busid);
//	usbIRQ_CallBack(USB_BUS_ID);
//
//	extern void USBD_IRQHandler(uint8_t busid);
//	USBD_IRQHandler(USB_BUS_ID);
// }
