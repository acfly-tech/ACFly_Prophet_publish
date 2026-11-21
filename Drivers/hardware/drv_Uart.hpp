#pragma once

#include "Basic.hpp"
#include "HardWareConfig.hpp"
#include <stdint.h>

// 奇偶校验模式
enum UartParityMode : uint8_t
{
    UartParity_None = 0,
    UartParity_Odd = 1,
    UartParity_Even = 2,
};

// 通配头转义协议：
// 开启奇偶校验后，ISR将按以下规则写入接收缓冲区：
// - 奇偶校验错误：写入 [wildcard_byte, 0x01]
// - 正常数据且数据==wildcard_byte：写入 [wildcard_byte, 0xFF]
// - 正常数据且数据!=wildcard_byte：直接写入数据
#define UART_PARITY_ESC_ERROR 0x01
#define UART_PARITY_ESC_LITERAL 0xFF

// USART 外设指针表 [uart_num], 索引 1~8, [0]=0
extern USART_TypeDef *const HW_UsartTable[9];

// USART 中断号表 [uart_num], 索引 1~8
extern const IRQn_Type HW_UsartIrqnTable[9];

// USART 波特率时钟表 [uart_num], 索引 1~8
extern const uint32_t HW_UsartBaudClkTable[9];

// USART RCC 使能信息
extern const HW_RccEnableInfo HW_UsartRccTable[9];

// ==== USART 中断回调系统 (类似DMA回调) ====

// USART 回调函数类型
typedef void (*USART_CallbackFunc)(void);

// 注册 USART 中断回调函数
// usart_num: UART编号 1~8
// callback:  回调函数 (在ISR上下文中被调用)
// 返回: true=注册成功  false=编号无效或该UART已被占用
bool USART_CallBackFunc_Register(uint8_t usart_num, USART_CallbackFunc callback);

class UART_CLASS
{
  public:
    bool init(uint8_t uart_num);

    uint16_t Read(uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime);
    uint16_t Write(const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime);
    bool Lock(double Sync_waitTime);
    void Unlock();
    bool ResetRx(double Sync_waitTime);
    bool WaitSent(double waitTime);
    bool SetBaudRate(uint32_t baud_rate, double Send_waitTime, double Sync_waitTime);
    uint16_t TxSpacesAvailable();
    bool SetTxInvert(bool invert);
    bool SetRxInvert(bool invert);
    bool SetParity(UartParityMode parity_mode, uint8_t wildcard_byte, double Send_waitTime, double Sync_waitTime);

    // ISR回调 (由DMA回调和USART向量入口调用)
    void handleDmaIRQ();
    void handleUsartIRQ();

  private:
    void startSend(const uint8_t *buf, uint16_t len);
    void setBaudRateInternal(uint32_t baud);
    static void txCallbackWrapper(void *pvParameter1, uint32_t ulParameter2);

    void *usart;     // USART_TypeDef*
    void *dmaStream; // DMA_Stream_TypeDef*
    volatile uint32_t *dmaIfcr;
    uint32_t dmaIfcrVal;
    uint32_t baudClk;
    uint8_t uartNum;

    void *TxSemphr;
    void *RxSemphr;
    void *TxStreamBuffer;
    void *RxStreamBuffer;
    void *events;
    volatile bool TxCompleted;
    uint8_t *TxBuffer;
    volatile bool parityEnabled;
    volatile uint8_t parityWildcard;
};

// 支持的UART编号: 1~8 (USART1,USART2,USART3,UART4,UART5,USART6,UART7,UART8)

// 根据HardWare配置初始化所有串口设备
void init_drv_uart();

// 获取指定串口号的UART_CLASS实例
// 未初始化的返回nullptr
UART_CLASS *get_uart_instance(uint8_t uart_num);
