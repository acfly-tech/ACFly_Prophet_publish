#pragma once

#include "Basic.hpp"
#include "FreeRTOS.h"
#include <stdint.h>

// DMA 流表
extern DMA_Stream_TypeDef *const HW_DMA1Streams[8];
extern DMA_Stream_TypeDef *const HW_DMA2Streams[8];

// DMA 流中断号表 [dma_idx(0/1)][stream]
extern const IRQn_Type HW_DmaIrqnTable[2][8];

// DMA 标志清除位模式 [stream & 3]
extern const uint32_t HW_DmaClearBits[4];

// 清空DMA中断标志位
// dma_num:    DMA控制器号 1=DMA1  2=DMA2
// stream_num: DMA流号 0~7
void clear_DMA_flags(uint8_t dma_num, uint8_t stream_num);

// DMA 回调函数类型
// isr: bit0~bit6
// HigherPriorityTaskWoken: 供ISR中FreeRTOS API使用
typedef void (*DMA_CallbackFunc)(uint8_t isr, BaseType_t *HigherPriorityTaskWoken);

// 注册 DMA 通道回调函数，DMA中断触发时自动清除标志位并调用回调
// dma_num:    DMA控制器号 1=DMA1  2=DMA2
// stream_num: DMA流号 0~7
// callback:   回调函数
// 返回: true=注册成功  false=该通道已被占用
bool DMA_CallBackFunc_Register(uint8_t dma_num, uint8_t stream_num, DMA_CallbackFunc callback);

// MDMA 通道表 [0..15]
extern MDMA_Channel_TypeDef *const HW_MDMAChannels[16];

// MDMA 回调函数类型 (与DMA回调类型一致)
// isr: 本通道 CISR 寄存器原始值 (位定义见 MDMA_CISR_*)
// HigherPriorityTaskWoken: 供ISR中FreeRTOS API使用
typedef DMA_CallbackFunc MDMA_CallbackFunc;

// 注册 MDMA 通道回调函数，MDMA中断触发时由dma.cpp统一的中断处理分派
//   (H743 MDMA 共用单一 MDMA_IRQn)；中断处理会自动清除本通道的全部CISR标志位
// channel_num: MDMA通道号 0~15
// callback:   回调函数
// 返回: true=注册成功  false=该通道已被占用
bool MDMA_CallBackFunc_Register(uint8_t channel_num, MDMA_CallbackFunc callback);

// 初始化DMA中断系统 (使能DMA1/DMA2时钟)
void init_DMA(void);
