/**
 * @file    serial.h
 * @brief   串口通信模块 — 格式化发送 + DMA 空闲接收 + HMI 协议
 *
 * 物理通道：
 *   USART1 (PB14/PB15, 921600) — PC 调试 / Vofa 上报
 *   USART3 (PB10/PB11, 9600)   — HMI 串口屏
 *
 * 接收链路:
 *   硬件 DMA → uartX_rx_buf → IDLE 中断
 *     → HAL_UARTEx_RxEventCallback() 只设 flag+len
 *     → 主循环轮询 flag → D-cache 失效 → 读 buf → 清 flag
 *
 * HMI 帧格式: 控件名="值"\xff\xff\xff
 */

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32h7xx_hal.h"
#include "app_types.h"

/* ================================================================
 * 一、格式化发送
 * ================================================================ */

void Serial_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);

/* ================================================================
 * 二、DMA 空闲接收
 * ================================================================ */

void Serial_RxInit(UART_HandleTypeDef *huart);
void Serial_RxOnIdle(UART_HandleTypeDef *huart);

/* ---- DMA 接收缓冲区 ---- */
#define UART_RX_BUF_SIZE  128

extern uint8_t  uart1_rx_buf[UART_RX_BUF_SIZE];
extern uint16_t uart1_rx_len;
extern uint8_t  uart1_rx_flag;

extern uint8_t  uart3_rx_buf[UART_RX_BUF_SIZE];
extern uint16_t uart3_rx_len;
extern uint8_t  uart3_rx_flag;

/* ================================================================
 * 三、HMI 串口屏协议 — 薄封装（帧尾 \xff\xff\xff，写到 uart3_tx_buf）
 * ================================================================ */

void HMI_SendStr(const char *ctl, const char *text);
void HMI_SendInt(const char *ctl, int num);
void HMI_SendFloat(const char *ctl, float num, int decimals);
void HMI_WaveAdd(const char *ctl, int ch, int val);
void HMI_WaveClear(const char *ctl, int ch);

void UART1_Printf(const char *format,...);
void UART3_Printf(const char *format,...);

#endif
