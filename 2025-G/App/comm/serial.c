/**
 * @file    serial.c
 * @brief   串口通信模块 — DMA 空闲接收 + 格式化发送 + HMI 协议
 */

#include "serial.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ---- DMA 句柄（定义在 usart.c）---- */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ================================================================
 * 一、格式化发送（直接发送，带错误恢复）
 * ================================================================ */

void Serial_Printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len <= 0) return;
    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;

    HAL_StatusTypeDef rc = HAL_UART_Transmit(huart, (uint8_t *)buf, (uint16_t)len, 50);

    /* 发送失败时清错误标志 + 复位 HAL 状态 */
    if (rc != HAL_OK) {
        uint32_t isr = READ_REG(huart->Instance->ISR);
        if (isr & USART_ISR_ORE) SET_BIT(huart->Instance->ICR, USART_ICR_ORECF);
        if (isr & USART_ISR_FE)  SET_BIT(huart->Instance->ICR, USART_ICR_FECF);
        if (isr & USART_ISR_NE)  SET_BIT(huart->Instance->ICR, USART_ICR_NECF);
        if (isr & USART_ISR_PE)  SET_BIT(huart->Instance->ICR, USART_ICR_PECF);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->gState    = HAL_UART_STATE_READY;
        huart->RxState   = HAL_UART_STATE_READY;
    }
}

/* ================================================================
 * 二、DMA 空闲接收
 * ================================================================ */

uint8_t  uart1_rx_buf[UART_RX_BUF_SIZE];
uint16_t uart1_rx_len;
uint8_t  uart1_rx_flag;

uint8_t  uart3_rx_buf[UART_RX_BUF_SIZE];
uint16_t uart3_rx_len;
uint8_t  uart3_rx_flag;

void Serial_RxInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_rx_buf, UART_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    } else if (huart->Instance == USART3) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_rx_buf, UART_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}

void Serial_RxOnIdle(UART_HandleTypeDef *huart)
{
    (void)huart;
}

/* ---- HAL DMA 空闲接收回调 — 只置标志 ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        if (uart1_rx_flag == 0) {
            uart1_rx_len  = Size;
            uart1_rx_flag = 1;
        }
    } else if (huart->Instance == USART3) {
        if (uart3_rx_flag == 0) {
            uart3_rx_len  = Size;
            uart3_rx_flag = 1;
        }
    }
}

/* ================================================================
 * 三、HMI 串口屏协议 — 薄封装（帧尾 \xff\xff\xff，直接发送）
 * ================================================================ */

static void HMI_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len <= 0) return;

    /* 追加协议尾 */
    if ((unsigned)len + 3 <= sizeof(buf)) {
        buf[len + 0] = (char)0xff;
        buf[len + 1] = (char)0xff;
        buf[len + 2] = (char)0xff;
        len += 3;
    }
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

void HMI_SendStr(const char *ctl, const char *text) { HMI_Printf("%s=\"%s\"", ctl, text); }
void HMI_SendInt(const char *ctl, int num)          { HMI_Printf("%s=%d", ctl, num); }

void HMI_SendFloat(const char *ctl, float num, int decimals)
{
    static const int mul[] = {1, 10, 100, 1000};
    int d = (decimals < 0 || decimals > 3) ? 2 : decimals;
    long scaled = (long)(num * (float)mul[d] + 0.5f);
    HMI_Printf("%s=%ld", ctl, scaled);
}

void HMI_WaveAdd(const char *ctl, int ch, int val)  { HMI_Printf("add %s,%d,%d", ctl, ch, val); }
void HMI_WaveClear(const char *ctl, int ch)         { HMI_Printf("cle %s,%d", ctl, ch); }


void UART1_Printf(const char *format,...)
{
	char tmp[128];
	va_list argptr;
	va_start(argptr,format);
	vsprintf((char* )tmp,format,argptr);
	va_end(argptr);
	HAL_UART_Transmit(&huart1,(const uint8_t *)&tmp,strlen(tmp),HAL_MAX_DELAY);	
}

void UART3_Printf(const char *format,...)
{
	char tmp[128];
	va_list argptr;
	va_start(argptr,format);
	vsprintf((char* )tmp,format,argptr);
	va_end(argptr);
	HAL_UART_Transmit(&huart3,(const uint8_t *)&tmp,strlen(tmp),HAL_MAX_DELAY);	
}