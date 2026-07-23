#ifndef FILTER_RUNTIME_H
#define FILTER_RUNTIME_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum {
    FILTER_RUNTIME_OK = 0,
    FILTER_RUNTIME_ADC_START_FAILED,
    FILTER_RUNTIME_DAC_START_FAILED,
    FILTER_RUNTIME_TIM_START_FAILED,
} filter_runtime_status_t;

/* 启动/停止 ADC2(PB0) -> IIR -> DAC 的实时双缓冲链路。 */
filter_runtime_status_t filter_runtime_start(void);
void filter_runtime_stop(void);

/* 处理 DMA 中断留下的前半/后半缓冲标志，需在主循环中高频调用。 */
void filter_runtime_process_pending(void);

/* ADC DMA 回调入口：仅置位，不在中断中执行 IIR 运算。 */
uint8_t filter_runtime_on_adc_half_complete(ADC_HandleTypeDef *hadc);
uint8_t filter_runtime_on_adc_complete(ADC_HandleTypeDef *hadc);

/* 返回非零表示主循环曾未能及时处理某个半缓冲。 */
uint8_t filter_runtime_had_overrun(void);

#endif
