/**
 * @file    freq_measure.h
 * @brief   频率测量 API —— 双模式自动切换
 *
 * 用法：
 *   1. FreqMeasure_Init(&obj, &htim2)
 *   2. 主循环中周期性调用 FreqMeasure_Process()
 *   阈值/时钟频率等有合理默认值，也可在 Init 后手动修改 self->xxx
 */

#ifndef __FREQ_MEASURE_H
#define __FREQ_MEASURE_H

#include "app_types.h"
#include "stm32h7xx_hal.h"

typedef struct {
    /* ---- 配置（由 Init 写入）---- */
    TIM_HandleTypeDef *htim;
    float              tick_freq;
    float              switch_high_thr;
    float              switch_low_thr;
    uint32_t           gate_time_ms;

    /* ---- 内部状态 ---- */
    uint8_t  mode;
    uint32_t gate_start_ms;
    uint8_t  measuring;
    uint8_t  first_frame;
} FreqMeasure;

void FreqMeasure_Init(FreqMeasure *self, TIM_HandleTypeDef *htim);
void FreqMeasure_Process(FreqMeasure *self, Wave_Struct *wave);

#endif
