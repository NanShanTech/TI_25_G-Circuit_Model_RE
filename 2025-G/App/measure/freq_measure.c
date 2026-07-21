/**
 * @file    freq_measure.c
 * @brief   频率测量实现 —— 基于 STM32H743 TIM2 纯硬件双模式
 * @note    引脚: PA5 (TIM2_CH1)
 *          低频: 测周法 (TIM2 输入捕获 + 从模式复位)
 *          高频: 测频法 (TIM2 外部时钟模式1 ETR计数，零中断)
 */

#include "freq_measure.h"
#include "tim.h"
#include "gpio.h"

#define FMODE_PERIOD 0
#define FMODE_COUNT  1

/* ---- 默认配置 ---- */
#define TICK_FREQ_DEFAULT    240000000.0f  /* TIM2 挂载频率 (H743 APB1 定时器时钟) */
#define SWITCH_HIGH_DEFAULT  50000.0f      /* 高于此值切测频模式 */
#define SWITCH_LOW_DEFAULT   40000.0f      /* 低于此值切回测周模式 */
#define GATE_TIME_DEFAULT    500           /* 测频闸门时间 (ms) */

static void freq_hw_switch(FreqMeasure *self, uint8_t target_mode) {
    GPIO_InitTypeDef gpio = {0};
    self->first_frame = 1;

    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_TIM_Base_Stop(self->htim);
    HAL_TIM_IC_Stop(self->htim, TIM_CHANNEL_1);
    HAL_TIM_Base_DeInit(self->htim);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin       = GPIO_PIN_5;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    if (target_mode == FMODE_PERIOD) {
        MX_TIM2_Init();
        __HAL_TIM_CLEAR_FLAG(self->htim, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start(self->htim);
        HAL_TIM_IC_Start(self->htim, TIM_CHANNEL_1);
    } else {
        self->htim->Instance = TIM2;
        self->htim->Init.Prescaler = 0;
        self->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
        self->htim->Init.Period = 0xFFFFFFFF;
        self->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        HAL_TIM_Base_Init(self->htim);

        TIM_SlaveConfigTypeDef sSlaveConfig = {0};
        sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
        sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
        sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
        sSlaveConfig.TriggerFilter = 0;
        HAL_TIM_SlaveConfigSynchro(self->htim, &sSlaveConfig);

        __HAL_TIM_SET_COUNTER(self->htim, 0);
        HAL_TIM_Base_Start(self->htim);
    }
}

static float freq_read_period(FreqMeasure *self) {
    if (__HAL_TIM_GET_FLAG(self->htim, TIM_FLAG_CC1) != RESET) {
        uint32_t cap = HAL_TIM_ReadCapturedValue(self->htim, TIM_CHANNEL_1);
        __HAL_TIM_CLEAR_FLAG(self->htim, TIM_FLAG_CC1);
        __HAL_TIM_CLEAR_FLAG(self->htim, TIM_FLAG_UPDATE);

        if (self->first_frame) {
            self->first_frame = 0;
            return -1.0f;
        }

        if (cap == 0) return 0.0f;
        return (float)((double)self->tick_freq / (double)(cap + 1));
    }
    return -1.0f;
}

/* ================= 公共 API ================= */

void FreqMeasure_Init(FreqMeasure *self, TIM_HandleTypeDef *htim) {
    self->htim            = htim;
    self->tick_freq       = TICK_FREQ_DEFAULT;
    self->switch_high_thr = SWITCH_HIGH_DEFAULT;
    self->switch_low_thr  = SWITCH_LOW_DEFAULT;
    self->gate_time_ms    = GATE_TIME_DEFAULT;
    self->mode            = FMODE_PERIOD;
    freq_hw_switch(self, FMODE_PERIOD);
}

void FreqMeasure_Process(FreqMeasure *self, Wave_Struct *wave) {
    if (self->mode == FMODE_PERIOD) {
        float f = freq_read_period(self);
        if (f >= 0.0f) {
            wave->Freq = f;

            if (f > self->switch_high_thr) {
                self->mode = FMODE_COUNT;
                freq_hw_switch(self, FMODE_COUNT);
                self->measuring = 0;
            }
        }
    } else {
        if (!self->measuring) {
            __HAL_TIM_SET_COUNTER(self->htim, 0);
            self->gate_start_ms = HAL_GetTick();
            self->measuring     = 1;
        } else {
            uint32_t current_ms = HAL_GetTick();
            uint32_t delta_ms   = current_ms - self->gate_start_ms;

            if (delta_ms >= self->gate_time_ms) {
                uint32_t total_pulses = __HAL_TIM_GET_COUNTER(self->htim);

                if (delta_ms > 0) {
                    wave->Freq = (float)(((double)total_pulses * 1000.0) / (double)delta_ms);
                }

                self->measuring = 0;

                if (wave->Freq < self->switch_low_thr) {
                    self->mode = FMODE_PERIOD;
                    freq_hw_switch(self, FMODE_PERIOD);
                }
            }
        }
    }
}
