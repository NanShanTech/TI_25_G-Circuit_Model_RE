#ifndef __ADC_APP_H
#define __ADC_APP_H

#include "app_types.h"
#include "main.h"

#define ADC_VREF 3.3f


#define ADC_BITS      12
#define ADC_MAX_CODE  (1u << ADC_BITS)


typedef struct {
    float vpp;          /* 峰峰值电压，单位：V */
    float vmax;         /* 最大电压，单位：V */
    float vmin;         /* 最小电压，单位：V */
    float vavg;         /* 平均电压，也可理解为直流偏置，单位：V */
    uint16_t max_adc;   /* 最大 ADC 原始采样值 */
    uint16_t min_adc;   /* 最小 ADC 原始采样值 */
} adc_waveform_info_t;


void ADC_proc(void);

/*
 * 基础峰峰值计算函数。
 * 直接扫描整个数据缓冲区，寻找最大值和最小值。
 * 适合干净信号。
 */
void adc_get_vpp(adc_waveform_info_t *info,
                 const uint16_t *data,
                 uint32_t len);

/*
 * 抗毛刺峰峰值计算函数。
 * 使用百分位方法丢弃两端异常采样点，
 * 可以降低毛刺对峰峰值计算的影响。
 * discard_pct 表示每一端丢弃的百分比。
 */
void adc_get_vpp_robust(adc_waveform_info_t *info,
                        const uint16_t *data,
                        uint32_t len,
                        float discard_pct);

/*
 * 自动峰峰值计算函数。
 * 自动在基础算法和抗毛刺算法之间选择。
 */
void adc_get_vpp_auto(adc_waveform_info_t *info,
                      const uint16_t *data,
                      uint32_t len);

#endif
