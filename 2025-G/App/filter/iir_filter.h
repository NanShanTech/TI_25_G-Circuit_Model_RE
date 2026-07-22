#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <stdint.h>

/* 缓冲参数 */
#define IIR_BLOCK_SAMPLES  256U

/* 输出增益 0.0~1.0 */
#define IIR_OUTPUT_GAIN    0.8f

/* 用扫频得到的系数初始化 biquad */
void iir_filter_init(float b0, float b1, float b2,
                     float a1, float a2);

/* 处理一块 ADC 数据 → DAC 数据 */
void iir_filter_process_block(const uint16_t *adc_buf, uint16_t *dac_buf,
                              uint32_t adc_offs, uint32_t dac_offs,
                              uint32_t count);

/* 滤波器是否已就绪 */
uint8_t iir_filter_is_ready(void);

/* 重置滤波器状态（切换模式时清零历史） */
void iir_filter_reset(void);

#endif
