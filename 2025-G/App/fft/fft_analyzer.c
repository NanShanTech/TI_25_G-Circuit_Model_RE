/**
 * @file    fft_analyzer.c
 * @brief   FFT 频谱分析实现（基于 CMSIS-DSP + 自定义扩展库）
 */

#include "fft_analyzer.h"
#include "extra_ffts.h"
#include "arm_const_structs_extra.h"
#include "arm_common_tables_extra.h"
#include <math.h>
#include <string.h>

#define RAM_ADDR ".SRAM1"  // 大数组放入 SRAM_D1 区域

/*----------------------------------------------------------------------------
 * FFT 实例查找表（支持 16~16384 点）
 *----------------------------------------------------------------------------*/
static arm_cfft_instance_f32_extra *fft_get_instance(uint16_t fft_n) {
    switch (fft_n) {
        case 16:    return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len16_extra;
        case 32:    return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len32_extra;
        case 64:    return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len64_extra;
        case 128:   return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len128_extra;
        case 256:   return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len256_extra;
        case 512:   return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len512_extra;
        case 1024:  return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len1024_extra;
        case 2048:  return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len2048_extra;
        case 4096:  return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len4096_extra;
        case 8192:  return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len8192_extra;
        case 16384: return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len16384_extra;
        default:    return (arm_cfft_instance_f32_extra *)&arm_cfft_sR_f32_len2048_extra;
    }
}

/*----------------------------------------------------------------------------
 * 数据预处理：ADC 原始值 → 复数 Z = (ADC - DC_offset) * 3.3/4095 + 0*j
 *----------------------------------------------------------------------------*/
void fft_prepare(const uint16_t *adc_data, fftin_t *out) {
    float dc_offset = 0.0f;
    const float v_scale = 3.3f / 4095.0f;

    for (uint32_t i = 0; i < FFT_N; i++) {
        dc_offset += (float)adc_data[i];
    }
    dc_offset /= (float)FFT_N;

    for (uint32_t i = 0; i < FFT_N; i++) {
        out->cmp[2 * i]     = ((float)adc_data[i] - dc_offset) * v_scale;
        out->cmp[2 * i + 1] = 0.0f;
    }
}

/*----------------------------------------------------------------------------
 * 加窗
 *----------------------------------------------------------------------------*/
void fft_window(fftin_t *data, const float *window_func) {
    for (uint16_t i = 0; i < FFT_N; i++) {
        data->cmp[2 * i] *= window_func[i];
    }
}

/*----------------------------------------------------------------------------
 * FFT 主运算
 *----------------------------------------------------------------------------*/
void fft_process(fftin_t *data, fftout_t *output) {
    arm_cfft_f32_extra(fft_get_instance(FFT_N), data->cmp, 0, 1);

    for (uint16_t i = 0; i < FFT_N_2; i++) {
        float real = data->cmp[2 * i];
        float imag = data->cmp[2 * i + 1];
        float mag;
        arm_sqrt_f32(real * real + imag * imag, &mag);
        output->mag[i]   = mag;
        output->phase[i] = atan2f(imag, real);
    }

    /* 去除直流及低频分量 */
    freqaxis_t *axis = fft_get_axis();
    for (uint32_t i = 0; i < FFT_N_2; i++) {
        if (axis->axis[i] <= DC_SCOPE) {
            output->mag[i] = 0.0f;
        }
    }
}

/*----------------------------------------------------------------------------
 * 幅度归一化
 *----------------------------------------------------------------------------*/
void fft_normalize(fftout_t *result, float norm_val) {
    float max_mag = 0.0f;
    for (uint16_t i = 0; i < FFT_N_2; i++) {
        if (result->mag[i] > max_mag) max_mag = result->mag[i];
    }
    if (max_mag < MIN_VALID_DENOM) return;
    for (uint16_t i = 0; i < FFT_N_2; i++) {
        result->mag[i] = result->mag[i] / max_mag * norm_val;
    }
}

/*----------------------------------------------------------------------------
 * 频率轴（静态缓存）
 *----------------------------------------------------------------------------*/
freqaxis_t *fft_get_axis(void) {
    static freqaxis_t axis __attribute__((section(RAM_ADDR)));
    static uint8_t inited = 0;
    if (!inited) {
        for (uint32_t i = 0; i < FFT_N_2; i++) {
            axis.axis[i] = (float)i * FREQ_S / FFT_N;
        }
        inited = 1;
    }
    return &axis;
}

/*----------------------------------------------------------------------------
 * 前三峰值搜索
 *----------------------------------------------------------------------------*/
void fft_find_peaks(const fftout_t *mag, peak3_t *peaks) {
    freqaxis_t *axis = fft_get_axis();
    memset(peaks, 0, sizeof(peak3_t));

    /* 第一峰值 */
    float max_val = 0.0f;
    for (uint16_t i = 0; i < FFT_N_2; i++) {
        if (mag->mag[i] > max_val) {
            max_val = mag->mag[i];
            peaks->index[0] = i;
        }
    }

    /* 第二峰值（排除第一峰值邻域） */
    float max1_val = mag->mag[peaks->index[0]];
    float f1_up   = axis->axis[peaks->index[0]] + FREQ_SCOPE;
    float f1_down = axis->axis[peaks->index[0]] - FREQ_SCOPE;
    max_val = 0.0f;
    for (uint16_t i = 0; i < FFT_N_2; i++) {
        if (mag->mag[i] <= max_val || mag->mag[i] >= max1_val) continue;
        if (axis->axis[i] > f1_up || axis->axis[i] < f1_down) {
            max_val = mag->mag[i];
            peaks->index[1] = i;
        }
    }

    /* 第三峰值 */
    float max2_val = mag->mag[peaks->index[1]];
    float f2_up   = axis->axis[peaks->index[1]] + FREQ_SCOPE;
    float f2_down = axis->axis[peaks->index[1]] - FREQ_SCOPE;
    max_val = 0.0f;
    for (uint16_t i = 0; i < FFT_N_2; i++) {
        if (mag->mag[i] <= max_val || mag->mag[i] >= max2_val) continue;
        if ((axis->axis[i] > f1_up || axis->axis[i] < f1_down) &&
            (axis->axis[i] > f2_up || axis->axis[i] < f2_down)) {
            max_val = mag->mag[i];
            peaks->index[2] = i;
        }
    }
}

/*----------------------------------------------------------------------------
 * 频率规整
 *----------------------------------------------------------------------------*/
float fft_round_freq(float raw_freq) {
    if (raw_freq < MIN_VALID_DENOM) return 0.0f;

    uint32_t fi = (uint32_t)raw_freq;
    uint32_t up   = FREQ_STEP - (fi % (uint32_t)FREQ_STEP);
    uint32_t down = fi % (uint32_t)FREQ_STEP;
    float result  = (up >= down) ? (float)(fi - down) : (float)(fi + up);

    if (result > FREQ_LIMIT_HIGH) return FREQ_LIMIT_HIGH;
    if (result < FREQ_LIMIT_LOW)  return FREQ_LIMIT_LOW;
    return result;
}

/*----------------------------------------------------------------------------
 * 时域 Vpp
 *----------------------------------------------------------------------------*/
static inline float parabolic_interpolate(float y_m1, float y_m, float y_p1) {
    float denominator = y_m1 - 2.0f * y_m + y_p1;
    if (fabsf(denominator) < 1e-6f) {
        return y_m;
    }
    float delta = 0.5f * (y_m1 - y_p1) / denominator;
    float true_peak = y_m - 0.25f * (y_m1 - y_p1) * delta;
    return true_peak;
}

float find_vpp(const fftin_t *input) {
    uint32_t max_idx = 1;
    uint32_t min_idx = 1;
    float discrete_max = input->cmp[2]; // cmp[2*1]
    float discrete_min = input->cmp[2]; // cmp[2*1]

    for (uint32_t i = 1; i < FFT_N - 1; i++) {
        float v = input->cmp[2 * i]; 
        
        if (v > discrete_max) {
            discrete_max = v;
            max_idx = i;
        }
        if (v < discrete_min) {
            discrete_min = v;
            min_idx = i;
        }
    }
    float true_max = parabolic_interpolate(
        input->cmp[2 * (max_idx - 1)],
        input->cmp[2 * max_idx],
        input->cmp[2 * (max_idx + 1)]
    );

    float true_min = parabolic_interpolate(
        input->cmp[2 * (min_idx - 1)],
        input->cmp[2 * min_idx],
        input->cmp[2 * (min_idx + 1)]
    );

    return true_max - true_min;
}

/*----------------------------------------------------------------------------
 * 相位
 *----------------------------------------------------------------------------*/
void fft_phase_atan(const float *complex_data, uint32_t index, float *phase) {
    *phase = atan2f(complex_data[2 * index + 1], complex_data[2 * index]);
}

/*----------------------------------------------------------------------------
 * 波形识别
 *----------------------------------------------------------------------------*/
float fft_max_harmonic(const float *mag, uint16_t base_idx, uint8_t harmonic_n) {
    uint32_t target = harmonic_n * base_idx;
    if (target >= FFT_N_2) return 0.0f;

    float max_v = mag[target];
    for (int off = -2; off <= 2; off++) {
        int idx = (int)target + off;
        if (idx >= 0 && idx < FFT_N_2 && mag[idx] > max_v) {
            max_v = mag[idx];
        }
    }
    return max_v;
}

WaveType_t fft_rec_wavetype(const fftout_t *mag, uint16_t base_idx) {
    if (base_idx >= FFT_N_2 || base_idx == 0) return WAVE_UNKNOWN;

    float base_amp = mag->mag[base_idx];
    if (base_amp < 0.02f) return WAVE_SINE;

    float h3_amp = fft_max_harmonic(mag->mag, base_idx, 3);
    float ratio  = h3_amp / base_amp;

    if (ratio < 0.15f)       return WAVE_SINE;
    else if (ratio < 0.40f)  return WAVE_TRIANGLE;
    else                     return WAVE_SQUARE;
}
