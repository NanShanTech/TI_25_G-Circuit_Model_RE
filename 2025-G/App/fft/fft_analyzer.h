#ifndef __FFT_ANALYZER_H
#define __FFT_ANALYZER_H

#include "app_types.h"
#include "arm_math.h"


#define FFT_N           2048        // FFT 点数（必须是 2 的幂，最大 16384）
#define FFT_2N  (FFT_N * 2)
#define FREQ_S          2000000     // ADC 采样率 (Hz)，需与 TIM3 触发频率匹配
#define FFT_N_2         (FFT_N / 2) // 频谱有效点数（自动计算，勿改）

#define DC_SCOPE        80          // 直流分量屏蔽范围 (Hz)，低于此频率的幅值清零
#define FREQ_SCOPE      0           // 峰值搜索邻域 (Hz)，0=禁用
#define FREQ_STEP       10          // 频率规整步长 (Hz)，如 5000 的整数倍
#define HAMMING_WIN_CORR 1.85185f   // 汉明窗幅度恢复系数

/* 波形识别阈值 */
#define MIN_VALID_DENOM 1e-12f      // 防止除零的保护值
#define FREQ_LIMIT_HIGH 150000      // 频率上限 (Hz)
#define FREQ_LIMIT_LOW  5000        // 频率下限 (Hz)

/** FFT 输入（复数交错: real[0], imag[0], real[1], imag[1]...） */
typedef struct {
    float cmp[FFT_2N];
} fftin_t;

/** FFT 输出（幅度谱 + 相位谱） */
typedef struct {
    float phase[FFT_2N];
    float mag[FFT_N_2];
} fftout_t;


/** 前三大峰值索引 */
typedef struct {
    uint16_t index[3];
} peak3_t;


/** 频率轴 */
typedef struct {
    float axis[FFT_N_2];
} freqaxis_t;


/** 将 ADC 原始数据转换为 FFT 输入（去直流 + 电压定标） */
void fft_prepare(const uint16_t *adc_data, fftin_t *out);

/** 加窗函数（调用前 out 须已填充实部） */
void fft_window(fftin_t *data, const float *window_func);

/** 执行 FFT，结果填入 fftout_t */
void fft_process(fftin_t *data, fftout_t *output);

/** 幅度归一化（除以最大值再乘 norm_val） */
void fft_normalize(fftout_t *result, float norm_val);

/** 获取频率轴（首次调用计算，后续返回静态缓存） */
freqaxis_t *fft_get_axis(void);

/* ---- 峰值搜索与波形分析 ---- */

/** 获取幅度谱最大的三个峰值索引 */
void fft_find_peaks(const fftout_t *mag, peak3_t *peaks);

/** 频率规整到最近整数步长 */
float fft_round_freq(float raw_freq);

/** 识别波形类型（基于谐波比例） */
WaveType_t fft_rec_wavetype(const fftout_t *mag, uint16_t base_idx);

/** 时域峰峰值提取（滑动窗口法） */
float find_vpp(const fftin_t *input);

/** 相位提取 */
void fft_phase_atan(const float *complex_data, uint32_t index, float *phase);

/** 指定谐波最大幅值查找 */
float fft_max_harmonic(const float *mag, uint16_t base_idx, uint8_t harmonic_n);

#endif /* __FFT_ANALYZER_H */
