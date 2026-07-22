#ifndef SWEEP_LEARN_H
#define SWEEP_LEARN_H

#include <stdint.h>

/* ── 扫频参数 ── */
#define SWEEP_START_HZ       1000U
#define SWEEP_END_HZ         50000U
#define SWEEP_STEP_HZ        200U
#define SWEEP_SAMPLES        1024U
#define SWEEP_SETTLE_MS      3U
#define SWEEP_ADC_TIMEOUT_MS 20U

/* ── 拟合参数 ── */
#define FIT_MIN_POINTS       12U
#define FIT_VALID_MAG_RATIO  0.025f
#define FIT_MODEL_LOWPASS    0U
#define FIT_MODEL_BANDPASS   1U
#define FIT_MODEL_HIGHPASS   2U
#define FIT_MODEL_BANDSTOP   3U

/* ── IIR 系数 ── */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    uint8_t valid;
} iir_coeff_t;

/* ── 扫频数据点 ── */
typedef struct {
    float freq_hz;
    float mag;
    float phase_deg;
} sweep_point_t;

/* ── 拟合结果 ── */
typedef struct {
    uint32_t model;         /* 模型类型 0-3 */
    float    f0_hz;         /* 特征频率 */
    float    q;             /* 品质因数 */
    float    k;             /* 增益 */
    float    n2, n1, n0;    /* 模拟传递函数分子系数 */
    float    a, b;          /* 模拟传递函数分母系数: s² + a*s + b */
    float    delay_us;      /* 系统延迟 */
    float    phase0_deg;    /* 初始相位偏移 */
} fit_result_t;

/* ── 公开接口 ── */

/* ADC 完成通知（main.c 的 HAL_ADC_ConvCpltCallback 中调用） */
void sweep_learn_notify_adc_done(void);

/* 查询是否正在扫频（供 ADC 回调分流） */
uint8_t sweep_learn_is_sweeping(void);

/* 模拟二阶带通滤波器（跳过扫频，直接构造系数用于测试） */
void sweep_learn_sim_bandpass(float f0_hz, float q, float k);

/* 执行完整扫频+拟合+IIR系数生成 (阻塞, 约2-3秒) */
void sweep_learn_run(void);

/* 获取生成的 IIR 系数 */
const iir_coeff_t* sweep_learn_get_coeffs(void);

/* 获取拟合结果 */
const fit_result_t* sweep_learn_get_fit_result(void);

/* 获取模型名称字符串 */
const char* sweep_learn_model_name(uint32_t model);

/* 获取扫频数据点数 */
uint32_t sweep_learn_get_point_count(void);

/* 获取第 n 个扫频点 (用于屏幕绘制曲线) */
const sweep_point_t* sweep_learn_get_point(uint32_t idx);

#endif
