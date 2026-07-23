#include "sweep_learn.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "serial.h"
#include "ad9910.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

/* ── 硬件句柄外部声明 ── */
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

/* ── ADC/DAC 满量程 ── */
#define ADC_MID_CODE       2048.0f

/* ── 最大扫频点数 ── */
#define SWEEP_FINE_POINTS   (((SWEEP_FINE_END_HZ - SWEEP_START_HZ) / \
                              SWEEP_FINE_STEP_HZ) + 1U)
#define SWEEP_HIGH_POINTS   ((SWEEP_END_HZ - SWEEP_FINE_END_HZ) / \
                              SWEEP_HIGH_STEP_HZ)
#define SWEEP_MAX_POINTS    (SWEEP_FINE_POINTS + SWEEP_HIGH_POINTS)

/* ── 算法用 π ── */
#define PI_F                3.14159265358979323846f

/*仅用于内部物理约束搜索*/
#define RLC_DAMPING_SERIES  0U
#define RLC_DAMPING_PARALLEL 1U

/*采样率*/
#define SAMPLE_RATE_HZ      1500000.0f

/* 未执行有效扫频学习时使用的二阶低通默认参数。 */
#define DEFAULT_FILTER_F0_HZ      27793.0f
#define DEFAULT_FILTER_Q          0.08f
#define DEFAULT_FILTER_K          0.6766f
#define DEFAULT_FILTER_DELAY_US   (-0.2f)

/* ── 32字节对齐（D-Cache 行大小） ── */
#define ADC_BUF_ALIGN       __attribute__((aligned(32)))

/* ══════════════════════════════════════════════════════════════
 * 静态数据
 * ══════════════════════════════════════════════════════════════ */

static uint16_t sweep_adc1_buf[SWEEP_SAMPLES]
    __attribute__((section(".AXI_SRAM"))) ADC_BUF_ALIGN;
static uint16_t sweep_adc2_buf[SWEEP_SAMPLES]
    __attribute__((section(".AXI_SRAM"))) ADC_BUF_ALIGN;

/* 扫频结果数组 */
static sweep_point_t sweep_data[SWEEP_MAX_POINTS];
static uint32_t      sweep_count;

/* ADC 完成标志（中断中置位） */
static volatile uint8_t sweep_adc1_done;
static volatile uint8_t sweep_adc2_done;
static volatile uint8_t sweep_adc_error;
/* 扫频进行中标志（供 ADC 回调判断） */
static volatile uint8_t s_sweeping;

/* 上电先提供有效默认值，成功学习后再由扫频结果整体替换。 */
static fit_result_t fit_result = {
    .model = FIT_MODEL_LOWPASS,
    .f0_hz = DEFAULT_FILTER_F0_HZ,
    .q = DEFAULT_FILTER_Q,
    .k = DEFAULT_FILTER_K,
    .n2 = 0.0f,
    .n1 = 0.0f,
    .n0 = 2.0633010e10f,
    .a = 2.1828571e6f,
    .b = 3.0495137e10f,
    .delay_us = DEFAULT_FILTER_DELAY_US,
    .phase0_deg = 0.0f,
};
static iir_coeff_t iir_coeffs = {
    .b0 = 0.001340f,
    .b1 = 0.002681f,
    .b2 = 0.001340f,
    .a1 = -1.165444f,
    .a2 = 0.173369f,
    .valid = 1U,
};
static uint8_t s_has_learned;

/* 根据 f0 计算物理范围允许的 sqrt(L/C) 区间。 */
static uint8_t rlc_get_z_bounds(float f0_hz, float *z_min, float *z_max)
{
    float omega;
    float lower;
    float upper;

    if (f0_hz <= 0.0f || z_min == NULL || z_max == NULL) return 0U;

    omega = 2.0f * PI_F * f0_hz;
    lower = fmaxf(RLC_L_MIN_H * omega, 1.0f / (RLC_C_MAX_F * omega));
    upper = fminf(RLC_L_MAX_H * omega, 1.0f / (RLC_C_MIN_F * omega));
    if (lower > upper) return 0U;

    *z_min = lower;
    *z_max = upper;
    return 1U;
}

/* 给定 f0 和阻尼形式，计算 RLC 约束对应的 Q 可行区间。 */
static uint8_t rlc_get_q_bounds(float f0_hz, uint32_t topology,
                                float *q_min, float *q_max)
{
    float z_min;
    float z_max;

    if (q_min == NULL || q_max == NULL ||
        !rlc_get_z_bounds(f0_hz, &z_min, &z_max)) {
        return 0U;
    }

    if (topology == RLC_DAMPING_PARALLEL) {
        /* 并联 RLC：Q=R/√(L/C)。 */
        *q_min = RLC_R_MIN_OHM / z_max;
        *q_max = RLC_R_MAX_OHM / z_min;
    } else {
        /* 串联或混合串联阻尼：Q=√(L/C)/R。 */
        *q_min = z_min / RLC_R_MAX_OHM;
        *q_max = z_max / RLC_R_MIN_OHM;
    }
    return (*q_min <= *q_max) ? 1U : 0U;
}

/* 候选点必须能还原出题目范围内的一组 R、L、C。 */
static uint8_t rlc_candidate_is_feasible(float f0_hz, uint32_t topology,
                                          float q)
{
    float z_min;
    float z_max;

    if (q <= 0.0f || !rlc_get_z_bounds(f0_hz, &z_min, &z_max)) {
        return 0U;
    }

    if (topology == RLC_DAMPING_PARALLEL) {
        z_min = fmaxf(z_min, RLC_R_MIN_OHM / q);
        z_max = fminf(z_max, RLC_R_MAX_OHM / q);
    } else {
        z_min = fmaxf(z_min, RLC_R_MIN_OHM * q);
        z_max = fminf(z_max, RLC_R_MAX_OHM * q);
    }
    return (z_min <= z_max) ? 1U : 0U;
}

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：正交解调
 * ══════════════════════════════════════════════════════════════ */

/* 对 adc_buf 做同频正交解调，返回归一化复数基波分量。 */
static void sweep_quadrature_demod(const uint16_t *buf, uint32_t n,
                                   float freq_hz, float fs_hz,
                                   float *re_out, float *im_out)
{
    float sum   = 0.0f;
    float re    = 0.0f;
    float im    = 0.0f;
    float omega = 2.0f * PI_F * freq_hz / fs_hz;
    float cos_step = cosf(omega);
    float sin_step = sinf(omega);
    float cos_n = 1.0f;
    float sin_n = 0.0f;

    /* 去直流均值 */
    for (uint32_t i = 0; i < n; i++) {
        sum += (float)buf[i];
    }
    float mean = sum / (float)n;

    /* 正交投影 */
    for (uint32_t i = 0; i < n; i++) {
        float y = (float)buf[i] - mean;
        float next_cos;
        re += y * cos_n;
        im -= y * sin_n;

        next_cos = cos_n * cos_step - sin_n * sin_step;
        sin_n = sin_n * cos_step + cos_n * sin_step;
        cos_n = next_cos;

        if ((i & 0xFFU) == 0xFFU) {
            float norm = 1.0f / sqrtf(cos_n * cos_n + sin_n * sin_n);
            cos_n *= norm;
            sin_n *= norm;
        }
    }

    /* 以 ADC 中点 2048 为参考进行幅值归一化。 */
    *re_out = 2.0f * re / ((float)n * ADC_MID_CODE);
    *im_out = 2.0f * im / ((float)n * ADC_MID_CODE);
}

static uint8_t sweep_adc_is_clipped(const uint16_t *buf, uint32_t n)
{
    uint16_t min_code = 4095U;
    uint16_t max_code = 0U;

    for (uint32_t i = 0; i < n; i++) {
        if (buf[i] < min_code) min_code = buf[i];
        if (buf[i] > max_code) max_code = buf[i];
    }

    return (min_code <= SWEEP_ADC_CLIP_MARGIN ||
            max_code >= (4095U - SWEEP_ADC_CLIP_MARGIN));
}

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：数据存储
 * ══════════════════════════════════════════════════════════════ */

static void sweep_store(float freq, float mag, float phase_deg)
{
    if (sweep_count >= SWEEP_MAX_POINTS) return;
    sweep_data[sweep_count].freq_hz   = freq;
    sweep_data[sweep_count].mag       = mag;
    sweep_data[sweep_count].phase_deg = phase_deg;
    sweep_count++;
}

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：相位展开
 * ══════════════════════════════════════════════════════════════ */

static float unwrap_phase(float phase, float last)
{
    float delta;

    if (!isfinite(phase)) return isfinite(last) ? last : 0.0f;
    if (!isfinite(last)) return phase;

    /* 用取模一次完成相位折返，输入异常大时也能保证有限时间返回。 */
    delta = fmodf(fmodf(phase, 360.0f) - fmodf(last, 360.0f),
                  360.0f);
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    return last + delta;
}

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：RLC 幅频基函数
 * H_base(s) 的幅频响应（单位增益归一化前的形状）
 * ══════════════════════════════════════════════════════════════ */

static float rlc_mag_base(uint32_t model, float freq_hz, float f0, float q)
{
    float w    = 2.0f * PI_F * freq_hz;
    float w0   = 2.0f * PI_F * f0;
    float a    = w0 / q;
    float b    = w0 * w0;
    float real = b - (w * w);
    float imag = a * w;
    float den  = sqrtf(real * real + imag * imag);

    if (den <= 0.0f) return 0.0f;

    if (model == FIT_MODEL_LOWPASS)  return b / den;
    if (model == FIT_MODEL_HIGHPASS) return (w * w) / den;
    if (model == FIT_MODEL_BANDSTOP) return (real >= 0.0f ? real : -real) / den;
    /* BANDPASS */                    return imag / den;
}

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：RLC 相频响应
 * ══════════════════════════════════════════════════════════════ */

static float rlc_phase_deg(uint32_t model, float freq_hz, float f0, float q)
{
    float w    = 2.0f * PI_F * freq_hz;
    float w0   = 2.0f * PI_F * f0;
    float a    = w0 / q;
    float real = (w0 * w0) - (w * w);
    float imag = a * w;
    float num_phase = 90.0f;  /* 带通默认 */

    if (model == FIT_MODEL_LOWPASS)      num_phase = 0.0f;
    else if (model == FIT_MODEL_HIGHPASS) num_phase = 180.0f;
    else if (model == FIT_MODEL_BANDSTOP) num_phase = (real >= 0.0f) ? 0.0f : 180.0f;

    return num_phase - (atan2f(imag, real) * 180.0f / PI_F);
}

static float sweep_phase_error(uint32_t model, float f0, float q,
                               float valid_min_mag)
{
    float last_phase = sweep_data[0].phase_deg;
    float ref_freq = sweep_data[sweep_count / 2U].freq_hz;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float slope;
    float offset;
    float error = 0.0f;
    uint32_t used = 0U;

    for (uint32_t i = 0U; i < sweep_count; i++) {
        float unwrapped = unwrap_phase(sweep_data[i].phase_deg, last_phase);
        float x = sweep_data[i].freq_hz - ref_freq;

        last_phase = unwrapped;
        if (sweep_data[i].mag < valid_min_mag) continue;

        {
            float residual = unwrapped -
                rlc_phase_deg(model, sweep_data[i].freq_hz, f0, q);
            sum_x += x;
            sum_y += residual;
            sum_xx += x * x;
            sum_xy += x * residual;
            used++;
        }
    }

    if (used < 3U) return 3.4e38f;
    {
        float den = (float)used * sum_xx - sum_x * sum_x;
        if (fabsf(den) <= 1.0e-12f) return 3.4e38f;
        slope = ((float)used * sum_xy - sum_x * sum_y) / den;
        offset = (sum_y - slope * sum_x) / (float)used;
    }

    last_phase = sweep_data[0].phase_deg;
    used = 0U;
    for (uint32_t i = 0U; i < sweep_count; i++) {
        float unwrapped = unwrap_phase(sweep_data[i].phase_deg, last_phase);
        float x = sweep_data[i].freq_hz - ref_freq;

        last_phase = unwrapped;
        if (sweep_data[i].mag < valid_min_mag) continue;

        {
            float residual = unwrapped -
                rlc_phase_deg(model, sweep_data[i].freq_hz, f0, q);
            float e = residual - (offset + slope * x);
            error += e * e;
            used++;
        }
    }

    return error / (float)used;
}

/* 求出整个扫频范围内的 Q 搜索外框，避免并联 RLC 的 Q 范围被截断。 */
static uint8_t rlc_get_search_q_bounds(uint32_t topology,
                                       float f0_min_hz, float f0_max_hz,
                                       float *q_min, float *q_max)
{
    float probe[4];
    uint32_t count = 0U;
    uint8_t found = 0U;

    if (q_min == NULL || q_max == NULL) return 0U;

    probe[count++] = f0_min_hz;
    probe[count++] = f0_max_hz;
    probe[count++] = sqrtf(f0_min_hz * f0_max_hz);
    probe[count++] = 1.0f /
        (2.0f * PI_F * sqrtf(RLC_L_MIN_H * RLC_C_MAX_F));

    for (uint32_t i = 0U; i < count; i++) {
        float local_min;
        float local_max;

        if (probe[i] < f0_min_hz || probe[i] > f0_max_hz ||
            !rlc_get_q_bounds(probe[i], topology, &local_min, &local_max)) {
            continue;
        }
        if (!found) {
            *q_min = local_min;
            *q_max = local_max;
            found = 1U;
        } else {
            if (local_min < *q_min) *q_min = local_min;
            if (local_max > *q_max) *q_max = local_max;
        }
    }
    return found && (*q_min > 0.0f) && (*q_max >= *q_min);
}

/* 合并两种阻尼关系的 Q 外框，搜索时不需要输出或判断具体拓扑。 */
static uint8_t rlc_get_search_q_bounds_any(float f0_min_hz, float f0_max_hz,
                                           float *q_min, float *q_max)
{
    float series_min;
    float series_max;
    float parallel_min;
    float parallel_max;
    uint8_t series_ok = rlc_get_search_q_bounds(
        RLC_DAMPING_SERIES, f0_min_hz, f0_max_hz,
        &series_min, &series_max);
    uint8_t parallel_ok = rlc_get_search_q_bounds(
        RLC_DAMPING_PARALLEL, f0_min_hz, f0_max_hz,
        &parallel_min, &parallel_max);

    if (q_min == NULL || q_max == NULL || (!series_ok && !parallel_ok)) {
        return 0U;
    }
    if (!series_ok) {
        *q_min = parallel_min;
        *q_max = parallel_max;
    } else if (!parallel_ok) {
        *q_min = series_min;
        *q_max = series_max;
    } else {
        *q_min = fminf(series_min, parallel_min);
        *q_max = fmaxf(series_max, parallel_max);
    }
    return 1U;
}

/* 候选 Q 只要能由任一种 RLC 阻尼关系实现，就满足题目约束。 */
static uint8_t rlc_candidate_is_feasible_any(float f0_hz, float q)
{
    return rlc_candidate_is_feasible(f0_hz, RLC_DAMPING_SERIES, q) ||
           rlc_candidate_is_feasible(f0_hz, RLC_DAMPING_PARALLEL, q);
}

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：模型名称
 * ══════════════════════════════════════════════════════════════ */

const char *sweep_learn_model_name(uint32_t model)
{
    if (model == FIT_MODEL_LOWPASS)  return "LOWPASS";
    if (model == FIT_MODEL_HIGHPASS) return "HIGHPASS";
    if (model == FIT_MODEL_BANDSTOP) return "BANDSTOP";
    return "BANDPASS";
}

/* ══════════════════════════════════════════════════════════════
 * 网格搜索 + 最小二乘拟合 RLC 模型
 * ══════════════════════════════════════════════════════════════ */

static uint8_t sweep_fit_model(void)
{
    float peak_mag = 0.0f;
    float best_f0  = 5000.0f;
    float best_q   = 1.0f;
    float best_k   = 1.0f;
    float best_err = 3.4e38f;
    uint32_t best_model  = FIT_MODEL_BANDPASS;
    float    valid_min_mag;
    float    f0_min_hz;
    float    f0_max_hz;

    if (sweep_count < FIT_MIN_POINTS) {
        UART1_Printf("FIT_ERROR: not enough points (%lu)\r\n", sweep_count);
        return 0U;
    }

    /* 找峰值幅度 */
    for (uint32_t i = 0; i < sweep_count; i++) {
        if (sweep_data[i].mag > peak_mag) peak_mag = sweep_data[i].mag;
    }
    if (peak_mag <= 0.0f) {
        UART1_Printf("FIT_ERROR: bad data\r\n");
        return 0U;
    }
    valid_min_mag = peak_mag * FIT_VALID_MAG_RATIO;

    /* 由 L、C 范围直接得到特征频率范围，先排除不可能的搜索区域。 */
    f0_min_hz = 1.0f / (2.0f * PI_F * sqrtf(RLC_L_MAX_H * RLC_C_MAX_F));
    f0_max_hz = 1.0f / (2.0f * PI_F * sqrtf(RLC_L_MIN_H * RLC_C_MIN_F));
    if (f0_min_hz < (float)SWEEP_START_HZ) {
        f0_min_hz = (float)SWEEP_START_HZ;
    }
    if (f0_max_hz > (float)SWEEP_END_HZ) {
        f0_max_hz = (float)SWEEP_END_HZ;
    }
    if (f0_min_hz >= f0_max_hz) {
        UART1_Printf("FIT_ERROR: sweep range misses RLC range\r\n");
        return 0U;
    }
    /* 合并两种物理阻尼关系，只遍历四种滤波器响应。 */
    {
        float q_domain_min;
        float q_domain_max;

        if (!rlc_get_search_q_bounds_any(f0_min_hz, f0_max_hz,
                                         &q_domain_min, &q_domain_max)) {
            return 0U;
        }

        for (uint32_t model = FIT_MODEL_LOWPASS;
             model <= FIT_MODEL_BANDSTOP; model++) {
        float model_best_f0  = sqrtf(f0_min_hz * f0_max_hz);
        float model_best_q   = sqrtf(q_domain_min * q_domain_max);
        float model_best_k   = peak_mag;
        float model_best_err = 3.4e38f;
        float model_best_k_at_err = peak_mag;

        /* 3 轮网格搜索：粗 → 中 → 精 */
        for (uint32_t pass = 0; pass < 3; pass++) {
            float f_min, f_max, q_min, q_max;

            if (pass == 0) {
                f_min = f0_min_hz;
                f_max = f0_max_hz;
                q_min = q_domain_min;
                q_max = q_domain_max;
            } else {
                float f_span = model_best_f0 * ((pass == 1) ? 0.35f : 0.12f);
                float q_span = model_best_q  * ((pass == 1) ? 0.55f : 0.20f);
                f_min = model_best_f0 - f_span;
                f_max = model_best_f0 + f_span;
                q_min = model_best_q  - q_span;
                q_max = model_best_q  + q_span;
                if (f_min < f0_min_hz) f_min = f0_min_hz;
                if (f_max > f0_max_hz) f_max = f0_max_hz;
                if (q_min < q_domain_min) q_min = q_domain_min;
                if (q_max > q_domain_max) q_max = q_domain_max;
            }

            for (uint32_t fi = 0; fi <= 24; fi++) {
                float f0;

                if (pass == 0) {
                    f0 = f_min * powf(f_max / f_min,
                                     (float)fi / 24.0f);
                } else {
                    f0 = f_min + ((f_max - f_min) * (float)fi) / 24.0f;
                }
                for (uint32_t qi = 0; qi <= 24; qi++) {
                    float q;
                    float k_num  = 0.0f;
                    float k_den  = 0.0f;
                    float err    = 0.0f;
                    uint32_t used     = 0;
                    uint32_t err_used = 0;

                    if (pass == 0) {
                        q = q_min * powf(q_max / q_min,
                                        (float)qi / 24.0f);
                    } else {
                        q = q_min + ((q_max - q_min) * (float)qi) / 24.0f;
                    }

                    if (!rlc_candidate_is_feasible_any(f0, q)) {
                        continue;
                    }

                    /* 最小二乘求 K */
                    for (uint32_t i = 0; i < sweep_count; i++) {
                        if (model != FIT_MODEL_BANDSTOP &&
                            sweep_data[i].mag < valid_min_mag) {
                            continue;
                        }
                        float base = rlc_mag_base(model, sweep_data[i].freq_hz, f0, q);
                        if (base > 1.0e-9f && isfinite(base)) {
                            float measured = sweep_data[i].mag;
                            if (model == FIT_MODEL_BANDSTOP &&
                                measured < valid_min_mag) {
                                measured = valid_min_mag;
                            }
                            k_num += measured * base;
                            k_den += base * base;
                            used++;
                        }
                    }
                    if (used < FIT_MIN_POINTS || k_den <= 0.0f) continue;
                    model_best_k = k_num / k_den;

                    /* 计算归一化误差 */
                    for (uint32_t i = 0; i < sweep_count; i++) {
                        if (model != FIT_MODEL_BANDSTOP &&
                            sweep_data[i].mag < valid_min_mag) {
                            continue;
                        }
                        float base = rlc_mag_base(model, sweep_data[i].freq_hz, f0, q);
                        float pred = model_best_k * base;
                        float measured = sweep_data[i].mag;
                        if (model == FIT_MODEL_BANDSTOP &&
                            measured < valid_min_mag) {
                            measured = valid_min_mag;
                        }
                        if (pred < valid_min_mag) pred = valid_min_mag;
                        float e = 20.0f * log10f(pred / measured);
                        err += e * e;
                        err_used++;
                    }
                    if (err_used < FIT_MIN_POINTS) continue;
                    err /= (float)err_used;

                    if (err < model_best_err) {
                        model_best_err       = err;
                        model_best_f0        = f0;
                        model_best_q         = q;
                        model_best_k_at_err  = model_best_k;
                    }
                }
            }
        }
        if (model_best_err >= 3.0e38f) continue;
        {
            float phase_err = sweep_phase_error(model, model_best_f0,
                                                model_best_q, valid_min_mag);
            float model_score = model_best_err;
            if (phase_err < 3.0e37f) model_score += 0.01f * phase_err;

            if (model_score < best_err) {
                best_err   = model_score;
                best_model = model;
                best_f0    = model_best_f0;
                best_q     = model_best_q;
                best_k     = model_best_k_at_err;
            }
        }
    }
    }

    if (best_err >= 3.0e38f) {
        UART1_Printf("FIT_ERROR: no valid model\r\n");
        return 0U;
    }

    /* ── 相位延迟拟合 ── */
    float sum_x  = 0.0f;
    float sum_y  = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float ref_freq = sweep_data[sweep_count / 2U].freq_hz;
    float last_phase = sweep_data[0].phase_deg;
    float phase_slope = 0.0f;
    float phase_offset = 0.0f;
    uint32_t phase_used = 0;

    for (uint32_t i = 0; i < sweep_count; i++) {
        float unwrapped = unwrap_phase(sweep_data[i].phase_deg, last_phase);
        float x = sweep_data[i].freq_hz - ref_freq;
        last_phase = unwrapped;

        if (sweep_data[i].mag < valid_min_mag) continue;

        float residual = unwrapped
            - rlc_phase_deg(best_model, sweep_data[i].freq_hz, best_f0, best_q);
        sum_x  += x;
        sum_y  += residual;
        sum_xx += x * x;
        sum_xy += x * residual;
        phase_used++;
    }

    if (phase_used > 1) {
        float den = (float)phase_used * sum_xx - sum_x * sum_x;
        if (fabsf(den) > 0.0f) {
            phase_slope  = ((float)phase_used * sum_xy - sum_x * sum_y) / den;
            phase_offset = (sum_y - phase_slope * sum_x) /
                           (float)phase_used - phase_slope * ref_freq;
        }
    }

    /* ── 计算模拟传递函数系数 ── */
    float w0 = 2.0f * PI_F * best_f0;
    float a  = w0 / best_q;
    float b  = w0 * w0;

    fit_result.model = best_model;
    fit_result.f0_hz = best_f0;
    fit_result.q     = best_q;
    fit_result.k     = best_k;
    fit_result.a     = a;
    fit_result.b     = b;
    fit_result.delay_us = (-phase_slope / 360.0f) * 1000000.0f;
    fit_result.phase0_deg = phase_offset;

    /* 按模型类型填充分子 */
    if (best_model == FIT_MODEL_HIGHPASS || best_model == FIT_MODEL_BANDSTOP) {
        fit_result.n2 = best_k;
    } else {
        fit_result.n2 = 0.0f;
    }

    if (best_model == FIT_MODEL_BANDPASS) {
        fit_result.n1 = best_k * a;
    } else {
        fit_result.n1 = 0.0f;
    }

    if (best_model == FIT_MODEL_LOWPASS || best_model == FIT_MODEL_BANDSTOP) {
        fit_result.n0 = best_k * b;
    } else {
        fit_result.n0 = 0.0f;
    }

    UART1_Printf("FIT_RESULT: %s f0=%.0f Q=%.2f K=%.4f delay=%.1fus\r\n",
                 sweep_learn_model_name(best_model), best_f0, best_q,
                 best_k, fit_result.delay_us);
    return 1U;
}

/* ══════════════════════════════════════════════════════════════
 * 双线性变换：模拟域 → 数字域 IIR 系数
 * H(s) = (n2*s² + n1*s + n0) / (s² + a*s + b)
 *  →  Direct Form I: y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
 * ══════════════════════════════════════════════════════════════ */

static uint8_t bilinear_transform(float n2, float n1, float n0,
                                   float a, float b, float fs,
                                   iir_coeff_t *coeff)
{
    float c  = 2.0f * fs;
    float c2 = c * c;

    /* 分母系数 */
    float d0 = c2 + a * c + b;
    float d1 = -2.0f * c2 + 2.0f * b;
    float d2 = c2 - a * c + b;

    /* 分子系数 */
    float m0 = n2 * c2 + n1 * c + n0;
    float m1 = -2.0f * n2 * c2 + 2.0f * n0;
    float m2 = n2 * c2 - n1 * c + n0;

    if (fabsf(d0) < 1.0e-9f) return 0;

    coeff->b0 = m0 / d0;
    coeff->b1 = m1 / d0;
    coeff->b2 = m2 / d0;
    coeff->a1 = d1 / d0;
    coeff->a2 = d2 / d0;
    coeff->valid = 1;

    return 1;
}

/* ══════════════════════════════════════════════════════════════
 * 公开接口
 * ══════════════════════════════════════════════════════════════ */

/* ── 模拟二阶带通：H(s) = (K*a*s) / (s² + a*s + b),  a=w0/Q, b=w0² ── */
void sweep_learn_sim_bandpass(float f0_hz, float q, float k)
{
    float w0 = 2.0f * PI_F * f0_hz;
    float a  = w0 / q;
    float b  = w0 * w0;

    /* 填充分子: n2=0, n1=K*a, n0=0 */
    fit_result.model     = FIT_MODEL_BANDPASS;
    fit_result.f0_hz     = f0_hz;
    fit_result.q         = q;
    fit_result.k         = k;
    fit_result.n2        = 0.0f;
    fit_result.n1        = k * a;
    fit_result.n0        = 0.0f;
    fit_result.a         = a;
    fit_result.b         = b;
    fit_result.delay_us  = 0.0f;
    fit_result.phase0_deg = 0.0f;

    /* 生成 IIR 系数 */
    bilinear_transform(fit_result.n2, fit_result.n1, fit_result.n0,
                       fit_result.a, fit_result.b, SAMPLE_RATE_HZ,
                       &iir_coeffs);
    s_has_learned = 1U;
}

/* 通知 ADC 完成（由 main.c 的 HAL_ADC_ConvCpltCallback 调用） */
void sweep_learn_notify_adc_done(uint8_t adc_id)
{
    if (adc_id == 1U) {
        sweep_adc1_done = 1U;
    } else if (adc_id == 2U) {
        sweep_adc2_done = 1U;
    }
}

void sweep_learn_notify_adc_error(uint8_t adc_id)
{
    (void)adc_id;
    sweep_adc_error = 1U;
}

/* 查询是否正在扫频（供 ADC 回调判断） */
uint8_t sweep_learn_is_sweeping(void)
{
    return s_sweeping;
}

/* 旧 DAC 扫频实现仅作历史参考，实际链路使用 AD9910。 */
#if 0
/* ── DAC 扫频激励波形生成 ── */
#define DAC_WAVE_SAMPLES  256U          /* DAC 波形点数 */
#define DAC_MID_CODE_F    2048.0f       /* DAC 中点（零电压） */
#define DAC_AMP_CODE      1240.0f       /* 激励幅度（约 60% 满量程） */

static uint16_t dac_wave_buf[DAC_WAVE_SAMPLES] __attribute__((aligned(32)));

/* 生成指定频率的 DAC 余弦波激励表 */
static void sweep_make_dac_wave(float freq_hz)
{
    float cycles = freq_hz * (float)DAC_WAVE_SAMPLES / SAMPLE_RATE_HZ;
    for (uint32_t n = 0; n < DAC_WAVE_SAMPLES; n++) {
        float angle = 2.0f * PI_F * cycles * (float)n / (float)DAC_WAVE_SAMPLES;
        float sample = DAC_MID_CODE_F + (DAC_AMP_CODE * cosf(angle));
        if (sample < 0.0f)      sample = 0.0f;
        if (sample > 4095.0f)   sample = 4095.0f;
        dac_wave_buf[n] = (uint16_t)(sample + 0.5f);
    }
}

/* ── 执行完整扫频+拟合+系数生成（阻塞 2-3 秒）── */
static void sweep_learn_run_legacy(void)
{
    extern DMA_HandleTypeDef hdma_dac1_ch1;

    /* 1. 停止现有外设 */
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    /* 2. ADC1 DMA 切换为 NORMAL 单次模式 */
    HAL_DMA_DeInit(&hdma_adc1);
    hdma_adc1.Init.Mode                = DMA_NORMAL;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        UART1_Printf("SWEEP_ERROR: DMA_Init failed\r\n");
        return;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* 3. ADC1 切换为 ONESHOT 模式 */
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_ONESHOT);
    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD, ADC_OVR_DATA_OVERWRITTEN);

    /* 4. DAC1 DMA 保持 CIRCULAR 模式（扫频激励持续输出） */
    HAL_DMA_DeInit(&hdma_dac1_ch1);
    hdma_dac1_ch1.Init.Mode                = DMA_CIRCULAR;
    hdma_dac1_ch1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    HAL_DMA_Init(&hdma_dac1_ch1);
    __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

    /* 5. 清空扫频数据 */
    sweep_count = 0;
    memset(&fit_result, 0, sizeof(fit_result));
    memset(&iir_coeffs, 0, sizeof(iir_coeffs));

    UART1_Printf("SWEEP_START: %lu-%lu Hz step %lu/%lu\r\n",
                 SWEEP_START_HZ, SWEEP_END_HZ,
                 SWEEP_FINE_STEP_HZ, SWEEP_HIGH_STEP_HZ);

    /* 6. 扫频循环 */
    for (uint32_t freq = SWEEP_START_HZ;
         freq <= SWEEP_END_HZ;
         freq += (freq < SWEEP_FINE_END_HZ) ?
                 SWEEP_FINE_STEP_HZ : SWEEP_HIGH_STEP_HZ) {
        /* 生成当前频率的 DAC 激励波形 */
        sweep_make_dac_wave((float)freq);
        SCB_CleanDCache_by_Addr((uint32_t *)dac_wave_buf,
                                (int32_t)sizeof(dac_wave_buf));

        /* 启动 DAC 循环输出 + ADC 单次采集 */
        sweep_adc_done = 0;
        s_sweeping = 1;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sweep_adc_buf,
                                          (int32_t)sizeof(sweep_adc_buf));

        if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                              (uint32_t *)dac_wave_buf, DAC_WAVE_SAMPLES,
                              DAC_ALIGN_12B_R) != HAL_OK) {
            UART1_Printf("SWEEP_ERROR: DAC_Start_DMA @%luHz\r\n", freq);
            s_sweeping = 0;
            continue;
        }
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)sweep_adc_buf,
                              SWEEP_SAMPLES) != HAL_OK) {
            UART1_Printf("SWEEP_ERROR: ADC_Start_DMA @%luHz\r\n", freq);
            HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
            s_sweeping = 0;
            continue;
        }
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start(&htim6);

        /* 等待 ADC 完成或超时 */
        uint32_t t0 = HAL_GetTick();
        while (sweep_adc_done == 0) {
            if ((HAL_GetTick() - t0) > SWEEP_ADC_TIMEOUT_MS) break;
        }

        HAL_TIM_Base_Stop(&htim6);
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        s_sweeping = 0;

        if (sweep_adc_done) {
            SCB_InvalidateDCache_by_Addr((uint32_t *)sweep_adc_buf,
                                         (int32_t)sizeof(sweep_adc_buf));
            float mag, phase;
            sweep_quadrature_demod(sweep_adc_buf, SWEEP_SAMPLES,
                                   (float)freq, SAMPLE_RATE_HZ, &mag, &phase);
            sweep_store((float)freq, mag, phase);
        }
    }

    UART1_Printf("SWEEP_DONE: %lu points\r\n", sweep_count);

    /* 7. 拟合 + 生成 IIR 系数 */
    if (sweep_count >= FIT_MIN_POINTS) {
        sweep_fit_model();
        bilinear_transform(fit_result.n2, fit_result.n1, fit_result.n0,
                          fit_result.a, fit_result.b, SAMPLE_RATE_HZ,
                          &iir_coeffs);
        if (iir_coeffs.valid) {
            UART1_Printf("IIR_COEFF: b0=%.6f b1=%.6f b2=%.6f a1=%.6f a2=%.6f\r\n",
                         iir_coeffs.b0, iir_coeffs.b1, iir_coeffs.b2,
                         iir_coeffs.a1, iir_coeffs.a2);
        } else {
            UART1_Printf("IIR_COEFF_ERROR\r\n");
        }
    }

    /* 8. 恢复 ADC1 DMA 为 CIRCULAR（不启动，由调用方负责） */
    HAL_DMA_DeInit(&hdma_adc1);
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    HAL_DMA_Init(&hdma_adc1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_CIRCULAR);
    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD, ADC_OVR_DATA_PRESERVED);

    UART1_Printf("SWEEP_DONE: %lu points, model=%s\r\n",
                 sweep_count, sweep_learn_model_name(fit_result.model));
}
#endif

static float sweep_actual_frequency(float requested_hz)
{
    uint32_t ftw = (uint32_t)((double)requested_hz *
                              (4294967296.0 / AD9910_SYSCLK_HZ));
    return (float)((double)ftw * AD9910_SYSCLK_HZ / 4294967296.0);
}

static uint32_t sweep_step_for_frequency(uint32_t freq_hz)
{
    return (freq_hz < SWEEP_FINE_END_HZ) ?
           SWEEP_FINE_STEP_HZ : SWEEP_HIGH_STEP_HZ;
}

static uint8_t sweep_prepare_adc_dma(void)
{
    HAL_DMA_DeInit(&hdma_adc1);
    HAL_DMA_DeInit(&hdma_adc2);

    hdma_adc1.Init.Mode     = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc2.Init.Mode     = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK ||
        HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
        return 0U;
    }

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT,
               ADC_CONVERSIONDATA_DMA_ONESHOT);
    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD,
               ADC_OVR_DATA_OVERWRITTEN);
    MODIFY_REG(hadc2.Instance->CFGR, ADC_CFGR_DMNGT,
               ADC_CONVERSIONDATA_DMA_ONESHOT);
    MODIFY_REG(hadc2.Instance->CFGR, ADC_CFGR_OVRMOD,
               ADC_OVR_DATA_OVERWRITTEN);

    return 1U;
}

static void sweep_restore_adc_dma(void)
{
    HAL_DMA_DeInit(&hdma_adc1);
    HAL_DMA_DeInit(&hdma_adc2);

    hdma_adc1.Init.Mode     = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.Mode     = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;

    (void)HAL_DMA_Init(&hdma_adc1);
    (void)HAL_DMA_Init(&hdma_adc2);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;

    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT,
               ADC_CONVERSIONDATA_DMA_CIRCULAR);
    MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD,
               ADC_OVR_DATA_PRESERVED);
    MODIFY_REG(hadc2.Instance->CFGR, ADC_CFGR_DMNGT,
               ADC_CONVERSIONDATA_DMA_CIRCULAR);
    MODIFY_REG(hadc2.Instance->CFGR, ADC_CFGR_OVRMOD,
               ADC_OVR_DATA_PRESERVED);
}

/* AD9910 逐点激励，TIM6_TRGO 同步采集 ADC1(输入) 与 ADC2(输出)。 */
void sweep_learn_run(void)
{
    HAL_StatusTypeDef adc1_status;
    HAL_StatusTypeDef adc2_status;
    fit_result_t previous_fit = fit_result;
    iir_coeff_t previous_coeffs = iir_coeffs;
    uint8_t previous_has_learned = s_has_learned;
    uint8_t learned_ok = 0U;

    s_sweeping = 0U;
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    if (!sweep_prepare_adc_dma()) {
        UART1_Printf("SWEEP_ERROR: ADC DMA init failed\r\n");
        sweep_restore_adc_dma();
        return;
    }

    sweep_count = 0U;
    memset(&fit_result, 0, sizeof(fit_result));
    memset(&iir_coeffs, 0, sizeof(iir_coeffs));

    UART1_Printf("SWEEP_START: %lu-%lu Hz, step %lu/%lu, dual ADC\r\n",
                 SWEEP_START_HZ, SWEEP_END_HZ,
                 SWEEP_FINE_STEP_HZ, SWEEP_HIGH_STEP_HZ);
    AD9910_AmpWrite(SWEEP_AD9910_AMP);

    for (uint32_t freq = SWEEP_START_HZ;
         freq <= SWEEP_END_HZ;
         freq += sweep_step_for_frequency(freq)) {
        float actual_hz = sweep_actual_frequency((float)freq);
        float xre, xim, yre, yim;
        float h_re, h_im, h_mag, h_phase;
        float denominator;
        uint32_t settle_ms;

        AD9910_FreWrite((double)freq);
        settle_ms = ((1000U * SWEEP_SETTLE_CYCLES) + freq - 1U) / freq;
        if (settle_ms < SWEEP_SETTLE_MS) settle_ms = SWEEP_SETTLE_MS;
        HAL_Delay(settle_ms);

        sweep_adc1_done = 0U;
        sweep_adc2_done = 0U;
        sweep_adc_error = 0U;
        s_sweeping = 1U;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sweep_adc1_buf,
                                           (int32_t)sizeof(sweep_adc1_buf));
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sweep_adc2_buf,
                                           (int32_t)sizeof(sweep_adc2_buf));

        adc1_status = HAL_ADC_Start_DMA(&hadc1,
                                        (uint32_t *)sweep_adc1_buf,
                                        SWEEP_SAMPLES);
        adc2_status = HAL_ADC_Start_DMA(&hadc2,
                                        (uint32_t *)sweep_adc2_buf,
                                        SWEEP_SAMPLES);
        if (adc1_status != HAL_OK || adc2_status != HAL_OK) {
            UART1_Printf("SWEEP_ERROR: ADC start @%luHz\r\n", freq);
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Stop_DMA(&hadc2);
            s_sweeping = 0U;
            continue;
        }

        __HAL_TIM_SET_COUNTER(&htim6, 0U);
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start(&htim6);

        uint32_t t0 = HAL_GetTick();
        while ((!sweep_adc1_done || !sweep_adc2_done) &&
               !sweep_adc_error) {
            if ((HAL_GetTick() - t0) > SWEEP_ADC_TIMEOUT_MS) {
                break;
            }
        }

        HAL_TIM_Base_Stop(&htim6);
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_ADC_Stop_DMA(&hadc2);
        s_sweeping = 0U;

        if (!sweep_adc1_done || !sweep_adc2_done || sweep_adc_error) {
            UART1_Printf("SWEEP_ERROR: ADC timeout/error @%luHz\r\n", freq);
            continue;
        }

        SCB_InvalidateDCache_by_Addr((uint32_t *)sweep_adc1_buf,
                                      (int32_t)sizeof(sweep_adc1_buf));
        SCB_InvalidateDCache_by_Addr((uint32_t *)sweep_adc2_buf,
                                      (int32_t)sizeof(sweep_adc2_buf));

        if (sweep_adc_is_clipped(sweep_adc1_buf, SWEEP_SAMPLES) ||
            sweep_adc_is_clipped(sweep_adc2_buf, SWEEP_SAMPLES)) {
            UART1_Printf("SWEEP_WARN: ADC clipping @%luHz\r\n", freq);
            continue;
        }

        sweep_quadrature_demod(sweep_adc1_buf, SWEEP_SAMPLES,
                               actual_hz, SAMPLE_RATE_HZ, &xre, &xim);
        sweep_quadrature_demod(sweep_adc2_buf, SWEEP_SAMPLES,
                               actual_hz, SAMPLE_RATE_HZ, &yre, &yim);

        denominator = xre * xre + xim * xim;
        if (denominator <= (SWEEP_MIN_INPUT_MAG * SWEEP_MIN_INPUT_MAG)) {
            UART1_Printf("SWEEP_WARN: input amplitude too low @%luHz\r\n", freq);
            continue;
        }

        h_re = (yre * xre + yim * xim) / denominator;
        h_im = (yim * xre - yre * xim) / denominator;
        h_mag = sqrtf(h_re * h_re + h_im * h_im);
        h_phase = atan2f(h_im, h_re) * 180.0f / PI_F;
        if (!isfinite(h_mag) || !isfinite(h_phase)) {
            UART1_Printf("SWEEP_WARN: invalid response @%luHz\r\n", freq);
            continue;
        }
        sweep_store(actual_hz, h_mag, h_phase);
        UART1_Printf("SWEEP_POINT: %.3f,%.6f,%.3f\r\n",
                     actual_hz, h_mag, h_phase);
    }

    AD9910_AmpWrite(0U);
    UART1_Printf("SWEEP_DONE: %lu points\r\n", sweep_count);

    if (sweep_count >= FIT_MIN_POINTS && sweep_fit_model()) {
        if (bilinear_transform(fit_result.n2, fit_result.n1, fit_result.n0,
                               fit_result.a, fit_result.b, SAMPLE_RATE_HZ,
                               &iir_coeffs) && iir_coeffs.valid) {
            learned_ok = 1U;
            UART1_Printf("IIR_COEFF: b0=%.6f b1=%.6f b2=%.6f a1=%.6f a2=%.6f\r\n",
                         iir_coeffs.b0, iir_coeffs.b1, iir_coeffs.b2,
                         iir_coeffs.a1, iir_coeffs.a2);
        } else {
            UART1_Printf("IIR_COEFF_ERROR\r\n");
        }
    }

    sweep_restore_adc_dma();
    if (learned_ok) {
        s_has_learned = 1U;
        UART1_Printf("SWEEP_READY: model=%s\r\n",
                     sweep_learn_model_name(fit_result.model));
    } else {
        /* 学习失败不能破坏当前可用参数，恢复默认值或上一次学习值。 */
        fit_result = previous_fit;
        iir_coeffs = previous_coeffs;
        s_has_learned = previous_has_learned;
        UART1_Printf("SWEEP_FAILED: keep %s model=%s\r\n",
                     s_has_learned ? "LEARNED" : "DEFAULT",
                     sweep_learn_model_name(fit_result.model));
    }
}

const iir_coeff_t *sweep_learn_get_coeffs(void)
{
    return &iir_coeffs;
}

const fit_result_t *sweep_learn_get_fit_result(void)
{
    return &fit_result;
}

uint8_t sweep_learn_has_learned(void)
{
    return s_has_learned;
}

uint32_t sweep_learn_get_point_count(void)
{
    return sweep_count;
}

const sweep_point_t *sweep_learn_get_point(uint32_t idx)
{
    if (idx >= sweep_count) return NULL;
    return &sweep_data[idx];
}
