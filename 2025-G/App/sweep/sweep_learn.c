#include "sweep_learn.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "serial.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

/* ── 硬件句柄外部声明 ── */
extern DMA_HandleTypeDef hdma_adc1;

/* ── ADC/DAC 满量程 ── */
#define ADC_MID_CODE       2048.0f

/* ── 最大扫频点数 ── */
#define SWEEP_MAX_POINTS    (((SWEEP_END_HZ - SWEEP_START_HZ) / SWEEP_STEP_HZ) + 1U)

/* ── 算法用 π ── */
#define PI_F                3.14159265358979323846f

/*采样率*/
#define SAMPLE_RATE_HZ      1500000.0f

/* ── 32字节对齐（D-Cache 行大小） ── */
#define ADC_BUF_ALIGN       __attribute__((aligned(32)))

/* ══════════════════════════════════════════════════════════════
 * 静态数据
 * ══════════════════════════════════════════════════════════════ */

static uint16_t sweep_adc_buf[SWEEP_SAMPLES] ADC_BUF_ALIGN;

/* 扫频结果数组 */
static sweep_point_t sweep_data[SWEEP_MAX_POINTS];
static uint32_t      sweep_count;

/* ADC 完成标志（中断中置位） */
static volatile uint8_t sweep_adc_done;
/* 扫频进行中标志（供 ADC 回调判断） */
static volatile uint8_t s_sweeping;

/* 拟合结果 */
static fit_result_t fit_result;
static iir_coeff_t  iir_coeffs;

/* ══════════════════════════════════════════════════════════════
 * 内部辅助：正交解调
 * ══════════════════════════════════════════════════════════════ */

/* 对 adc_buf 做同频正交解调，返回归一化幅值和相位角 */
static void sweep_quadrature_demod(const uint16_t *buf, uint32_t n,
                                   float freq_hz, float fs_hz,
                                   float *mag, float *phase_deg)
{
    float sum   = 0.0f;
    float re    = 0.0f;
    float im    = 0.0f;
    float omega = 2.0f * PI_F * freq_hz / fs_hz;

    /* 去直流均值 */
    for (uint32_t i = 0; i < n; i++) {
        sum += (float)buf[i];
    }
    float mean = sum / (float)n;

    /* 正交投影 */
    for (uint32_t i = 0; i < n; i++) {
        float y     = (float)buf[i] - mean;
        float angle = omega * (float)i;
        re += y * cosf(angle);
        im -= y * sinf(angle);
    }

    /* 幅值：归一化到 [0, 1]（以 ADC 中点 2048 为参考） */
    float amp = 2.0f * sqrtf(re * re + im * im) / ((float)n * ADC_MID_CODE);
    *mag = amp;

    /* 相位 */
    *phase_deg = atan2f(im, re) * 180.0f / PI_F;
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
    while ((phase - last) > 180.0f)  phase -= 360.0f;
    while ((phase - last) < -180.0f) phase += 360.0f;
    return phase;
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

static void sweep_fit_model(void)
{
    float peak_mag = 0.0f;
    float best_f0  = 5000.0f;
    float best_q   = 1.0f;
    float best_k   = 1.0f;
    float best_err = 3.4e38f;
    uint32_t best_model  = FIT_MODEL_BANDPASS;
    float    valid_min_mag;

    if (sweep_count < FIT_MIN_POINTS) {
        UART1_Printf("FIT_ERROR: not enough points (%lu)\r\n", sweep_count);
        return;
    }

    /* 找峰值幅度 */
    for (uint32_t i = 0; i < sweep_count; i++) {
        if (sweep_data[i].mag > peak_mag) peak_mag = sweep_data[i].mag;
    }
    if (peak_mag <= 0.0f) {
        UART1_Printf("FIT_ERROR: bad data\r\n");
        return;
    }
    valid_min_mag = peak_mag * FIT_VALID_MAG_RATIO;

    /* 遍历 4 种模型 */
    for (uint32_t model = FIT_MODEL_LOWPASS; model <= FIT_MODEL_BANDSTOP; model++) {
        float model_best_f0  = sweep_data[sweep_count / 2].freq_hz;
        float model_best_q   = 1.0f;
        float model_best_k   = peak_mag;
        float model_best_err = 3.4e38f;
        float model_best_k_at_err = peak_mag;
        uint32_t model_best_used  = 0;

        /* 3 轮网格搜索：粗 → 中 → 精 */
        for (uint32_t pass = 0; pass < 3; pass++) {
            float f_min, f_max, q_min, q_max;

            if (pass == 0) {
                f_min = (float)SWEEP_START_HZ;
                f_max = (float)SWEEP_END_HZ;
                q_min = 0.10f;
                q_max = 20.0f;
            } else {
                float f_span = model_best_f0 * ((pass == 1) ? 0.35f : 0.12f);
                float q_span = model_best_q  * ((pass == 1) ? 0.55f : 0.20f);
                f_min = model_best_f0 - f_span;
                f_max = model_best_f0 + f_span;
                q_min = model_best_q  - q_span;
                q_max = model_best_q  + q_span;
                if (f_min < (float)SWEEP_START_HZ) f_min = (float)SWEEP_START_HZ;
                if (f_max > (float)SWEEP_END_HZ)   f_max = (float)SWEEP_END_HZ;
                if (q_min < 0.08f) q_min = 0.08f;
            }

            for (uint32_t fi = 0; fi <= 24; fi++) {
                float f0 = f_min + ((f_max - f_min) * (float)fi) / 24.0f;
                for (uint32_t qi = 0; qi <= 24; qi++) {
                    float q      = q_min + ((q_max - q_min) * (float)qi) / 24.0f;
                    float k_num  = 0.0f;
                    float k_den  = 0.0f;
                    float err    = 0.0f;
                    uint32_t used     = 0;
                    uint32_t err_used = 0;

                    /* 最小二乘求 K */
                    for (uint32_t i = 0; i < sweep_count; i++) {
                        if (model != FIT_MODEL_BANDSTOP && sweep_data[i].mag < valid_min_mag)
                            continue;
                        float base = rlc_mag_base(model, sweep_data[i].freq_hz, f0, q);
                        if (base > 1.0e-9f) {
                            k_num += sweep_data[i].mag * base;
                            k_den += base * base;
                            used++;
                        }
                    }
                    if (used < FIT_MIN_POINTS || k_den <= 0.0f) continue;
                    model_best_k = k_num / k_den;

                    /* 计算归一化误差 */
                    for (uint32_t i = 0; i < sweep_count; i++) {
                        if (model != FIT_MODEL_BANDSTOP && sweep_data[i].mag < valid_min_mag)
                            continue;
                        float base  = rlc_mag_base(model, sweep_data[i].freq_hz, f0, q);
                        float pred  = model_best_k * base;
                        float denom = sweep_data[i].mag;
                        if (denom < valid_min_mag) denom = valid_min_mag;
                        float e = (pred - sweep_data[i].mag) / denom;
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
                        model_best_used      = err_used;
                    }
                }
            }
        }
        if (model_best_err < best_err) {
            best_err   = model_best_err;
            best_model = model;
            best_f0    = model_best_f0;
            best_q     = model_best_q;
            best_k     = model_best_k_at_err;
        }
    }

    /* ── 相位延迟拟合 ── */
    float sum_x  = 0.0f;
    float sum_y  = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float last_phase = sweep_data[0].phase_deg;
    float phase_slope = 0.0f;
    float phase_offset = 0.0f;
    uint32_t phase_used = 0;

    for (uint32_t i = 0; i < sweep_count; i++) {
        float unwrapped = unwrap_phase(sweep_data[i].phase_deg, last_phase);
        last_phase = unwrapped;

        if (sweep_data[i].mag < valid_min_mag) continue;

        float residual = unwrapped
            - rlc_phase_deg(best_model, sweep_data[i].freq_hz, best_f0, best_q);
        sum_x  += sweep_data[i].freq_hz;
        sum_y  += residual;
        sum_xx += sweep_data[i].freq_hz * sweep_data[i].freq_hz;
        sum_xy += sweep_data[i].freq_hz * residual;
        phase_used++;
    }

    if (phase_used > 1) {
        float den = (float)phase_used * sum_xx - sum_x * sum_x;
        if (fabsf(den) > 0.0f) {
            phase_slope  = ((float)phase_used * sum_xy - sum_x * sum_y) / den;
            phase_offset = (sum_y - phase_slope * sum_x) / (float)phase_used;
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
    fit_result.model     = FIT_MODEL_HIGHPASS;
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
}

/* 通知 ADC 完成（由 main.c 的 HAL_ADC_ConvCpltCallback 调用） */
void sweep_learn_notify_adc_done(void)
{
    sweep_adc_done = 1;
}

/* 查询是否正在扫频（供 ADC 回调判断） */
uint8_t sweep_learn_is_sweeping(void)
{
    return s_sweeping;
}

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
void sweep_learn_run(void)
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

    UART1_Printf("SWEEP_START: %lu-%lu Hz step %lu\r\n",
                 SWEEP_START_HZ, SWEEP_END_HZ, SWEEP_STEP_HZ);

    /* 6. 扫频循环 */
    for (uint32_t freq = SWEEP_START_HZ; freq <= SWEEP_END_HZ; freq += SWEEP_STEP_HZ) {
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

const iir_coeff_t *sweep_learn_get_coeffs(void)
{
    return &iir_coeffs;
}

const fit_result_t *sweep_learn_get_fit_result(void)
{
    return &fit_result;
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
