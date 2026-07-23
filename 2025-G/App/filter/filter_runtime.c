#include "filter_runtime.h"

#include "adc.h"
#include "dac.h"
#include "iir_filter.h"
#include "sweep_learn.h"
#include "tim.h"
#include <string.h>

#define FILTER_BUFFER_SAMPLES  256U
#define FILTER_HALF_SAMPLES    (FILTER_BUFFER_SAMPLES / 2U)
#define DAC_MID_CODE           2048U

static uint16_t s_adc_buffer[FILTER_BUFFER_SAMPLES]
    __attribute__((section(".AXI_SRAM"), aligned(32)));
static uint16_t s_dac_buffer[FILTER_BUFFER_SAMPLES]
    __attribute__((section(".AXI_SRAM"), aligned(32)));

static volatile uint8_t s_process_flags;
static volatile uint8_t s_overrun;
static volatile uint8_t s_active;

/* DMA 正在写另一半缓冲时，CPU 处理当前半缓冲并回写对应 DAC 区域。 */
static void filter_runtime_process_half(uint32_t offset)
{
    /* DMA 与 M7 D-Cache 不自动保持一致，读取 ADC 数据前必须失效缓存。 */
    SCB_InvalidateDCache_by_Addr((uint32_t *)&s_adc_buffer[offset],
                                 FILTER_HALF_SAMPLES * sizeof(uint16_t));

    if (iir_filter_is_ready()) {
        iir_filter_process_block(s_adc_buffer, s_dac_buffer, offset, offset,
                                 FILTER_HALF_SAMPLES);
    } else {
        for (uint32_t i = 0U; i < FILTER_HALF_SAMPLES; i++) {
            uint32_t value = (uint32_t)s_adc_buffer[offset + i] * 3U / 2U;
            if (value > 4095U) value = 4095U;
            s_dac_buffer[offset + i] = (uint16_t)value;
        }
    }

    /* 确保 CPU 写入的新数据对 DAC DMA 可见。 */
    SCB_CleanDCache_by_Addr((uint32_t *)&s_dac_buffer[offset],
                            FILTER_HALF_SAMPLES * sizeof(uint16_t));
}

filter_runtime_status_t filter_runtime_start(void)
{
    const iir_coeff_t *coeffs;

    /* 先停止公共触发源和相关 DMA，确保 ADC/DAC 从同一触发沿重新起步。 */
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    s_active = 0U;

    memset(s_adc_buffer, 0, sizeof(s_adc_buffer));
    for (uint32_t i = 0U; i < FILTER_BUFFER_SAMPLES; i++) {
        s_dac_buffer[i] = DAC_MID_CODE;
    }

    /* H7 的 DMA 与 D-Cache 不自动保持一致，启动前必须同步整个缓冲区。 */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)s_adc_buffer,
                                          (int32_t)sizeof(s_adc_buffer));
        SCB_CleanDCache_by_Addr((uint32_t *)s_dac_buffer,
                               (int32_t)sizeof(s_dac_buffer));
    }

    s_process_flags = 0U;
    s_overrun = 0U;

    coeffs = sweep_learn_get_coeffs();
    if (coeffs->valid) {
        iir_filter_init(coeffs->b0, coeffs->b1, coeffs->b2,
                        coeffs->a1, coeffs->a2);
    }

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc_buffer,
                          FILTER_BUFFER_SAMPLES) != HAL_OK) {
        return FILTER_RUNTIME_ADC_START_FAILED;
    }
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                          (uint32_t *)s_dac_buffer, FILTER_BUFFER_SAMPLES,
                          DAC_ALIGN_12B_R) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc2);
        return FILTER_RUNTIME_DAC_START_FAILED;
    }

    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc2);
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        return FILTER_RUNTIME_TIM_START_FAILED;
    }

    s_active = 1U;
    return FILTER_RUNTIME_OK;
}

void filter_runtime_stop(void)
{
    s_active = 0U;
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    s_process_flags = 0U;
}

void filter_runtime_process_pending(void)
{
    uint8_t flags;

    if (!s_active) return;

    /* 与 DMA 中断共享标志，取出并清零必须放在临界区。 */
    __disable_irq();
    flags = s_process_flags;
    s_process_flags = 0U;
    __enable_irq();

    if (flags & 0x01U) {
        filter_runtime_process_half(0U);
    }
    if (flags & 0x02U) {
        filter_runtime_process_half(FILTER_HALF_SAMPLES);
    }
}

uint8_t filter_runtime_on_adc_half_complete(ADC_HandleTypeDef *hadc)
{
    if (!s_active || hadc->Instance != ADC2) return 0U;

    if (s_process_flags & 0x01U) s_overrun = 1U;
    s_process_flags |= 0x01U;
    return 1U;
}

uint8_t filter_runtime_on_adc_complete(ADC_HandleTypeDef *hadc)
{
    if (!s_active || hadc->Instance != ADC2) return 0U;

    if (s_process_flags & 0x02U) s_overrun = 1U;
    s_process_flags |= 0x02U;
    return 1U;
}

uint8_t filter_runtime_had_overrun(void)
{
    return s_overrun;
}
