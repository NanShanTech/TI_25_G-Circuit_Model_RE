#include "filter_runtime.h"

#include "adc.h"
#include "dac.h"
#include "iir_filter.h"
#include "sweep_learn.h"
#include "tim.h"
#include <string.h>
#include "Serial.h"

#define FILTER_HALF_SAMPLES    256U
#define FILTER_BUFFER_SAMPLES  (FILTER_HALF_SAMPLES * 2U)
#define DAC_MID_CODE           2048U
#define DAC_MAX_CODE           4095U

static uint16_t s_adc_buffer[FILTER_BUFFER_SAMPLES]
    __attribute__((section(".AXI_SRAM"), aligned(32)));
static uint16_t s_dac_buffer[FILTER_BUFFER_SAMPLES]
    __attribute__((section(".AXI_SRAM"), aligned(32)));

static volatile uint8_t s_process_flags;
static volatile uint8_t s_overrun;
static volatile uint8_t s_active;

/* 扫频和普通采样可能改写 DAC 状态，进入滤波模式时重新建立完整输出链路。 */
static uint8_t filter_runtime_prepare_dac(void)
{
    extern DMA_HandleTypeDef hdma_dac1_ch1;
    DAC_ChannelConfTypeDef config = {0};

    config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    config.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &config, DAC_CHANNEL_1) != HAL_OK) {
        return 0U;
    }

    if (HAL_DMA_DeInit(&hdma_dac1_ch1) != HAL_OK) {
        return 0U;
    }

    hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK) {
        return 0U;
    }

    __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);
    return 1U;
}

/* DMA 正在写另一半缓冲时，CPU 处理当前半缓冲并回写对应 DAC 区域。 */
static void filter_runtime_process_half(uint32_t offset)
{
    /* DMA 与 M7 D-Cache 不自动保持一致，读取 ADC 数据前必须失效缓存。 */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)&s_adc_buffer[offset],
                                     FILTER_HALF_SAMPLES * sizeof(uint16_t));
    }

    if (iir_filter_is_ready()) {
        iir_filter_process_block(s_adc_buffer, s_dac_buffer, offset, offset,
                                 FILTER_HALF_SAMPLES);
    } //根据学习的参数进行IIR滤波处理
    
    else {
        for (uint32_t i = 0U; i < FILTER_HALF_SAMPLES; i++) {
            int32_t centered = (int32_t)s_adc_buffer[offset + i] -
                               (int32_t)DAC_MID_CODE;
            int32_t value = (int32_t)DAC_MID_CODE + centered;
            if (value < 0) value = 0;
            if (value > (int32_t)DAC_MAX_CODE) value = DAC_MAX_CODE;
            s_dac_buffer[offset + i] = (uint16_t)value;
        }
    }

    /* 确保 CPU 写入的新数据对 DAC DMA 可见。 */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache_by_Addr((uint32_t *)&s_dac_buffer[offset],
                                FILTER_HALF_SAMPLES * sizeof(uint16_t));
    }
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

    if (!filter_runtime_prepare_dac()) {
        return FILTER_RUNTIME_DAC_START_FAILED;
    }

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

    /* 实时流发生偶发过载时保留最新样本，避免 ADC DMA 停在旧数据上。 */
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    MODIFY_REG(hadc2.Instance->CFGR, ADC_CFGR_DMNGT,
               ADC_CONVERSIONDATA_DMA_CIRCULAR);
    MODIFY_REG(hadc2.Instance->CFGR, ADC_CFGR_OVRMOD,
               ADC_OVR_DATA_OVERWRITTEN);

    /* 首次触发先输出中点，随后由循环 DMA 连续更新 PA4。 */
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                         DAC_MID_CODE) != HAL_OK) {
        return FILTER_RUNTIME_DAC_START_FAILED;
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
