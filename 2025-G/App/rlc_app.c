#include "rlc_app.h"

#include "adc.h"
#include "ad9910.h"
#include "dac.h"
#include "fft_analyzer.h"
#include "filter_runtime.h"
#include "gpio.h"
#include "hmi_sweep.h"
#include "iir_filter.h"
#include "main.h"
#include "protocol.h"
#include "scheduler.h"
#include "serial.h"
#include "sweep_learn.h"
#include "tim.h"
#include "usart.h"

static uint16_t s_adc1_buffer[FFT_N]
    __attribute__((section(".AXI_SRAM")));
static uint16_t s_adc2_buffer[FFT_N]
    __attribute__((section(".AXI_SRAM")));

volatile uint8_t adc1_flag;
volatile uint8_t adc2_flag;

/* 空闲、波形输出和控制模式共用普通双 ADC 采样链路。 */
static void rlc_app_start_normal_sampling(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_buffer, FFT_N);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_buffer, FFT_N);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start(&htim6);
}

static void rlc_app_exit_mode(SysMode_t mode)
{
    /* 离开滤波模式时先停止实时链路，再恢复普通采样。 */
    if (mode == MODE_FILTER) {
        filter_runtime_stop();
        rlc_app_start_normal_sampling();
    }
}

static void rlc_app_update_relay(SysMode_t mode)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,
                      (mode == MODE_FILTER) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rlc_app_enter_learn_mode(void)
{
    const iir_coeff_t *coeffs;
    const fit_result_t *fit;

    sweep_learn_run();

    //hmi_sweep_draw_curves();  //串口屏绘制幅频响应曲线
    
    coeffs = sweep_learn_get_coeffs();
    fit = sweep_learn_get_fit_result();
    if (coeffs->valid) {
        iir_filter_init(coeffs->b0, coeffs->b1, coeffs->b2,
                        coeffs->a1, coeffs->a2);
        UART3_Printf("t2.txt=\"%s\"\xff\xff\xff",
                     sweep_learn_model_name(fit->model));
    } else {
        UART3_Printf("t2.txt=\"FIT ERROR\"\xff\xff\xff");
    }
    UART3_Printf("tm0.en=0\xff\xff\xff");

    rlc_app_start_normal_sampling();
    g_sys_mode = MODE_IDLE;
}

static void rlc_app_enter_mode(SysMode_t mode)
{
    rlc_app_update_relay(mode);

    switch (mode) {
    case MODE_LEARN:
        rlc_app_enter_learn_mode();
        break;

    case MODE_FILTER: {
        filter_runtime_status_t status;

        UART1_Printf("FILTER_START: ADC=PB0 DAC=PA4 COEFF=%s SOURCE=%s\r\n",
                     sweep_learn_get_coeffs()->valid ? "VALID" : "INVALID",
                     sweep_learn_has_learned() ? "LEARNED" : "DEFAULT");
        status = filter_runtime_start();
        if (status != FILTER_RUNTIME_OK) {
            UART1_Printf("FILTER_FAIL: stage=%u ADC_ERR=0x%08lX DAC_ERR=0x%08lX\r\n",
                         (unsigned int)status,
                         (unsigned long)HAL_ADC_GetError(&hadc2),
                         (unsigned long)HAL_DAC_GetError(&hdac1));
            rlc_app_start_normal_sampling();
            g_sys_mode = MODE_IDLE;
            rlc_app_update_relay(g_sys_mode);
        }
        break;
    }

    default:
        break;
    }
}

static void rlc_app_process_current_mode(void)
{
    switch (g_sys_mode) {
    case MODE_IDLE:
        AD9910_AmpWrite(0U);
        break;

    case MODE_SINE_WAVE:
        if (s_data_ready) {
            uint16_t amp;
            uint32_t freq_hz;

            s_data_ready = false;
            amp = VppToAmp(s_last_vpp_raw, 1U);
            freq_hz = s_last_freq_raw / 10U;
            AD9910_FreWrite(freq_hz);
            AD9910_AmpWrite(amp);
            UART3_Printf("t9.txt=\"%.1f\"\xff\xff\xff",
                         s_last_vpp_raw * 0.1f);
            UART3_Printf("t10.txt=\"%.1f\"\xff\xff\xff",
                         (float)freq_hz);
        }
        break;

    case MODE_CONTROL:
        if (s_data_ready) {
            uint32_t freq_hz;
            float amp_value;

            s_data_ready = false;
            freq_hz = s_last_freq_raw / 10U;
            amp_value = (float)VppToAmp(s_last_vpp_raw, CONTROL_AMP_MUL) *
                        AmpComp_GetK(freq_hz);
            if (amp_value > 16384.0f) amp_value = 16384.0f;
            AD9910_FreWrite(freq_hz);
            AD9910_AmpWrite((uint16_t)amp_value);
            UART3_Printf("t9.txt=\"%.1f\"\xff\xff\xff",
                         s_last_vpp_raw * 0.1f);
            UART3_Printf("t10.txt=\"%.1f\"\xff\xff\xff",
                         (float)freq_hz);
        }
        break;

    case MODE_LEARN:
    case MODE_FILTER:
    default:
        break;
    }
}

static void rlc_app_process_mode(void)
{
    static SysMode_t previous_mode = MODE_IDLE;

    /* 模式变化时严格按照“退出旧模式 -> 进入新模式”的顺序处理。 */
    if (previous_mode != g_sys_mode) {
        rlc_app_exit_mode(previous_mode);
        rlc_app_enter_mode(g_sys_mode);
        previous_mode = g_sys_mode;
    }

    rlc_app_process_current_mode();
}

static void rlc_app_process_uart(void)
{
    if (uart1_rx_flag) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)uart1_rx_buf,
                                     UART_RX_BUF_SIZE);
        uart1_rx_flag = 0U;
        Serial_RxInit(&huart1);
    }

    if (uart3_rx_flag) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)uart3_rx_buf,
                                     UART_RX_BUF_SIZE);
        for (uint16_t i = 0U; i < uart3_rx_len; i++) {
            Protocol_ParseByte(uart3_rx_buf[i]);
        }
        uart3_rx_flag = 0U;
        Serial_RxInit(&huart3);
    }
}

void rlc_app_init(void)
{
    rlc_app_update_relay(g_sys_mode);
    Serial_RxInit(&huart3);
    Scheduler_Init();
    Init_AD9910();

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    rlc_app_start_normal_sampling();
}

void rlc_app_task(void)
{
    /* 实时滤波优先处理，避免等待 10 ms 调度周期导致 DMA 覆盖。 */
    if (g_sys_mode == MODE_FILTER) {
        filter_runtime_process_pending();
    }
    Scheduler_Run();
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (g_sys_mode == MODE_FILTER) {
        (void)filter_runtime_on_adc_half_complete(hadc);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 扫频期间 ADC1/ADC2 都属于扫频模块，完成事件不能落入普通模式。 */
    if (sweep_learn_is_sweeping()) {
        if (hadc->Instance == ADC1) {
            sweep_learn_notify_adc_done(1U);
        } else if (hadc->Instance == ADC2) {
            sweep_learn_notify_adc_done(2U);
        }
        return;
    }

    if (g_sys_mode == MODE_FILTER &&
        filter_runtime_on_adc_complete(hadc)) {
        return;
    }

    /* 普通模式仅记录采样完成，数据处理由对应的周期任务负责。 */
    if (hadc->Instance == ADC1 && adc1_flag == 0U) {
        adc1_flag = 1U;
    }
    if (hadc->Instance == ADC2 && adc2_flag == 0U) {
        adc2_flag = 1U;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (sweep_learn_is_sweeping()) {
        sweep_learn_notify_adc_error((hadc->Instance == ADC2) ? 2U : 1U);
    }
}

void Task_10ms(uint16_t ticks)
{
    (void)ticks;
    rlc_app_process_mode();
    rlc_app_process_uart();
}

void Task_100ms(void)
{
}

void Task_1sec(void)
{
}

void Task_1min(void)
{
}
