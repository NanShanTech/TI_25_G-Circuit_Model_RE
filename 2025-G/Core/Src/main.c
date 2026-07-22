/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_types.h"
#include "fft_analyzer.h"
#include "scheduler.h"
#include "serial.h"
#include "protocol.h"
#include "ad9910.h"
#include "adc_app.h"
#include "sweep_learn.h"
#include "iir_filter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Wave_Struct   g_wave_info;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static __attribute__((section(".AXI_SRAM"))) uint16_t adc_1_buffer[FFT_N];
static __attribute__((section(".AXI_SRAM"))) uint16_t adc_2_buffer[FFT_N];//用于学习模式 两路ADC同步采样以获取幅相频曲线


#define FILTER_BUFFER_SAMPLES  256U
#define FILTER_HALF_SAMPLES    (FILTER_BUFFER_SAMPLES / 2U)
#define DAC_MID_CODE           2048U

static __attribute__((section(".AXI_SRAM"), aligned(32))) uint16_t adc_buffer[FILTER_BUFFER_SAMPLES];//滤波模式ADC缓冲
static __attribute__((section(".AXI_SRAM"), aligned(32))) uint16_t dac_buffer[FILTER_BUFFER_SAMPLES];//DAC缓冲

volatile uint8_t adc1_flag;
volatile uint8_t adc2_flag;//学习模式两路ADC同步采样标志

volatile uint8_t dac_filter_half;
volatile uint8_t iir_process_flags; // bit0=前半缓冲就绪(ConvHalfCplt) bit1=后半缓冲就绪(ConvCplt)
volatile uint8_t iir_overrun;      // 1=发生过半缓冲覆盖 主循环处理不及时

static void filter_process_half(uint32_t adc_offs, uint32_t dac_offs)
{
    SCB_InvalidateDCache_by_Addr(&adc_buffer[adc_offs],
                                 FILTER_HALF_SAMPLES * sizeof(uint16_t));

    if (iir_filter_is_ready()) {
        iir_filter_process_block(adc_buffer, dac_buffer, adc_offs, dac_offs,
                                 FILTER_HALF_SAMPLES);
    } else {
        for (uint16_t i = 0; i < FILTER_HALF_SAMPLES; i++) {
            uint32_t val = (uint32_t)adc_buffer[adc_offs + i] * 3 / 2;
            if (val > 4095) val = 4095;
            dac_buffer[dac_offs + i] = (uint16_t)val;
        }
    }

    SCB_CleanDCache_by_Addr(&dac_buffer[dac_offs],
                            FILTER_HALF_SAMPLES * sizeof(uint16_t));
}

     void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
     {
         if (hadc->Instance == ADC1 && g_sys_mode == MODE_FILTER) {
             if (iir_process_flags & 0x01U) iir_overrun = 1;
             iir_process_flags |= 0x01U;  // 仅置标志位，处理在主循环
         }
     }

     void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
     {
         /* 扫频模式：通知扫频模块 ADC 采集完成 */
         if (sweep_learn_is_sweeping()) {
             if (hadc->Instance == ADC1) {
                 sweep_learn_notify_adc_done();
             }
             return;
         }

         /* 滤波模式：后半 ADC 就绪，主循环写回后半 DAC */
         if (hadc->Instance == ADC1 && g_sys_mode == MODE_FILTER) {
             if (iir_process_flags & 0x02U) iir_overrun = 1;
             iir_process_flags |= 0x02U;  // 仅置标志位，处理在主循环
             return;
         }

         /* 普通模式：标记采集完成 */
         if (hadc->Instance == ADC1) {
             if (adc1_flag == 0) adc1_flag = 1;
         }
         if (hadc->Instance == ADC2) {
             if (adc2_flag == 0) adc2_flag = 1;
         }
     }

     void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
     {
         (void)hdac;
     }
     void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
     {
         (void)hdac;
         // DAC后半缓冲已消耗完
     }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  Serial_RxInit(&huart3);
  Scheduler_Init();
  Init_AD9910();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_1_buffer, FFT_N);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_2_buffer, FFT_N);
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 滤波模式高速轮询：中断仅置标志位，主循环中处理半缓冲 */
      if (g_sys_mode == MODE_FILTER) {
          uint8_t flags;
          __disable_irq();
          flags = iir_process_flags;
          iir_process_flags = 0;
          __enable_irq();
          if (flags & 0x01U) filter_process_half(0, 0);
          if (flags & 0x02U) {
              filter_process_half(FILTER_HALF_SAMPLES, FILTER_HALF_SAMPLES);
          }
      }
      Scheduler_Run();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void APP_Proc(void)
{
    static SysMode_t prev_mode = MODE_IDLE;

     //1. 退出动作：离开某模式时的清理工作
    if (prev_mode != g_sys_mode) {
        switch (prev_mode) {
        case MODE_FILTER:
            HAL_TIM_Base_Stop(&htim6);
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

            /* 恢复 ADC1 FFT 模式（ADC2 始终在运行，无需操作） */
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_1_buffer, FFT_N);

            __HAL_TIM_SET_COUNTER(&htim6, 0);
            __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
            HAL_TIM_Base_Start(&htim6);
            break;
        default:
            break;
        }
    }

     //2. 进入动作：进入某模式时的初始化工作
    if (prev_mode != g_sys_mode) {
        switch (g_sys_mode) {
        case MODE_LEARN: {
            /* 模拟二阶带通：f0=15kHz, Q=5, K=1.0 */
            sweep_learn_sim_bandpass(10000.0f, 5.0f, 5.0f);

            const iir_coeff_t *c = sweep_learn_get_coeffs();
            if (c->valid) {
                iir_filter_init(c->b0, c->b1, c->b2, c->a1, c->a2);
            }

            const fit_result_t *fit = sweep_learn_get_fit_result();
            UART3_Printf("t2.txt=\"%s\"\xff\xff\xff", sweep_learn_model_name(fit->model));
            UART3_Printf("tm0.en=0\xff\xff\xff");

            g_sys_mode = MODE_IDLE;
            break;
        }
        case MODE_FILTER: {
            /* 同步 ADC/DAC DMA：停 TIM6 → 停外设 → 清缓冲 → 同时启动 */
            HAL_TIM_Base_Stop(&htim6);
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

            memset(adc_buffer, 0, sizeof(adc_buffer));
            for (uint32_t i = 0; i < FILTER_BUFFER_SAMPLES; i++) {
                dac_buffer[i] = DAC_MID_CODE;
            }

            iir_process_flags = 0;
            iir_overrun = 0;

            const iir_coeff_t *c = sweep_learn_get_coeffs();
            if (c->valid) {
                iir_filter_init(c->b0, c->b1, c->b2, c->a1, c->a2);
            }

            /* ADC 和 DAC DMA 同时从 0 开始 */
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer,
                              FILTER_BUFFER_SAMPLES);
            HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                              (uint32_t *)dac_buffer, FILTER_BUFFER_SAMPLES,
                              DAC_ALIGN_12B_R);

            /* 复位 TIM6 计数器，ADC/DAC 从同一个触发沿起步 */
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
            HAL_TIM_Base_Start(&htim6);
            break;
        }
        default:
            break;
        }
        prev_mode = g_sys_mode;
    }


     //3. 稳态执行：当前模式的周期性工作
    switch (g_sys_mode) {
    case MODE_IDLE:
        AD9910_AmpWrite(0);
        break;

    case MODE_SINE_WAVE:
        if (s_data_ready) {
            s_data_ready = false;
            uint16_t amp = VppToAmp(s_last_vpp_raw, 1);
            uint32_t freq_hz = s_last_freq_raw / 10;
            AD9910_FreWrite(freq_hz);
            AD9910_AmpWrite(amp);
            UART3_Printf("t9.txt=\"%.1f\"\xff\xff\xff", s_last_vpp_raw * 0.1f);
            UART3_Printf("t10.txt=\"%.1f\"\xff\xff\xff", (float)freq_hz);
        }
        break;

    case MODE_CONTROL:
        if (s_data_ready) {
            s_data_ready = false;
            uint32_t freq_hz = s_last_freq_raw / 10;
            float comp_k = AmpComp_GetK(freq_hz);
            float amp_f = (float)VppToAmp(s_last_vpp_raw, CONTROL_AMP_MUL) * comp_k;
            if (amp_f > 16384.0f) amp_f = 16384.0f;
            uint16_t amp = (uint16_t)amp_f;
            AD9910_FreWrite(freq_hz);
            AD9910_AmpWrite(amp);
            UART3_Printf("t9.txt=\"%.1f\"\xff\xff\xff", s_last_vpp_raw * 0.1f);
            UART3_Printf("t10.txt=\"%.1f\"\xff\xff\xff", (float)freq_hz);
        }
        break;

    case MODE_LEARN:
        /* 一次性任务，进入时已执行完毕 */
        break;

    case MODE_FILTER:
        /* 滤波处理在主循环 while(1) 中高速轮询，不经过 10ms 任务 */
        break;

    default:
        break;
    }
}

 void UartProc(void)
{
    if (uart1_rx_flag) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)uart1_rx_buf, UART_RX_BUF_SIZE);
        uart1_rx_flag = 0;
        Serial_RxInit(&huart1);
    }

    if (uart3_rx_flag) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)uart3_rx_buf, UART_RX_BUF_SIZE);
        for (uint16_t i = 0; i < uart3_rx_len; i++) {
            Protocol_ParseByte(uart3_rx_buf[i]);
        }
        uart3_rx_flag = 0;
        Serial_RxInit(&huart3);
    }
}


/* 10ms 周期*/
void Task_10ms(uint16_t ticks)
{
    (void)ticks;
    APP_Proc();
    UartProc();
}

/* 100ms 周期 */
void Task_100ms(void)
{
}

/* 1 秒周期 */
void Task_1sec(void)
{
}

/* 1 分钟周期*/
void Task_1min(void)
{
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
