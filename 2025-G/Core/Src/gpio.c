/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN (OSC32_IN)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT (OSC32_OUT)   ------> RCC_OSC32_OUT
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AD9910_CLK_Pin|AD9910_CSB_Pin|AD9910_DPH_Pin|AD9910_SDIO_Pin
                          |AD9910_PWR_Pin|AD9910_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AD9910_DRO_Pin|AD9959_UPDATE_Pin|AD9959_SP3_Pin|AD9959_CS_Pin
                          |AD9959_SDIO0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, AD9959_SP0_Pin|AD9959_PDC_Pin|AD9959_SP1_Pin|AD9959_RESET_Pin
                          |AD9959_SP2_Pin|AD9910_OSK_Pin|AD9910_IOUP_Pin|AD9910_DRC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AD9959_SCLK_Pin|AD9959_SDIO1_Pin|AD9959_SDIO3_Pin|AD9959_SDIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AD9910_PF0_Pin|AD9910_PF1_Pin|AD9910_PF2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AD9910_CLK_Pin */
  GPIO_InitStruct.Pin = AD9910_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD9910_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9910_CSB_Pin AD9910_DPH_Pin AD9910_SDIO_Pin AD9910_PWR_Pin
                           AD9910_RST_Pin */
  GPIO_InitStruct.Pin = AD9910_CSB_Pin|AD9910_DPH_Pin|AD9910_SDIO_Pin|AD9910_PWR_Pin
                          |AD9910_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9910_DRO_Pin AD9959_UPDATE_Pin AD9959_SP3_Pin AD9959_CS_Pin
                           AD9959_SDIO0_Pin */
  GPIO_InitStruct.Pin = AD9910_DRO_Pin|AD9959_UPDATE_Pin|AD9959_SP3_Pin|AD9959_CS_Pin
                          |AD9959_SDIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9959_SP0_Pin AD9959_PDC_Pin AD9959_SP1_Pin AD9959_RESET_Pin
                           AD9959_SP2_Pin AD9910_OSK_Pin AD9910_IOUP_Pin AD9910_DRC_Pin */
  GPIO_InitStruct.Pin = AD9959_SP0_Pin|AD9959_PDC_Pin|AD9959_SP1_Pin|AD9959_RESET_Pin
                          |AD9959_SP2_Pin|AD9910_OSK_Pin|AD9910_IOUP_Pin|AD9910_DRC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9959_SCLK_Pin AD9959_SDIO1_Pin AD9959_SDIO3_Pin AD9959_SDIO2_Pin */
  GPIO_InitStruct.Pin = AD9959_SCLK_Pin|AD9959_SDIO1_Pin|AD9959_SDIO3_Pin|AD9959_SDIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9910_PF0_Pin AD9910_PF1_Pin AD9910_PF2_Pin */
  GPIO_InitStruct.Pin = AD9910_PF0_Pin|AD9910_PF1_Pin|AD9910_PF2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
