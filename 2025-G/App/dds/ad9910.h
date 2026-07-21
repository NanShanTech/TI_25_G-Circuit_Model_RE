#ifndef __AD9910_H__
#define __AD9910_H__

#include "main.h"
#include <stdint.h>

#define AD9910_SYSCLK_HZ 999969103.4

/**
 * @brief AD9910 RAM 波形枚举
 */
typedef enum
{
    AD9910_WAVE_TRI = 0,   // 三角波
    AD9910_WAVE_SQR,       // 方波
    AD9910_WAVE_SINC,      // Sinc 波
} AD9910_WAVE_ENUM;

/* ---------------- AD9910 控制引脚重新映射 ---------------- */
/* 映射关系完全对齐 gpio.c 中的 MX_GPIO_Init 配置 */

// 电源与复位相关 (GPIOE)
#define AD9910_PWR_GPIO_Port      GPIOE
#define AD9910_PWR_Pin            GPIO_PIN_0// AD9910_PWR_Pin
#define AD9910_MAS_REST_GPIO_Port GPIOE
#define AD9910_MAS_REST_Pin       AD9910_RST_Pin

// SPI 通信相关 (GPIOE)
#define AD9910_SCLK_GPIO_Port     GPIOE
#define AD9910_SCLK_Pin           AD9910_CLK_Pin
#define AD9910_CS_GPIO_Port       GPIOE
#define AD9910_CS_Pin             AD9910_CSB_Pin
#define AD9910_SDIO_GPIO_Port     GPIOE
// #define AD9910_SDIO_Pin           AD9910_SDIO_Pin

// 更新与控制 (GPIOD)
#define AD9910_UP_DAT_GPIO_Port   GPIOD
#define AD9910_UP_DAT_Pin         AD9910_IOUP_Pin
#define AD9910_OSK_GPIO_Port      GPIOD
//#define AD9910_OSK_Pin            AD9910_OSK_Pin

// 数字斜率/扫描控制 (Digital Ramp Generator - DRG)
#define AD9910_DRHOLD_GPIO_Port   GPIOE
#define AD9910_DRHOLD_Pin         AD9910_DPH_Pin
#define AD9910_DROVER_GPIO_Port   GPIOC          // 根据 gpio.c，DRO 属于 GPIOC
#define AD9910_DROVER_Pin         AD9910_DRO_Pin
#define AD9910_DRCTL_GPIO_Port    GPIOD          // 根据 gpio.c，DRC 属于 GPIOD
#define AD9910_DRCTL_Pin          AD9910_DRC_Pin

// Profile 切换引脚 (GPIOB)
#define AD9910_PROFILE0_GPIO_Port GPIOB
#define AD9910_PROFILE0_Pin       AD9910_PF0_Pin
#define AD9910_PROFILE1_GPIO_Port GPIOB
#define AD9910_PROFILE1_Pin       AD9910_PF1_Pin
#define AD9910_PROFILE2_GPIO_Port GPIOB
#define AD9910_PROFILE2_Pin       AD9910_PF2_Pin

/* ---------------- GPIO 电平控制宏定义 ---------------- */

#define AD9910_PWR_0      HAL_GPIO_WritePin(AD9910_PWR_GPIO_Port, AD9910_PWR_Pin, GPIO_PIN_RESET)
#define AD9910_PWR_1      HAL_GPIO_WritePin(AD9910_PWR_GPIO_Port, AD9910_PWR_Pin, GPIO_PIN_SET)

#define AD9910_SDIO_0     HAL_GPIO_WritePin(AD9910_SDIO_GPIO_Port, AD9910_SDIO_Pin, GPIO_PIN_RESET)
#define AD9910_SDIO_1     HAL_GPIO_WritePin(AD9910_SDIO_GPIO_Port, AD9910_SDIO_Pin, GPIO_PIN_SET)

#define AD9910_SCLK_0     HAL_GPIO_WritePin(AD9910_SCLK_GPIO_Port, AD9910_SCLK_Pin, GPIO_PIN_RESET)
#define AD9910_SCLK_1     HAL_GPIO_WritePin(AD9910_SCLK_GPIO_Port, AD9910_SCLK_Pin, GPIO_PIN_SET)

#define AD9910_CS_0       HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_RESET)
#define AD9910_CS_1       HAL_GPIO_WritePin(AD9910_CS_GPIO_Port, AD9910_CS_Pin, GPIO_PIN_SET)

#define AD9910_MAS_REST_0 HAL_GPIO_WritePin(AD9910_MAS_REST_GPIO_Port, AD9910_MAS_REST_Pin, GPIO_PIN_RESET)
#define AD9910_MAS_REST_1 HAL_GPIO_WritePin(AD9910_MAS_REST_GPIO_Port, AD9910_MAS_REST_Pin, GPIO_PIN_SET)

#define AD9910_UP_DAT_0   HAL_GPIO_WritePin(AD9910_UP_DAT_GPIO_Port, AD9910_UP_DAT_Pin, GPIO_PIN_RESET)
#define AD9910_UP_DAT_1   HAL_GPIO_WritePin(AD9910_UP_DAT_GPIO_Port, AD9910_UP_DAT_Pin, GPIO_PIN_SET)

#define AD9910_DRCTL_0    HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_RESET)
#define AD9910_DRCTL_1    HAL_GPIO_WritePin(AD9910_DRCTL_GPIO_Port, AD9910_DRCTL_Pin, GPIO_PIN_SET)

#define AD9910_DPH_0    HAL_GPIO_WritePin(AD9910_DRHOLD_GPIO_Port, AD9910_DRHOLD_Pin, GPIO_PIN_RESET)
#define AD9910_DPH_1    HAL_GPIO_WritePin(AD9910_DRHOLD_GPIO_Port, AD9910_DRHOLD_Pin, GPIO_PIN_SET)

#define AD9910_PROFILE0_0 HAL_GPIO_WritePin(AD9910_PROFILE0_GPIO_Port, AD9910_PROFILE0_Pin, GPIO_PIN_RESET)
#define AD9910_PROFILE0_1 HAL_GPIO_WritePin(AD9910_PROFILE0_GPIO_Port, AD9910_PROFILE0_Pin, GPIO_PIN_SET)
#define AD9910_PROFILE1_0 HAL_GPIO_WritePin(AD9910_PROFILE1_GPIO_Port, AD9910_PROFILE1_Pin, GPIO_PIN_RESET)
#define AD9910_PROFILE1_1 HAL_GPIO_WritePin(AD9910_PROFILE1_GPIO_Port, AD9910_PROFILE1_Pin, GPIO_PIN_SET)
#define AD9910_PROFILE2_0 HAL_GPIO_WritePin(AD9910_PROFILE2_GPIO_Port, AD9910_PROFILE2_Pin, GPIO_PIN_RESET)
#define AD9910_PROFILE2_1 HAL_GPIO_WritePin(AD9910_PROFILE2_GPIO_Port, AD9910_PROFILE2_Pin, GPIO_PIN_SET)

/* ---------------- 函数声明 ---------------- */

void Init_AD9910(void);
void AD9910_FreWrite(double Freq);
void AD9910_AmpWrite(uint16_t Amp);
void AD9910_PhaWrite(float phase);
void AD9910_RAM_WAVE_Set(AD9910_WAVE_ENUM wave);

/* DRG 数字扫描发生器 */
void AD9910_DRG_FreInit_AutoSet(uint8_t autoSweepEn);
void AD9910_DRG_FrePara_Set(uint32_t lowFre, uint32_t upFre, uint32_t posStep, uint32_t negStep, uint16_t posRate, uint16_t negRate);

#endif /* __AD9910_H__ */
