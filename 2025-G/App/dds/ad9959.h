/**
 * @file    ad9959.h
 * @brief   AD9959 四通道 DDS 驱动 API（SPI 软件模拟）
 * 
 * 移植时需修改：此文件底部的管脚重映射宏
 */

#ifndef __AD9959_H
#define __AD9959_H

#include "main.h"
#include <stdint.h>

/* ===== 通道掩码 ===== */
#define CH0  0x10
#define CH1  0x20
#define CH2  0x40
#define CH3  0x80

/* ===== 调制类型 ===== */
#define MOD_DISABLE  0x00
#define MOD_ASK      0x40
#define MOD_FSK      0x80
#define MOD_PSK      0xC0

/* ===== 扫频 ===== */
#define SWEEP_EN   0x40
#define SWEEP_DIS  0x00

/* ===== 调制电平 ===== */
#define LEVEL_2   0x00
#define LEVEL_4   0x01
#define LEVEL_8   0x02
#define LEVEL_16  0x03

/*----------------------------------------------------------------------------
 * 管脚重映射 —— 在此修改为你的实际接线
 *----------------------------------------------------------------------------*/
#define AD9959_CS_1      HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET)
#define AD9959_CS_0      HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET)
#define AD9959_SCLK_1    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin, GPIO_PIN_SET)
#define AD9959_SCLK_0    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin, GPIO_PIN_RESET)
#define AD9959_SDIO0_1   HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin, GPIO_PIN_SET)
#define AD9959_SDIO0_0   HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin, GPIO_PIN_RESET)
#define AD9959_UPDATE_1  HAL_GPIO_WritePin(AD9959_UPDATE_GPIO_Port, AD9959_UPDATE_Pin, GPIO_PIN_SET)
#define AD9959_UPDATE_0  HAL_GPIO_WritePin(AD9959_UPDATE_GPIO_Port, AD9959_UPDATE_Pin, GPIO_PIN_RESET)
#define AD9959_RESET_1   HAL_GPIO_WritePin(AD9959_RESET_GPIO_Port, AD9959_RESET_Pin, GPIO_PIN_SET)
#define AD9959_RESET_0   HAL_GPIO_WritePin(AD9959_RESET_GPIO_Port, AD9959_RESET_Pin, GPIO_PIN_RESET)
#define AD9959_PDC_1     HAL_GPIO_WritePin(AD9959_PDC_GPIO_Port, AD9959_PDC_Pin, GPIO_PIN_SET)
#define AD9959_PDC_0     HAL_GPIO_WritePin(AD9959_PDC_GPIO_Port, AD9959_PDC_Pin, GPIO_PIN_RESET)

/*----------------------------------------------------------------------------
 * 公共 API
 *
 * 使用规范：
 *   1. 先调用 Set 系列函数写入各通道参数（写入缓冲寄存器，不立即生效）
 *   2. 所有通道配置完毕后，调用 AD9959_Update() 统一触发 IO_UPDATE，
 *      所有通道参数同步转移到工作寄存器，同时启动输出
 *
 * 参数单位：
 *   freq_hz    — 频率，单位 Hz，范围由 SYSCLK 决定（500MHz 时最大约 200MHz）
 *   amp        — 幅度，10 位 DAC 满幅 = 1023（12 位寄存器，低 2 位无效）
 *   phase      — 相位偏移字，14 位 (0 ~ 16383)，phase / 16384 × 360° = 实际角度
 *                常用值：0°=0  90°=4096  180°=8192  270°=12288
 *----------------------------------------------------------------------------*/
void AD9959_Init(void);
void AD9959_Set_Freq(uint8_t channel, uint32_t freq_hz);
void AD9959_Set_Amp(uint8_t channel, uint16_t amplitude);
void AD9959_Set_Phase(uint8_t channel, uint16_t phase);
void AD9959_Set_All(uint8_t channel, uint32_t freq_hz, uint16_t amp, uint16_t phase);
void AD9959_Mod_Init(uint8_t channel, uint8_t mod_type, uint8_t sweep_en, uint8_t level);
void AD9959_Sweep_Set(uint8_t channel, uint32_t start_hz, uint32_t end_hz,
                      uint32_t r_delta, uint32_t f_delta,
                      uint8_t rsrr, uint8_t fsrr,
                      uint16_t amp, uint16_t phase);
void AD9959_Update(void);
void AD9959_Profile_Freq(uint8_t profile, uint32_t freq_hz);
void AD9959_Profile_Amp(uint8_t profile, uint16_t amp);
void AD9959_Profile_Phase(uint8_t profile, uint16_t phase);

#endif
