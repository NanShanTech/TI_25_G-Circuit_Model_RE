/**
 * @file    ad9959.c
 * @brief   AD9959 四通道 DDS 驱动实现
 *
 * 寄存器映射（指令字节 = 寄存器地址 + R/Wb，R/Wb=0 为写）：
 *   CSR  0x00 — 通道选择（bit4~7 对应 CH0~CH3）
 *   FR1  0x01 — 功能寄存器 1（VCO、PLL、调制电平）
 *   FR2  0x02 — 功能寄存器 2（扫频累加器使能等）
 *   CFR  0x03 — 通道功能寄存器（调制类型、扫频使能）
 *   CFTW 0x04 — 频率调谐字（32 位，FTW = freq × 2^32 / SYSCLK）
 *   CPOW 0x05 — 相位偏移字（14 位，phase / 16384 × 360° = 实际角度）
 *   ACR  0x06 — 幅度控制字（10 位 DAC，bit0~1 无效，bit2 必须为 1）
 *   LSRR 0x07 — 扫频斜率控制
 *   RDW  0x08 — 上升 delta 字
 *   FDW  0x09 — 下降 delta 字
 *   Profile0~15 0x0A~0x19 — 16 个 Profile（每个 8 字节）
 *
 * 关键参数：
 *   SYSCLK = 500 MHz（内部 PLL，25 MHz 晶振 × 20）
 *   频率分辨率 = 500 MHz / 2^32 ≈ 0.1164 Hz
 *   相位分辨率 = 14 位（16384 级，约 0.022°/LSB）
 *   幅度分辨率 = 10 位（1024 级，ASF = 0 时输出为 0）
 */

#include "ad9959.h"
#include <string.h>

/* ---- 简易微秒延时（DWT 周期计数器，首次调用自动初始化）---- */
static void delay_us(uint32_t us)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t target = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < target);
}

/* 频率因子 K = 2^32 / 500MHz，用于 Hz → FTW 转换 */
#define FREQ_FACTOR  8.589934592

/* 默认寄存器值 */
static const uint8_t FR1_DEF[3] = {0xD0, 0x00, 0x00};  /* 电荷泵 75µA，SYNC_CLK 禁用 */
static const uint8_t FR2_DEF[2] = {0x80, 0x00};        /* 单频模式，无扫频累加器 */
static const uint8_t CFR_DEF[3] = {0x00, 0x03, 0x02};  /* 正弦波，调制禁用，MSB 优先 */

/*----------------------------------------------------------------------------
 * SPI 底层
 *----------------------------------------------------------------------------*/
static void spi_write(uint8_t addr, uint8_t len, const uint8_t *data) {
    uint8_t byte = addr;
    AD9959_CS_0;
    for (int b = 0; b < 8; b++) {
        AD9959_SCLK_0;
        (byte & 0x80) ? AD9959_SDIO0_1 : AD9959_SDIO0_0;
        delay_us(5); AD9959_SCLK_1; delay_us(5);
        byte <<= 1;
    }
    for (int i = 0; i < len; i++) {
        byte = data[i];
        for (int b = 0; b < 8; b++) {
            AD9959_SCLK_0;
            (byte & 0x80) ? AD9959_SDIO0_1 : AD9959_SDIO0_0;
            delay_us(5); AD9959_SCLK_1; delay_us(5);
            byte <<= 1;
        }
    }
    AD9959_CS_1;
}

static void io_update(void) {
    AD9959_UPDATE_0; delay_us(1); AD9959_UPDATE_1; delay_us(1); AD9959_UPDATE_0;
}

static void chip_reset(void) {
    AD9959_RESET_0; delay_us(2);
    AD9959_RESET_1; delay_us(5);
    AD9959_RESET_0; delay_us(10);
}

/* 写频率调谐字到 CFTW (0x04)，freq_hz 单位 Hz */
static void write_ftw(uint32_t freq_hz) {
    uint32_t f = (uint32_t)(freq_hz * FREQ_FACTOR);
    uint8_t d[4] = {(uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f};
    spi_write(0x04, 4, d);
}

/* 写幅度到 ACR (0x06)，amp 范围 0~1023（10 位 DAC），bit2 必须置 1 使能乘法器 */
static void write_acr(uint16_t amp) {
    uint32_t a = amp | 0x1000;
    uint8_t d[3] = {0, (uint8_t)(a >> 8), (uint8_t)a};
    spi_write(0x06, 3, d);
}

/* 写相位偏移字到 CPOW (0x05)，phase 范围 0~16383（14 位） */
static void write_cpow(uint16_t phase) {
    uint8_t d[2] = {(uint8_t)(phase >> 8), (uint8_t)phase};
    spi_write(0x05, 2, d);
}

/*============================================================================
 * 公共 API
 *============================================================================*/

void AD9959_Init(void) {
    AD9959_PDC_0; AD9959_CS_1; AD9959_SCLK_0; AD9959_UPDATE_0;
    chip_reset();
    spi_write(0x01, 3, FR1_DEF);
    spi_write(0x02, 2, FR2_DEF);
    io_update();
}

void AD9959_Set_Freq(uint8_t channel, uint32_t freq_hz) {
    spi_write(0x00, 1, &channel);
    write_ftw(freq_hz);
}

void AD9959_Set_Amp(uint8_t channel, uint16_t amp) {
    spi_write(0x00, 1, &channel);
    write_acr(amp);
}

void AD9959_Set_Phase(uint8_t channel, uint16_t phase) {
    spi_write(0x00, 1, &channel);
    write_cpow(phase);
}

void AD9959_Set_All(uint8_t channel, uint32_t freq_hz, uint16_t amp, uint16_t phase) {
    spi_write(0x00, 1, &channel);
    write_ftw(freq_hz);
    write_acr(amp);
    write_cpow(phase);
}

void AD9959_Mod_Init(uint8_t channel, uint8_t mod_type, uint8_t sweep_en, uint8_t level) {
    uint8_t fr1[3], cfr[3];
    memcpy(fr1, FR1_DEF, 3); memcpy(cfr, CFR_DEF, 3);
    spi_write(0x00, 1, &channel);
    fr1[1] = level;
    cfr[0] = mod_type; cfr[1] |= sweep_en;
    spi_write(0x01, 3, fr1);
    spi_write(0x03, 3, cfr);
}

void AD9959_Sweep_Set(uint8_t channel, uint32_t start_hz, uint32_t end_hz,
                      uint32_t r_delta, uint32_t f_delta,
                      uint8_t rsrr, uint8_t fsrr,
                      uint16_t amp, uint16_t phase) {
    spi_write(0x00, 1, &channel);
    write_cpow(phase); write_acr(amp);
    uint8_t lsrr[2] = {fsrr, rsrr}; spi_write(0x07, 2, lsrr);
    uint32_t rd = (uint32_t)(r_delta * FREQ_FACTOR);
    uint8_t r[4] = {(uint8_t)(rd >> 24), (uint8_t)(rd >> 16), (uint8_t)(rd >> 8), (uint8_t)rd};
    spi_write(0x08, 4, r);
    uint32_t fd = (uint32_t)(f_delta * FREQ_FACTOR);
    uint8_t f[4] = {(uint8_t)(fd >> 24), (uint8_t)(fd >> 16), (uint8_t)(fd >> 8), (uint8_t)fd};
    spi_write(0x09, 4, f);
    write_ftw(start_hz);
    AD9959_Profile_Freq(0, end_hz);
}

/* 统一触发 IO_UPDATE，使所有已写入的通道参数同步生效 */
void AD9959_Update(void) {
    io_update();
}

void AD9959_Profile_Freq(uint8_t profile, uint32_t freq_hz) {
    uint32_t f = (uint32_t)(freq_hz * FREQ_FACTOR);
    uint8_t d[4] = {(uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f};
    spi_write(0x0A + profile, 4, d);
}

void AD9959_Profile_Amp(uint8_t profile, uint16_t amp) {
    uint8_t d[4] = {0, (uint8_t)(amp << 6), (uint8_t)(amp >> 2), 0};
    spi_write(0x0A + profile, 4, d);
}

void AD9959_Profile_Phase(uint8_t profile, uint16_t phase) {
    uint8_t d[4] = {0, (uint8_t)(phase << 2), (uint8_t)(phase >> 6), 0};
    spi_write(0x0A + profile, 4, d);
}
