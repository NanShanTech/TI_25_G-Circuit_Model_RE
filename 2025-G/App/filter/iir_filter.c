#include "iir_filter.h"
#include <string.h>

#define ADC_MID_CODE   2048.0f
#define DAC_MID_CODE   2048.0f
#define DAC_MAX_CODE   4095.0f
#define DAC_GAIN_COMP  1.72f
#define INV_ADC_MID    0.00048828125f  /* 1/2048 */

static float b0, b1, b2;
static float a1, a2;
static float x1, x2;
static float y1, y2;
static uint8_t ready;

/* ═══════════════════════════════════════════════════════════ */

void iir_filter_init(float _b0, float _b1, float _b2,
                     float _a1, float _a2)
{
    b0 = _b0;  b1 = _b1;  b2 = _b2;
    a1 = _a1;  a2 = _a2;
    x1 = 0.0f;  x2 = 0.0f;
    y1 = 0.0f;  y2 = 0.0f;
    ready = 1;
}

/*
 * Direct Form I 双二阶, 手写融合循环.
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */
void iir_filter_process_block(const uint16_t *adc_buf, uint16_t *dac_buf,
                              uint32_t adc_offs, uint32_t dac_offs,
                              uint32_t count)
{
    if (!ready || count > IIR_BLOCK_SAMPLES) return;

    float _x1 = x1, _x2 = x2;
    float _y1 = y1, _y2 = y2;

    for (uint32_t i = 0; i < count; i++) {
        float x = ((float)adc_buf[adc_offs + i] - ADC_MID_CODE) * INV_ADC_MID;
        float y = b0 * x + b1 * _x1 + b2 * _x2 - a1 * _y1 - a2 * _y2;

        _x2 = _x1;  _x1 = x;
        _y2 = _y1;  _y1 = y;

        float dac = DAC_MID_CODE +
                    (y * (DAC_MID_CODE - 1.0f) *
                     IIR_OUTPUT_GAIN * DAC_GAIN_COMP);
        if (dac < 0.0f)       dac = 0.0f;
        if (dac > DAC_MAX_CODE) dac = DAC_MAX_CODE;
        dac_buf[dac_offs + i] = (uint16_t)(dac + 0.5f);
    }

    x1 = _x1;  x2 = _x2;
    y1 = _y1;  y2 = _y2;
}

uint8_t iir_filter_is_ready(void) { return ready; }

void iir_filter_reset(void)
{
    ready = 0;
    x1 = 0.0f;  x2 = 0.0f;
    y1 = 0.0f;  y2 = 0.0f;
    ready = 1;
}
