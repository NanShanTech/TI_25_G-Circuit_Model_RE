#include "hmi_sweep.h"

#include "serial.h"
#include "sweep_learn.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define HMI_SWEEP_WAVE_ID  "s0.id"
#define HMI_SWEEP_WIDTH    300U
#define HMI_SWEEP_HEIGHT   200U

static uint8_t hmi_sweep_to_value(float normalized)
{
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    return (uint8_t)(normalized * (float)HMI_SWEEP_HEIGHT + 0.5f);
}

static float hmi_sweep_phase_lerp(float phase0, float phase1, float fraction)
{
    float delta = phase1 - phase0;

    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    phase0 += delta * fraction;
    if (phase0 > 180.0f) phase0 -= 360.0f;
    if (phase0 < -180.0f) phase0 += 360.0f;

    return phase0;
}

void hmi_sweep_draw_curves(void)
{



    
}
