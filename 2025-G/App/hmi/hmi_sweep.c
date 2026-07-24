#include "hmi_sweep.h"

#include "serial.h"
#include "sweep_learn.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define HMI_SWEEP_WAVE_ID  "s0.id"
#define HMI_SWEEP_WIDTH    300U
#define HMI_SWEEP_HEIGHT   200U
#define HMI_SWEEP_MAG_CH   0U
#define HMI_SWEEP_PHASE_CH 1U

static uint8_t hmi_sweep_mag_values[HMI_SWEEP_WIDTH];
static uint8_t hmi_sweep_phase_values[HMI_SWEEP_WIDTH];

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
    uint32_t point_count = sweep_learn_get_point_count();
    float max_mag = 0.0f;

    if (point_count == 0U) return;

    for (uint32_t i = 0U; i < point_count; i++) {
        const sweep_point_t *point = sweep_learn_get_point(i);

        if (point != NULL && isfinite(point->mag) && point->mag > max_mag) {
            max_mag = point->mag;
        }
    }
    if (max_mag <= 0.0f) max_mag = 1.0f;

    for (uint32_t x = 0U; x < HMI_SWEEP_WIDTH; x++) {
        float source_pos = 0.0f;
        uint32_t index0;
        uint32_t index1;
        float fraction;
        const sweep_point_t *point0;
        const sweep_point_t *point1;
        float mag;
        float phase;

        if (point_count > 1U) {
            source_pos = ((float)(HMI_SWEEP_WIDTH - 1U - x) *
                          (float)(point_count - 1U)) /
                         (float)(HMI_SWEEP_WIDTH - 1U);
        }

        index0 = (uint32_t)source_pos;
        index1 = (index0 + 1U < point_count) ? index0 + 1U : index0;
        fraction = source_pos - (float)index0;
        point0 = sweep_learn_get_point(index0);
        point1 = sweep_learn_get_point(index1);

        if (point0 == NULL || point1 == NULL) {
            mag = 0.0f;
            phase = 0.0f;
        } else {
            mag = point0->mag + (point1->mag - point0->mag) * fraction;
            phase = hmi_sweep_phase_lerp(point0->phase_deg,
                                         point1->phase_deg, fraction);
            if (!isfinite(mag)) mag = 0.0f;
            if (!isfinite(phase)) phase = 0.0f;
        }

        hmi_sweep_mag_values[x] = hmi_sweep_to_value(mag / max_mag);
        hmi_sweep_phase_values[x] =
            hmi_sweep_to_value((phase + 180.0f) / 360.0f);
    }

    HMI_WaveClear(HMI_SWEEP_WAVE_ID, HMI_SWEEP_MAG_CH);
    HMI_WaveClear(HMI_SWEEP_WAVE_ID, HMI_SWEEP_PHASE_CH);

    if (!HMI_WaveAddBatch(HMI_SWEEP_WAVE_ID, HMI_SWEEP_MAG_CH,
                          hmi_sweep_mag_values, HMI_SWEEP_WIDTH)) {
        return;
    }
    (void)HMI_WaveAddBatch(HMI_SWEEP_WAVE_ID, HMI_SWEEP_PHASE_CH,
                           hmi_sweep_phase_values, HMI_SWEEP_WIDTH);
}
