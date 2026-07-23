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
    uint32_t point_count = sweep_learn_get_point_count();
    const sweep_point_t *first_point;
    const sweep_point_t *last_point;
    float log_start;
    float log_span;
    float max_mag = 0.0f;
    uint8_t mag_curve[HMI_SWEEP_WIDTH];
    uint8_t phase_curve[HMI_SWEEP_WIDTH];

    if (point_count == 0U) return;
    first_point = sweep_learn_get_point(0U);
    last_point = sweep_learn_get_point(point_count - 1U);
    if (first_point == NULL || last_point == NULL ||
        first_point->freq_hz <= 0.0f || last_point->freq_hz <= 0.0f) {
        return;
    }

    log_start = logf(first_point->freq_hz);
    log_span = logf(last_point->freq_hz) - log_start;

    for (uint32_t i = 0U; i < point_count; i++) {
        const sweep_point_t *point = sweep_learn_get_point(i);
        if (point != NULL && point->mag > max_mag) max_mag = point->mag;
    }
    if (max_mag <= 0.0f) return;

    UART3_Printf("ref_stop\xff\xff\xff");
    HMI_WaveClear(HMI_SWEEP_WAVE_ID, 0);
    HMI_WaveClear(HMI_SWEEP_WAVE_ID, 1);

    uint32_t upper_index = (point_count > 1U) ? 1U : 0U;
    for (uint32_t x = 0U; x < HMI_SWEEP_WIDTH; x++) {
        float axis_fraction = (float)x / (float)(HMI_SWEEP_WIDTH - 1U);
        float target_freq = expf(log_start + log_span * axis_fraction);
        uint32_t index0;
        uint32_t index1;
        float fraction = 0.0f;
        const sweep_point_t *point0;
        const sweep_point_t *point1;
        float mag;
        float phase;
        uint32_t wire_index;

        while (upper_index < point_count) {
            const sweep_point_t *upper = sweep_learn_get_point(upper_index);
            if (upper != NULL && upper->freq_hz >= target_freq) break;
            upper_index++;
        }

        if (upper_index >= point_count) {
            index0 = point_count - 1U;
            index1 = index0;
        } else {
            index1 = upper_index;
            index0 = (index1 > 0U) ? index1 - 1U : 0U;
        }

        point0 = sweep_learn_get_point(index0);
        point1 = sweep_learn_get_point(index1);
        if (point0 == NULL || point1 == NULL) continue;

        if (point1->freq_hz > point0->freq_hz) {
            fraction = (target_freq - point0->freq_hz) /
                       (point1->freq_hz - point0->freq_hz);
            if (fraction < 0.0f) fraction = 0.0f;
            if (fraction > 1.0f) fraction = 1.0f;
        }

        mag = point0->mag + (point1->mag - point0->mag) * fraction;
        phase = hmi_sweep_phase_lerp(point0->phase_deg,
                                     point1->phase_deg, fraction);

        /* 波形控件先收到的数据位于右侧，因此按横坐标反序发送。 */
        wire_index = HMI_SWEEP_WIDTH - 1U - x;
        mag_curve[wire_index] = hmi_sweep_to_value(mag / max_mag);
        phase_curve[wire_index] =
            hmi_sweep_to_value((phase + 180.0f) / 360.0f);
    }

    if (!HMI_WaveAddBatch(HMI_SWEEP_WAVE_ID, 0U,
                          mag_curve, HMI_SWEEP_WIDTH) ||
        !HMI_WaveAddBatch(HMI_SWEEP_WAVE_ID, 1U,
                          phase_curve, HMI_SWEEP_WIDTH)) {
        UART1_Printf("HMI_ERROR: sweep curve transfer failed\r\n");
    }

    UART3_Printf("ref_star\xff\xff\xff");
}
