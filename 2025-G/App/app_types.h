#ifndef __APP_TYPES_H
#define __APP_TYPES_H

#include <stdint.h>


/** 波形类型枚举 */
typedef enum {
    WAVE_SINE = 0,      // 正弦波
    WAVE_SQUARE,        // 方波
    WAVE_TRIANGLE,      // 三角波
    WAVE_UNKNOWN        // 未知 / 初始状态
} WaveType_t;

/** 波形测量结果（频率 + 峰峰值 + 波形类型） */
typedef struct {
    float   Freq;       // 频率 (Hz)
    float   Vpp;        // 峰峰值 (V)
    WaveType_t Wave_type;
} Wave_Struct;

/** 测频模式 */
typedef enum {
    FMODE_PERIOD = 0,   // 测周法（低频高精度）
    FMODE_COUNT  = 1    // 测频法（高频）
} FreqMode_t;

#endif /* __APP_TYPES_H */
