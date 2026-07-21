#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* 系统工作模式 */
typedef enum {
    MODE_IDLE = 0,
    MODE_SINE_WAVE,
    MODE_CONTROL,
    MODE_LEARN,
    MODE_FILTER,
} SysMode_t;



/* 控制模式幅度额外乘数 */
#define CONTROL_AMP_MUL 1

/* 当前系统模式（外部可读） */
extern SysMode_t g_sys_mode;

/* 帧数据标志和最近一帧的原始值（main.c 中 APP_Proc 使用） */
extern bool     s_data_ready;
extern uint32_t s_last_vpp_raw;
extern uint32_t s_last_freq_raw;

/* 逐字节喂入串口接收数据 */
void Protocol_ParseByte(uint8_t byte);

/* Vpp 原始值 → AD9959 幅度值 */
uint16_t VppToAmp(uint32_t vpp_raw, uint8_t mul);

/* 幅值补偿查找表：根据频率 Hz 返回补偿系数 K(f) */
float AmpComp_GetK(uint32_t freq_hz);


#endif
