#include "protocol.h"
#include "ad9959.h"
#include <string.h>
#include <stdbool.h>

#define FRAME_HEAD1     0x55    /* 帧头第一字节 */
#define FRAME_HEAD2     0xAA    /* 帧头第二字节 */
#define FRAME_TAIL      0xFF    /* 帧尾字节（连续两个） */
#define FRAME_PAYLOAD   8       /* Vpp(4) + Freq(4) */

#define CMD_BUF_SIZE    16      /* 命令字符串最大长度 */

SysMode_t g_sys_mode = MODE_IDLE;

typedef enum {
    FSM_IDLE,
    FSM_HEAD_AA,        /* 收到 0x55，等 0xAA */
    FSM_DATA,           /* 收 8 字节数据负载 */
    FSM_TAIL_FF1,       /* 等第一个 0xFF */
    FSM_TAIL_FF2,       /* 等第二个 0xFF，完成 */
} FrameState_t;

static FrameState_t s_fsm = FSM_IDLE;
static uint8_t  s_payload[FRAME_PAYLOAD];
static uint8_t  s_payload_idx;

bool     s_data_ready;       /* 有新帧待处理 */
uint32_t s_last_vpp_raw;     /* 上一次的 Vpp 原始值（用于变化检测） */
uint32_t s_last_freq_raw;    /* 上一次的 Freq 原始值 */


static char    s_cmd_buf[CMD_BUF_SIZE];
static uint8_t s_cmd_idx;

/* 尝试匹配命令缓冲中的字符串，匹配成功则切换模式 */
static void Cmd_TryMatch(void)
{
    const char *cmd = s_cmd_buf;

    if (strcmp(cmd, "default") == 0) {
        g_sys_mode = MODE_IDLE;
    } else if (strcmp(cmd, "sinewave") == 0) {
        g_sys_mode = MODE_SINE_WAVE;
        s_last_vpp_raw  = 0;
        s_last_freq_raw = 0;
    } else if (strcmp(cmd, "control") == 0) {
        g_sys_mode = MODE_CONTROL;
        s_last_vpp_raw  = 0;
        s_last_freq_raw = 0;
    } else if (strcmp(cmd, "start_learn") == 0) {
        g_sys_mode = MODE_LEARN;
    } else if (strcmp(cmd, "start_filter") == 0) {
        g_sys_mode = MODE_FILTER;
    } else {
        /* 不匹配任何命令，继续累积 */
        return;
    }

    /* 匹配成功，清除待处理数据并清空命令缓冲 */
    s_data_ready = false;
    s_cmd_idx = 0;
    s_cmd_buf[0] = '\0';
}

/* 解析负载中的 Vpp 和 Freq（小端序 uint32_t） */
static void Frame_ParsePayload(const uint8_t *buf, uint32_t *vpp, uint32_t *freq)
{
    *vpp  = (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);

    *freq = (uint32_t)buf[4]
         | ((uint32_t)buf[5] << 8)
         | ((uint32_t)buf[6] << 16)
         | ((uint32_t)buf[7] << 24);
}

uint16_t VppToAmp(uint32_t vpp_raw, uint8_t mul)
{
    float vpp = (float)(int32_t)vpp_raw * 0.1f;   /* 0.1V → V */
    float amp = vpp * 2000;                   /* V → DAC 码 */
    amp *= (float)mul;
    if (amp < 0.0f) amp = 0.0f;
    if (amp > 1023.0f) amp = 1023.0f;
    return (uint16_t)amp;
}

void Protocol_ParseByte(uint8_t byte)
{
    /* ---- 二进制帧解析（优先）---- */
    if (byte == FRAME_HEAD1 && s_fsm == FSM_IDLE) {
        s_fsm = FSM_HEAD_AA;
        s_cmd_idx = 0;                      /* 清空命令缓冲 */
        s_cmd_buf[0] = '\0';
        return;
    }

    if (s_fsm != FSM_IDLE) {
        switch (s_fsm) {
        case FSM_HEAD_AA:
            s_fsm = (byte == FRAME_HEAD2) ? FSM_DATA : FSM_IDLE;
            if (s_fsm == FSM_DATA) s_payload_idx = 0;
            break;

        case FSM_DATA:
            s_payload[s_payload_idx++] = byte;
            if (s_payload_idx >= FRAME_PAYLOAD) {
                s_fsm = FSM_TAIL_FF1;
            }
            break;

        case FSM_TAIL_FF1:
            s_fsm = (byte == FRAME_TAIL) ? FSM_TAIL_FF2 : FSM_IDLE;
            break;

        case FSM_TAIL_FF2:
            if (byte == FRAME_TAIL) {
                /* 完整帧接收完成 */
                uint32_t vpp_raw, freq_raw;
                Frame_ParsePayload(s_payload, &vpp_raw, &freq_raw);
                if (vpp_raw != s_last_vpp_raw || freq_raw != s_last_freq_raw) {
                    s_last_vpp_raw  = vpp_raw;
                    s_last_freq_raw = freq_raw;
                    s_data_ready = true;
                }
            }
            s_fsm = FSM_IDLE;
            break;

        default:
            s_fsm = FSM_IDLE;
            break;
        }
        return;
    }

    /* ---- ASCII 命令累积 ---- */
    if ((byte >= 'a' && byte <= 'z') || byte == '_') {
        if (s_cmd_idx < CMD_BUF_SIZE - 1) {
            s_cmd_buf[s_cmd_idx++] = (char)byte;
            s_cmd_buf[s_cmd_idx] = '\0';
            Cmd_TryMatch();
        }
    } else {
        /* 非命令字符：清空缓冲 */
        s_cmd_idx = 0;
        s_cmd_buf[0] = '\0';
    }
}