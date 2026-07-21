/**
 * @file    scheduler.c
 * @brief   时间触发任务调度器实现
 *
 *   基于 SysTick 1ms 时基，通过级联计数器实现多级周期调度。
 *   16 位 Tick 自动处理溢出（65536ms ≈ 65.5 秒循环）。
 *   余数保留机制避免累计误差。
 */

#include "scheduler.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ---- 全局变量 ---- */
SchedTimer g_sched;
static uint16_t s_last_tick;

/* ---- 获取当前 1ms Tick（16 位掩码）---- */
static uint16_t Tick_Get(void) {
    return (uint16_t)(HAL_GetTick() & 0xFFFF);
}

/* ---- 计算流逝 Tick 数（处理 16 位溢出）---- */
static uint16_t Tick_Elapsed(uint16_t last) {
    uint16_t now = Tick_Get();
    if (now >= last)
        return now - last;
    /* 计数器溢出：如 65530 → 5，实际流逝 = 5 + (65536-65530) = 11 */
    return now + (65536U - last);
}

/* ================================================================ */

void Scheduler_Init(void) {
    memset(&g_sched, 0, sizeof(g_sched));
    s_last_tick = Tick_Get();
}

void Scheduler_Run(void) {
    uint16_t elapsed = Tick_Elapsed(s_last_tick);
    s_last_tick += elapsed;
    s_last_tick &= 0xFFFF;

    if (elapsed == 0) return;

    /* ---- 1ms ---- */
    g_sched.ms1 += elapsed;
    Task_1ms(elapsed);

    if (g_sched.ms1 < NUM_1MS_FOR_10MS) return;

    /* ---- 10ms ---- */
    {
        uint16_t ticks = g_sched.ms1;
        g_sched.ms10 += ticks / NUM_1MS_FOR_10MS;
        g_sched.ms1  %= NUM_1MS_FOR_10MS;
        Task_10ms(ticks);
    }

    if (g_sched.ms10 < NUM_10MS_FOR_100MS) return;

    /* ---- 100ms ---- */
    g_sched.ms100 += g_sched.ms10 / NUM_10MS_FOR_100MS;
    g_sched.ms10  %= NUM_10MS_FOR_100MS;
    Task_100ms();

    if (g_sched.ms100 < NUM_100MS_FOR_1SEC) return;

    /* ---- 1 秒 ---- */
    {
        uint16_t sec_add = g_sched.ms100 / NUM_100MS_FOR_1SEC;
        g_sched.ms100   %= NUM_100MS_FOR_1SEC;
        g_sched.sec1    += sec_add;
        Task_1sec();
    }

    if (g_sched.sec1 < NUM_1SEC_FOR_1MIN) return;

    /* ---- 1 分钟 ---- */
    g_sched.min1 += g_sched.sec1 / NUM_1SEC_FOR_1MIN;
    g_sched.sec1 %= NUM_1SEC_FOR_1MIN;
    Task_1min();
}

/* ---- 默认空实现（用户按需覆盖）---- */
__weak void Task_1ms  (uint16_t ticks) { (void)ticks; }
__weak void Task_10ms (uint16_t ticks) { (void)ticks; }
__weak void Task_100ms(void)            { }
__weak void Task_1sec (void)            { }
__weak void Task_1min (void)            { }
