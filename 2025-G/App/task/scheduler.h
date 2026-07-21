/**
 * @file    scheduler.h
 * @brief   时间触发任务调度器（基于 SysTick 1ms 级联）
 *
 *   参考模板：sample_app_task_scheduler
 *   时间链：1ms → 10ms → 100ms → 1秒 → 1分钟
 *
 *   用法：
 *     1. main() 中 Scheduler_Init() 初始化
 *     2. while(1) 中 Scheduler_Run() 循环调用
 *     3. 按需覆盖 Task_1ms / Task_10ms / Task_100ms / Task_1sec / Task_1min
 */

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>

/* ---- 时间片配置 ---- */
#define NUM_1MS_FOR_10MS    10U    /* 10 个 1ms = 1 个 10ms   */
#define NUM_10MS_FOR_100MS  10U    /* 10 个 10ms = 1 个 100ms */
#define NUM_100MS_FOR_1SEC  10U    /* 10 个 100ms = 1 秒      */
#define NUM_1SEC_FOR_1MIN   60U    /* 60 个 1s = 1 分钟        */

/* ---- 全局 Tick 计数器 ---- */
typedef struct {
    uint16_t ms1;      /* 1ms 累计  */
    uint16_t ms10;     /* 10ms 累计 */
    uint16_t ms100;    /* 100ms 累计*/
    uint16_t sec1;     /* 1秒 累计  */
    uint16_t min1;     /* 1分钟累计 */
} SchedTimer;

extern SchedTimer g_sched;

/* ---- 任务函数（__weak 空实现，按需覆盖）---- */
void Task_1ms  (uint16_t ticks);
void Task_10ms (uint16_t ticks);
void Task_100ms(void);
void Task_1sec (void);
void Task_1min (void);

/* ---- 调度器 API ---- */
void Scheduler_Init(void);
void Scheduler_Run (void);

#endif
