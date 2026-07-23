#ifndef RLC_APP_H
#define RLC_APP_H

/* 初始化应用服务，并启动普通双 ADC 采样。 */
void rlc_app_init(void);

/* 应用前台任务，需要在 main 的 while(1) 中持续调用。 */
void rlc_app_task(void);

#endif
