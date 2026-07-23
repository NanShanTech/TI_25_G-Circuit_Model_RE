# 应用层结构

本工程的应用入口参考 `2025G/Core/App` 的组织方式：`main.c` 只负责 CubeMX 生成的硬件初始化，业务统一从 `rlc_app` 进入。

```text
Core/Src/main.c
  -> rlc_app_init()
  -> while (1): rlc_app_task()

App/rlc_app.c                 顶层状态机、任务调度、HAL 回调分发
  -> comm/protocol.c          串口命令解析，修改 g_sys_mode
  -> sweep/sweep_learn.c      双 ADC 扫频、幅相计算、模型拟合、IIR 系数生成
  -> filter/filter_runtime.c  ADC2(PB0) -> IIR -> DAC 双缓冲运行时
  -> filter/iir_filter.c      单个数据块的 IIR 运算
  -> hmi/hmi_sweep.c          扫频曲线生成与发送
  -> comm/serial.c            UART DMA 与 HMI 基础通信
```

## 主循环

`rlc_app_task()` 每次循环先处理滤波 DMA 留下的半缓冲标志，再运行普通周期任务。实时滤波不等待 10 ms 调度器，因此不会额外增加一个任务周期的延迟。

```text
ADC2 DMA 半满/全满中断
  -> HAL_ADC_ConvHalfCpltCallback / HAL_ADC_ConvCpltCallback
  -> filter_runtime_on_adc_*()
  -> 只设置前半/后半待处理标志
  -> rlc_app_task()
  -> filter_runtime_process_pending()
  -> iir_filter_process_block()
  -> 清理 DAC 缓存，交给 DAC DMA 输出
```

## 模式切换

`protocol.c` 解析 HMI 命令并修改 `g_sys_mode`。`rlc_app.c` 的 10 ms 任务检测模式变化，按“退出旧模式 -> 进入新模式 -> 执行当前模式”的顺序处理。

```text
MODE_IDLE
  | start_learn
  v
MODE_LEARN -> 扫频 + 拟合 + 生成 IIR 系数 -> MODE_IDLE

MODE_IDLE
  | start_filter
  v
MODE_FILTER -> ADC2(PB0) + DAC 双缓冲实时滤波
```

滤波参数采用“默认值保底、成功学习后覆盖”的规则：上电未学习时使用内置低通系数；扫频拟合和 IIR 生成都成功后切换为学习系数。扫频失败不会清空当前参数，而是继续保留默认值或上一次成功学习值。UART1 的 `FILTER_START` 日志通过 `SOURCE=DEFAULT/LEARNED` 标明当前来源。

离开 `MODE_FILTER` 时，`filter_runtime_stop()` 会停止 TIM6、ADC DMA 和 DAC DMA，然后 `rlc_app` 恢复普通双 ADC 采样。

## RLC 物理约束

未知电路按题目限定为各一个 R、L、C 元件组成的二阶网络。扫频拟合使用同一个二阶分母，并把下列范围作为硬约束：

```text
R: 1kΩ  ~ 10kΩ
L: 1mH  ~ 10mH
C: 10nF ~ 100nF
```

因此候选特征频率必须满足：

```text
f0 = 1 / (2π√(LC)) 约为 5.03kHz ~ 50.33kHz
```

程序会在内部同时检查两种二阶阻尼关系，并根据当前 `f0` 计算对应的 Q 可行区间，排除无法由题目范围内 R、L、C 组成的候选点。串联或并联不作为输出结论，最终只输出四种滤波器类型和双线性变换后的 IIR 系数。

## HAL 回调

HAL 回调集中放在 `rlc_app.c`，只负责路由事件：

- 扫频进行中：ADC1/ADC2 完成事件交给 `sweep_learn`。
- 滤波模式：ADC2 半满/全满事件交给 `filter_runtime`。
- 普通模式：记录 ADC1/ADC2 完成标志。

这样 `main.c` 不再包含算法、状态机、DMA 缓冲或串口协议代码。
