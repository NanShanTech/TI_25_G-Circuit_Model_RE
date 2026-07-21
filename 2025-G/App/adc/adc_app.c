#include "adc_app.h"


///*DMA 采样到一半时触发*/
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
//{
//	if(hadc->Instance==ADC1)
//	{
//		ADC_Half_flage = 1;
//	}
//}

///*DMA 整个缓冲区采样完成后*/
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//	if(hadc->Instance==ADC1)
//	{
//		ADC_Full_flag = 1;
//	}
//}

void ADC_proc(void)
{
	// if(adc1_flag==1)
	// {
	
	// }
}



//static inline float adc_to_v(uint16_t val)
//{
//    return val * ADC_VREF / (float)ADC_MAX_CODE;
//}

//volatile uint8_t adc_con_flag = 0;
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//	if(hadc->Instance==ADC1)
//	{
//		if(adc_con_flag==0)
//			adc_con_flag = 1;
//	}
//}	
///*
// * 基础峰峰值分析：扫描整个缓冲区，寻找最大值和最小值。
// *
// * 对于干净的周期波形，该方法测量结果准确。
// * 适用于正弦波、方波、三角波、锯齿波等波形。
// *
// * 缺点是容易受到毛刺或异常点影响。
// * 如果信号噪声较大，建议使用 adc_get_vpp_robust()
// * 或 adc_get_vpp_auto()。
// */
//void adc_get_vpp(adc_waveform_info_t *info, const uint16_t *data, uint32_t len)
//{
//    if (info == NULL || data == NULL || len == 0)
//        return;

//    uint16_t vmax = 0;
//    uint16_t vmin = 65535;
//    uint64_t sum = 0;

//    for (uint32_t i = 0; i < len; i++) {
//        uint16_t v = data[i];
//        sum += v;

//        if (v > vmax)
//            vmax = v;

//        if (v < vmin)
//            vmin = v;
//    }

//    info->max_adc = vmax;
//    info->min_adc = vmin;
//    info->vmax = adc_to_v(vmax);
//    info->vmin = adc_to_v(vmin);
//    info->vpp  = info->vmax - info->vmin;
//    info->vavg = ((float)sum / len) * ADC_VREF / (float)ADC_MAX_CODE;
////			UART1_Printf("max_adc:%d,min_adc:%d,vmax;%.2fmv,vmin:%.2fmv,vpp:%.2fmv\r\n",info->max_adc,info->min_adc,
////		(info->vmax)*1000.0f,(info->vmin)*1000.0f,(info->vpp)*1000.0f);
//}

///*
// * 基于百分位的抗毛刺峰峰值分析。
// *
// * 该函数通过建立 512 个区间的直方图，然后分别寻找低百分位点
// * 和高百分位点，从而丢弃两端的异常采样点。
// *
// * 最终峰峰值由高百分位电压减去低百分位电压得到。
// *
// * discard_pct：
// *   表示从最低端和最高端分别丢弃多少百分比的采样点。
// *
// *   例如：
// *     discard_pct = 2.0f
// *
// *   表示丢弃最低的 2% 采样点和最高的 2% 采样点。
// *
// *   该参数会被限制在 [0, 50) 范围内。
// *
// *   如果传入 0 或负数，则自动退回到基础峰峰值算法 adc_get_vpp()。
// *
// * 注意：
// *   百分位算法会低估某些波形的真实峰峰值。
// *   例如三角波、锯齿波、窄脉冲等幅度分布较均匀或占空比较小的波形。
// *
// *   如果这些信号本身比较干净，建议使用 adc_get_vpp()。
// */
//void adc_get_vpp_robust(adc_waveform_info_t *info, const uint16_t *data,
//                        uint32_t len, float discard_pct)
//{
//	
//    if (info == NULL || data == NULL || len == 0)
//        return;

//    if (discard_pct <= 0.0f) {
//        adc_get_vpp(info, data, len);
//        return;
//    }

//    if (discard_pct >= 50.0f)
//        discard_pct = 49.0f;

//    uint32_t hist[HIST_BINS];
//    uint32_t i;

//    for (i = 0; i < HIST_BINS; i++)
//        hist[i] = 0;

//    /*
//     * 单次遍历：
//     *   1. 建立 ADC 数据直方图
//     *   2. 同时累加采样值，用于计算平均电压
//     */
//    uint64_t sum = 0;

//    for (i = 0; i < len; i++) {
//        uint16_t v  = data[i];
//        uint32_t bin = v / BIN_WIDTH;

//        if (bin >= HIST_BINS)
//            bin = HIST_BINS - 1;

//        hist[bin]++;
//        sum += v;
//    }

//    /*
//     * 计算每一端需要丢弃的采样点数量。
//     *
//     * 例如：
//     *   len = 8192
//     *   discard_pct = 2.0f
//     *
//     * 则：
//     *   discard_n = 8192 * 2 / 100 = 163
//     *
//     * 表示从低端丢弃 163 个点，
//     * 从高端也丢弃 163 个点。
//     */
//    uint32_t discard_n = (uint32_t)(len * discard_pct / 100.0f);

//    if (discard_n == 0)
//        discard_n = 1;

//    /*
//     * 寻找低端百分位值。
//     *
//     * 从直方图最低区间开始累加，
//     * 跳过最低的 discard_n 个采样点。
//     */
//    uint32_t cum = 0;
//    uint16_t lo_val = 0;

//    for (i = 0; i < HIST_BINS; i++) {
//        uint32_t prev = cum;
//        cum += hist[i];

//        if (cum >= discard_n) {
//            uint32_t offset = discard_n - prev;
//            uint32_t n      = hist[i];

//            if (n == 0)
//                n = 1;

//            lo_val = (uint16_t)(i * BIN_WIDTH + offset * BIN_WIDTH / n);
//            break;
//        }
//    }

//    /*
//     * 寻找高端百分位值。
//     *
//     * 从直方图最高区间开始反向累加，
//     * 跳过最高的 discard_n 个采样点。
//     */
//    cum = 0;
//    uint16_t hi_val = 65535;

//    for (i = HIST_BINS; i-- > 0; ) {
//        uint32_t prev = cum;
//        cum += hist[i];

//        if (cum >= discard_n) {
//            uint32_t offset = discard_n - prev;
//            uint32_t n      = hist[i];

//            if (n == 0)
//                n = 1;

//            uint32_t v = (i + 1) * BIN_WIDTH - offset * BIN_WIDTH / n;

//            if (v > 65535)
//                v = 65535;

//            hi_val = (uint16_t)v;
//            break;
//        }
//    }

//    info->max_adc = hi_val;
//    info->min_adc = lo_val;
//    info->vmax = adc_to_v(hi_val);
//    info->vmin = adc_to_v(lo_val);
//    info->vpp  = info->vmax - info->vmin;
//    info->vavg = ((float)sum / len) * ADC_VREF / (float)ADC_MAX_CODE;
////			UART3_Printf("max_adc:%d,min_adc:%d,vmax;%.2fmv,vmin:%.2fmv,vpp:%.2fmv\r\n",info->max_adc,info->min_adc,
////		(info->vmax)*1000.0f,(info->vmin)*1000.0f,(info->vpp)*1000.0f);
//}

///*
// * 自动峰峰值分析。
// *
// * 该函数会同时计算：
// *   1. 基础峰峰值 basic
// *   2. 抗毛刺峰峰值 robust
// *
// * 然后比较两者差异。
// *
// * 如果基础峰峰值明显大于抗毛刺峰峰值，
// * 说明数据中可能存在毛刺或异常点，
// * 此时返回 robust 结果。
// *
// * 如果两者差异不大，
// * 说明信号较干净，
// * 此时返回 basic 结果。
// *
// * 当前判断条件：
// *
// *   basic.vpp > robust.vpp * 1.08f
// *
// * 表示基础峰峰值比抗毛刺峰峰值大 8% 以上时，
// * 认为存在毛刺。
// */
//void adc_get_vpp_auto(adc_waveform_info_t *info, const uint16_t *data,
//                      uint32_t len)
//{
//    if (info == NULL || data == NULL || len == 0)
//        return;

//    adc_waveform_info_t basic;
//    adc_waveform_info_t robust;

//    adc_get_vpp(&basic, data, len);
//    adc_get_vpp_robust(&robust, data, len, 3.0f);

//    if (basic.vpp > robust.vpp * 1.08f)
//        *info = robust;
//    else
//        *info = basic;
//		
////		UART1_Printf("max_adc:%d,min_adc:%d,vmax;%.2fmv,vmin:%.2fmv,vpp:%.2fmv\r\n",info->max_adc,info->min_adc,(info->vmax)*1000.0f,
////		(info->vmin)*1000.0f,(info->vpp)*1000.0f-(info->vpp)*1000.0f*0.005f);
//}

///*
// * ADC 主处理函数。
// *
// * 该函数通常由调度器、主循环或定时任务周期性调用。
// *
// * DMA 和 D-Cache 注意事项：
// *
// * 如果 ADC_data 是由 DMA 填充的，并且系统开启了 D-Cache，
// * 那么 CPU 在读取 ADC_data 之前，必须先使对应缓存失效。
// *
// * 否则 CPU 可能读取到旧数据，导致峰峰值计算错误。
// *
// * 缓存失效示例：
// *
// *   uint32_t sz = (ADC_LEN * sizeof(uint16_t) + 31u) & ~31u;
// *   SCB_InvalidateDCache_by_Addr((uint32_t *)ADC_data, sz);
// *
// * 注意：
// *   地址和长度最好按照 32 字节对齐。
// *
// * DMA 循环模式注意事项：
// *
// * 如果 DMA 工作在循环模式下，DMA 可能在 CPU 读取缓冲区时
// * 同时覆盖该缓冲区，导致读取到一半新数据、一半旧数据。
// *
// * 可以考虑以下方案：
// *
// *   1. 使用半传输完成中断和全传输完成中断进行乒乓处理
// *   2. 在分析前 memcpy() 一份快照数据
// *   3. 每次只处理 DMA 已经写完的半个缓冲区
// */
// 

//void adc_proc(void)
//{
//		adc_waveform_info_t info;

//		 if(adc_con_flag==1)
//		 {
//			
//			HAL_ADC_Stop_DMA(&hadc1);
//			SCB_InvalidateDCache_by_Addr((uint32_t *)&ADC_data, ADC_LEN*sizeof(uint16_t));
//			adc_get_vpp_auto(&info, ADC_data, ADC_LEN);

//			adc_con_flag = 0;
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_data, ADC_LEN);
//		 }
//}

