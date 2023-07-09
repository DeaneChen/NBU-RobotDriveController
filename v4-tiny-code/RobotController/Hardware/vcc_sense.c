#include "vcc_sense.h"
#include "adc.h"




/**
 * @brief  获取电池电压
 * @return 电池电压，单位mV
 */
#if (IS_ENABLE_VBAT_MEAN_WINDOW_FILTER)
static uint16_t voltage_group[8] = {0};
static uint32_t voltage_acc = 0;
static uint16_t voltage_count = 0;
static uint16_t voltage_index = 0;
#endif
uint16_t Get_BattryVoltage(void) {
    
    uint8_t error = 0;

    error |= HAL_ADC_Start(&hadc3);
    error |= HAL_ADC_PollForConversion(&hadc3,2); /* ADC采样等待 超时2ms */
    uint32_t adc_value = HAL_ADC_GetValue(&hadc3);

    /* 将 /4096 拆分为两次运算，避免整形溢出 */
    adc_value = (((adc_value * ADC_REF_VOLTAGE) >> 4) * BAT_VOLTAGE_R_SAMPLE_RATIO) >> 8;

/* 启用均值窗口滤波 */
#if (IS_ENABLE_VBAT_MEAN_WINDOW_FILTER)

    /* 增加新值，减去旧值 */
    voltage_acc += adc_value - voltage_group[voltage_index];
    /* 存储新值 */
    voltage_group[voltage_index] = adc_value;
    /* 移动旧值索引 */
    voltage_index += 1;
    if (voltage_index >= 8) {
        voltage_index = 0;
    }
    /* 统计窗口填充数 */
    if (voltage_count < 8) {
        voltage_count += 1;
    }

    return (voltage_acc / voltage_count);

#else

    return (adc_value);

#endif

}

