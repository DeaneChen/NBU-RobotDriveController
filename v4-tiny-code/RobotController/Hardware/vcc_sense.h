#ifndef __VCC_SENSE_H
#define __VCC_SENSE_H

#include "main.h"

/**
 * @brief  获取电池电压
 * @note   该函数为阻塞查询函数，消耗的时间一般可以忽略不计（理想情况下ADC查询开销几us），但在最坏的情况下可能发生ADC超时（极少）。
 * @return 电池电压，单位mV
 */
uint16_t Get_BattryVoltage(void);

#endif
