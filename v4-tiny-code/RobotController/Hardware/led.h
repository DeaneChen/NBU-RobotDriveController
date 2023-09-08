#ifndef _WS2812_H
#define _WS2812_H

#include "main.h"


/* LED声明 ----------------------------------------------------------------- */

/* 多彩LED的序号 */
#define FnLED2 0
#define FnLED3 1

/* 函数声明 ---------------------------------------------------------------- */

/* 点亮FnLED1 */
#define FnLED1_ON()    HAL_GPIO_WritePin(FnLED1_GPIO_Port, FnLED1_Pin, GPIO_PIN_RESET)
/* 关闭FnLED1 */
#define FnLED1_OFF()   HAL_GPIO_WritePin(FnLED1_GPIO_Port, FnLED1_Pin, GPIO_PIN_SET)
/* 切换FnLED1：亮灭反转 */
#define FnLED1_SHIFT() HAL_GPIO_TogglePin(FnLED1_GPIO_Port,FnLED1_Pin)

/**
 * @brief  关闭 序号为id 的多彩LED
 * @param  FnLED_ID 多彩LED的ID序号，可选值为 FnLED2（0）或 FnLED3（1）
 *                  出于数组管理的考虑，第一个多彩LED（FnLED2）的序号为0
 * @note   该函数带有阻塞性数据传输过程
 *         
 */
extern void FnLED_OFF(uint8_t FnLED_ID);


/**
 * @brief  设置 序号为id 的多彩led的RGB值
 * @param  FnLED_ID 多彩LED的ID序号，可选值为 FnLED2（0）或 FnLED3（1）
 *                  出于数组管理的考虑，第一个多彩LED（FnLED2）的序号为0
 * @param  R        红色亮度分量（0 ~ 255）
 * @param  G        绿色亮度分量（0 ~ 255）
 * @param  B        蓝色亮度分量（0 ~ 255）
 * @param  is_show  刷新控制 当is_show为1（True）时，则将设定的颜色直接显示在多彩LED上，
 *                           这会刷新显示所有多彩LED的颜色。
 *                           当is_show为0（False）时，则只变更颜色缓冲区，暂不刷新显示，
 *                           可用于连续同时修改多个多彩LED后一并刷新显示。
 */
extern void FnLED_SetRGB(uint8_t FnLED_ID, uint8_t R, uint8_t G, uint8_t B, uint8_t is_show);



/* 私有函数 ---------------------------------------------------------------- */
static void ws2812_show(void);

#endif
