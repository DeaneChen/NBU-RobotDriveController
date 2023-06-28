#ifndef _WS2812_H
#define _WS2812_H

	#include "main.h"

	// 清除第id号led
	void ws2812_clear(uint8_t id);
	// 设置id号的led的RGB值
	void Led_set_RGB(uint8_t id, uint8_t R, uint8_t G, uint8_t B);

#endif
