#include "Led_ws2812.h"

#define LED_NUM 2

#define LedIO_Reset HAL_GPIO_WritePin(FnLEDn_GPIO_Port, FnLEDn_Pin, GPIO_PIN_RESET)
#define LedIO_Set HAL_GPIO_WritePin(FnLEDn_GPIO_Port, FnLEDn_Pin, GPIO_PIN_SET)

uint8_t buffer[LED_NUM][3] = {0};


//发送保存在数组中的颜色信息，对外不用使用
void ws2812_show(void);

// 关闭id号led
void Led_clear(uint8_t id)
{
	uint8_t i;
	for (i = 0; i < 3; i++)
		buffer[id][i] = 0; // 占空比为0，全为低电平
	ws2812_show();
}
// 设置id号的led的RGB值
// RGB，红色、绿色、蓝色，每个值在0~255之间
void Led_set_RGB(uint8_t id, uint8_t R, uint8_t G, uint8_t B)
{
	buffer[id][0] = G;
	buffer[id][1] = R;
	buffer[id][2] = B;

	ws2812_show();
}

//=====================上面是调用即可的函数，下面是具体的控制实现=================

// 控制需要纳秒级的延迟，n为1时大概延时时间为370ns左右,3时为400ns
void delay_ns(volatile uint16_t ns)
{
	ns=ns;
	while (ns--);
}

// 发送WS2812定义的”0“
void Data0()
{
	LedIO_Set;
	delay_ns(1);
	LedIO_Reset;
	delay_ns(8);
}
// 发送WS2812定义的”1“
void Data1()
{
	LedIO_Set;
	delay_ns(7);
	LedIO_Reset;
	delay_ns(8);
}
// 发送WS2812 8位数据
void send_data(uint8_t dat)
{
	uint8_t i;
	for (i = 8; i > 0; i--)
	{
		if (dat & 0x80)
			Data1();
		else
			Data0();
		dat <<= 1;
	}
}

// Led的显示，把RGB数据发送过去，一个led发送24位数据，每次刷新需要全部发送一遍。
void ws2812_show(void)
{
	for (uint8_t led = 0; led < LED_NUM; led++)
		for (uint8_t i = 0; i < 3; i++)
			send_data(buffer[led][i]);

	HAL_Delay(1); // 故意延迟，防止连续修改RGB颜色时，没有延迟时间，导致芯片通信无结果
}
