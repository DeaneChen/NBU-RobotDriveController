/* 头文件 ---------------------------------------------------------------------------- */
#include "keys.h"
#include "delay.h"


uint8_t Key_Pressed(uint8_t nKey)  //当有按键按下时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
{
	switch(nKey)
	{
		case 2:
			//配置PE3为Key2
			if(HAL_GPIO_ReadPin(FnKEY2_GPIO_Port,FnKEY2_Pin)==0)
			{
				delay_us(50);
				if(HAL_GPIO_ReadPin(FnKEY2_GPIO_Port,FnKEY2_Pin)==0)
					return 1;
			}
			break;
		case 1:
			//配置PE2为Key1
			if(HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY1_Pin)==0)
			{
				delay_us(50);
				if(HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY1_Pin)==0)
					return 1;
			}
			break;
		default:
			;
	}
	return 0;
}
uint8_t Key_Released(uint8_t nKey) //当有按键按下并释放时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
{
	switch(nKey)
	{
		case 2:
			//配置PE3为Key2
			if(HAL_GPIO_ReadPin(FnKEY2_GPIO_Port,FnKEY2_Pin)==0)
			{
				delay_us(50);
				if(HAL_GPIO_ReadPin(FnKEY2_GPIO_Port,FnKEY2_Pin)==0)
				{
					while(HAL_GPIO_ReadPin(FnKEY2_GPIO_Port,FnKEY2_Pin)==0); //如果一直按着则一直等着
					return 1;
				}
			}
			break;
		case 1:
			//配置PE2为Key1
			if(HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY1_Pin)==0)
			{
				delay_us(50);
				if(HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY1_Pin)==0)
				{
					while(HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY1_Pin)==0); //如果一直按着则一直等着
					return 1;
				}
			}
			break;
		default:
			;
	}
	return 0;	
}

