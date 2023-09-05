#include "keyled.h"
#include "delay.h" 


//KEY Part

//按键初始化函数
void Key_Init(void)
{
   /* 初始化相关IO */
   ;
} 

//----------------------------------------------------------
//按键处理函数
//参数表：mode:0 不支持连续按，1 支持连续按
//返回值：0 没有任何按键按下，1 KEY1按下，2 KEY2按下，4 WKUP按下 WK_UP
//响应优先级,KEY0>KEY1>WK_UP
//----------------------------------------------------------
uint8_t Key_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1; //按键按松开标志
	if(mode)key_up=1;        //支持连按		  
	if(key_up&&(KEY1_PIN==0||KEY2_PIN==0))
	{
		Delay_ms(10);//去抖动 
		key_up=0;
		if     (KEY1_PIN==0)  return KEY1;
		else if(KEY2_PIN==0)  return KEY2;
	}else if(KEY1_PIN==1&&KEY2_PIN==1)key_up=1; 	    
 	return 0;// 无按键按下
}


//LED Part


//初始化PA6和PA7为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)  
{
    /* 初始化相关IO */
    ;
}




















