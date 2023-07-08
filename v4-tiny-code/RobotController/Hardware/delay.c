#include "delay.h"


#define CPU_FREQUENCY_MHZ    168		// STM32时钟主频
//延迟指定微秒
void delay_us(__IO uint32_t us)  //1us时实际延迟会多0.5us
{
    int last, curr, val;
    int temp;

    while (us != 0)
    {
        temp = us > 900 ? 900 : us;// 确定实际延迟的时间，最大延迟为900微秒
        last = SysTick->VAL; // 获取当前的SysTick计数器的值,它是24位**递减**计数器，频率为内部时钟频率
        curr = last - CPU_FREQUENCY_MHZ * temp;// 计算出期望的SysTick计数器的值
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;//加上溢出周期，systick为溢出周期为1ms
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        us -= temp;
    }
}

