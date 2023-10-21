/* 头文件 ---------------------------------------------------------------------------- */
#include "led.h"

/* 用户接口配置 ------------------------------------------------------------------ */

/* 多彩LED数量定义 */
#define LED_NUM 2

/* 外设接口定义：读取/控制IO电平 */
#define LedIO_Reset HAL_GPIO_WritePin(FnLEDn_GPIO_Port, FnLEDn_Pin, GPIO_PIN_RESET)
#define LedIO_Set HAL_GPIO_WritePin(FnLEDn_GPIO_Port, FnLEDn_Pin, GPIO_PIN_SET)

/* 变量定义 ---------------------------------------------------------------------- */

/* 多彩LED颜色数据缓冲区 */
static uint8_t buffer[LED_NUM][3] = {0};

/* 公有函数定义 ---------------------------------------------------------------------- */

/* 关闭 序号为id 的多彩LED */
void FnLED_OFF(uint8_t id) {
    /* 溢出控制 */
    if (id > LED_NUM)
        return;

    for (uint8_t i = 0; i < 3; i++)
        buffer[id][i] = 0;

    /* 刷新多彩LED的颜色显示 */
    ws2812_show();
}

/* 设置 序号为id 的多彩led的RGB值 */
/* RGB，红色、绿色、蓝色，每个值在0~255之间 */
void FnLED_SetRGB(uint8_t id, uint8_t R, uint8_t G, uint8_t B, uint8_t is_show) {
    /* 溢出控制 */
    if (id > LED_NUM)
        return;

    buffer[id][0] = G;
    buffer[id][1] = R;
    buffer[id][2] = B;

    if(is_show){
        ws2812_show();
    }
}

/* 私有函数定义 ---------------------------------------------------------------------- */

/**
 * @brief  控制需要纳秒级的延迟
 * @param  ns
 * @note   对于168MHz时钟，n为1时大概延时时间为370ns左右,3时为400ns
 */


void delay_ns(__IO uint16_t ns)
{
    while (ns--)
        ;
}

/**
 * ws2812 LED灯通信需要纳秒级延时通信
 * 由于纳秒级别的延迟，受到编译器版本，优化等级的影响，请根据自己的实际情况，选择下面的设置。
 * 编译器版本已自动判断，但优化等级需要自行选择。
 * 目前已对led.c文件单独设置了o3优化等级，全局设置中，不会影响此文件优化等级。
 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)
    // Arm Compiler 6
		#if 1 //请根据自己设置的优化等级选择情况
        // 优化级别为 O3
			#define DATA0H_DELAY_NS 5
			#define DATA0L_DELAY_NS 15
			#define DATA1H_DELAY_NS 15
			#define DATA1L_DELAY_NS 15
		#else  //优化级别为 O0
			#define DATA0H_DELAY_NS 1
			#define DATA0L_DELAY_NS 7
			#define DATA1H_DELAY_NS 6
			#define DATA1L_DELAY_NS 7
		#endif

#else // Arm Compiler 5
    #define DATA0H_DELAY_NS 1
		#define DATA0L_DELAY_NS 12
    #define DATA1H_DELAY_NS 9
		#define DATA1L_DELAY_NS 9
#endif

/**
 * @brief  发送WS2812定义的“0”
 */
void Data0() {
    LedIO_Set;
    delay_ns(DATA0H_DELAY_NS);
    LedIO_Reset;
    delay_ns(DATA0L_DELAY_NS);
}

/**
 * @brief  发送WS2812定义的“1”
 */
void Data1() {
    LedIO_Set;
    delay_ns(DATA1H_DELAY_NS);
    LedIO_Reset;
    delay_ns(DATA1L_DELAY_NS);
}


/**
 * @brief  基于单极性归零码发送 WS2812 8位数据
 */
void send_data(uint8_t dat) {
    for (uint8_t i = 8; i > 0; i--) {
        /* 高位优先，发送串行数据 */
        if (dat & 0x80)
            Data1();
        else
            Data0();
        dat <<= 1;
    }
}

/**
 * @brief  基于单极性归零码发送 WS2812 8位数据
 */
static int last_time = 0;
void ws2812_show(void) {

    /* 基于单极性归零码帧控制的故意延迟，防止连续通讯修改RGB颜色时，芯片通信无结果 */
    int current_time = HAL_GetTick();
    if (current_time - last_time <= 1) {
        HAL_Delay(1); 
    }
    last_time=current_time;
    for (uint8_t led = 0; led < LED_NUM; led++)
        for (uint8_t i = 0; i < 3; i++)
            send_data(buffer[led][i]);
}
