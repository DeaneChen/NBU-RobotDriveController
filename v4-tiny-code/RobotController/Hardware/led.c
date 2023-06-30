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

    ws2812_show();
}

/* 私有函数定义 ---------------------------------------------------------------------- */

/**
 * @brief  控制需要纳秒级的延迟
 * @param  ns
 * @note   对于168MHz时钟，n为1时大概延时时间为370ns左右,3时为400ns
 */
void delay_ns(__IO uint16_t ns) {
    while (ns--)
        ;
}

/**
 * @brief  发送WS2812定义的“0”
 */
void Data0() {
    LedIO_Set;
    delay_ns(1);
    LedIO_Reset;
    delay_ns(12);
}

/**
 * @brief  发送WS2812定义的“0”
 */
void Data1() {
    LedIO_Set;
    delay_ns(9);
    LedIO_Reset;
    delay_ns(9);
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
    if (current_time - last_time < 0) {
        HAL_Delay(1); 
    }

    for (uint8_t led = 0; led < LED_NUM; led++)
        for (uint8_t i = 0; i < 3; i++)
            send_data(buffer[led][i]);
}
