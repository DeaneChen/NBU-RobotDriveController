
#include "amt1450_uart.h"
#include "delay.h"
#include "usart.h"

amt1450_UART_Rx_t amt1450_1_Rx;

/**
 * @brief: 串口接收配置使能or失能
 * @param {type}
 * @retval: None
 */
void AMT1450_UART_Cmd(FunctionalState NewState) {
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE) {
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart3_rx, 1);
    } else {
        HAL_UART_AbortReceive_IT(&huart3);
    }
}

/**
 * @brief: 获取amt1450数据（压缩数据）
 * @param {type}
 * @retval: begin_Color: 1/黑；0/白
 */
void get_AMT1450Data_UART(uint8_t* begin_Color, uint8_t* jump_Count, uint8_t* jump_Location) {
    *begin_Color = (amt1450_1_Rx.ValidData[1] & 0x80) >> 7;
    *jump_Count = amt1450_1_Rx.ValidData[1] & 0x0f;
    for (uint8_t i = 0; i < *jump_Count; i++) {
        jump_Location[i] = amt1450_1_Rx.ValidData[2 + i];
    }
}

uint8_t begin, jump, count[6];  // 最大6个跳变，即3条线
uint8_t position;
void amt1450_Test_UART(void) {
    while (1) {
        get_AMT1450Data_UART(&begin, &jump, count);
        if (jump == 2)
            position = 0.5f * (count[0] + count[1]);
        HAL_Delay(10);
    }
}

/**
 * @brief: SUM校验，传输校验
 * @param {type}
 * @retval: None
 */
uint8_t check_SUM(uint8_t* pData, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length - 1; i++) {
        sum += pData[i];
    }
    if ((uint8_t)sum == pData[length - 1])
        return 1;
    return 0;
}
/**
 * @brief: 一个字节中断接收
 * @param {type}
 * @retval: None
 */
void AMT1450_GetChar(uint8_t pData, amt1450_UART_Rx_t* amt1450_UART_Rx) {
    // 串口
    if (amt1450_UART_Rx->RxDataPtr >= 9) {
        amt1450_UART_Rx->RxDataPtr = 0;
        return;
    }
    if (amt1450_UART_Rx->RxDataPtr == 0 && pData == AMT1450_FRAME_START) {  // 帧头
        amt1450_UART_Rx->RxData[amt1450_UART_Rx->RxDataPtr++] = pData;
    } else if (amt1450_UART_Rx->RxDataPtr == 1) {
        amt1450_UART_Rx->RxData[amt1450_UART_Rx->RxDataPtr++] = pData;  // num
    } else if (amt1450_UART_Rx->RxDataPtr < ((amt1450_UART_Rx->RxData[1] & 0x0f) + 3)) {
        amt1450_UART_Rx->RxData[amt1450_UART_Rx->RxDataPtr++] = pData;
        //
        if (amt1450_UART_Rx->RxDataPtr == ((amt1450_UART_Rx->RxData[1] & 0x0f) + 3)) {  // 接收完成
            // 校验
            if (check_SUM(amt1450_UART_Rx->RxData, amt1450_UART_Rx->RxDataPtr)) {
                for (uint8_t i = 0; i < ((amt1450_UART_Rx->RxData[1] & 0x0f) + 3); i++)
                    amt1450_UART_Rx->ValidData[i] = amt1450_UART_Rx->RxData[i];  // 数据存储
                // printf ("%d\n", amt1450_UART_Rx->ValidData[1]&0x0f);
            }
            amt1450_UART_Rx->RxDataPtr = 0;
        }
    } else
        amt1450_UART_Rx->RxDataPtr = 0;
}
