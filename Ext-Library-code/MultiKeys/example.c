/*
 * @Author       : LuHeQiu
 * @Date         : 2023-09-05 18:16:48
 * @LastEditTime : 2023-09-05 19:06:27
 * @LastEditors  : LuHeQiu
 * @Description  : 
 * @FilePath     : /NBU-RobotDriveController/Ext-Library-code/MultiKeys/example.c
 * @HomePage     : https://www.luheqiu.com
 */
/* 应用示例 */

#include "multi_keys.h"

void Key1_Callback(KeyEvent e){
    
    /* 按键按下 */
    if(Key_CheckEvent(e,KEY_EVENT_PRESSED)){
        printf("key1 Press!\n");
    }

    /* 按键释放 */
    if(Key_CheckEvent(e,KEY_EVENT_RELEASED)){
        printf("key1 Up!\n");
    }

    /* 按键长按 */
    if(Key_CheckEvent(e,KEY_EVENT_LONG_PRESSED)){
        printf("key1 Long Press!\n");
    }

    /* 按键仍然在长按 */
    if(Key_CheckEvent(e,KEY_EVENT_LONG_PRESSED_REPEAT)){
        printf("key1 Long Press Still!\r\n");
    }

    /* 长按的按键被释放 */
    if(Key_CheckEvent(e,KEY_EVENT_LONG_PRESSED_OVER)){
        printf("key1 Long Press Over!\n");
    }

    /* 按键连击 */
    if(Key_CheckEvent(e,KEY_EVENT_MULTI_CLICKED)){
        /* 双击 */
        if(key1->getClickCount(key1)==2){
            printf("key1 Double Click!\n");
            /* 多击 */
        }else{
            printf("key1 %d Click!\n",key1->getClickCount(key1));
        }
        
    }
}


void Key2_Callback(KeyEvent e){
    /* 当按键被按下 */
    if(Key_CheckEvent(e,KEY_EVENT_PRESSED)){
        /* 做一些事 */
    }
}


void main(void){

    /* 设置当按键响应时会触发调用的回调函数 */
    key1->Callback_Handler = &Key1_Callback;
    key2->Callback_Handler = &Key2_Callback;

    while(1){
        /* 在后台循环中扫描按键，在前台主循环执行按键，从而避免按键执行过程中的阻塞影响系统运行 */
        Key_Exec();
    }
}


/* 后台循环函数，或者类似的多任务系统中的任务 */
void Backend_Loop(void){
    /* 按键库的异步触发模式 */
    Key_Scan();
}