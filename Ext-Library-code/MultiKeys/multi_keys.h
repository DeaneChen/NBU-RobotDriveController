/*
 * @Author       : LuHeQiu
 * @Date         : 2023-04-19 23:08:45
 * @LastEditTime : 2023-09-05 19:04:16
 * @LastEditors  : LuHeQiu
 * @Description  : 按键库v1.0
 *                 这是一个基于对象的超轻量的事件驱动的多按键库，采用扫描与响应分离设计。
 *                 按键扫描函数为Key_Scan() 按键响应函数为Key_Exec() ，之间采用事件标志联系。
 *                 当Key_Scan和Key_Exec在同一个任务的上下文中时为同步按键响应。
 *                 当Key_Scan和Key_Exec在不同任务的上下文中时为异步按键响应。
 *                 后续开发进度：补充 条件编译的 宏定义，用于设置 允许长按、允许连击、最大连击次数、指定事件屏蔽 等功能。
 * @FilePath     : /NBU-RobotDriveController/Ext-Library-code/MultiKeys/multi_keys.h
 * @HomePage     : https://www.luheqiu.com
 */
#ifndef __MULTI_KEYS_H
#define __MULTI_KEYS_H

#include "main.h"

/* 系统配置---------------------------------------------------------------- */

/* 变量类型与所需的函数接口配置 */
typedef uint8_t  Key8BitsType;
typedef uint16_t  Key16BitsType;

/* 按键声明 */
typedef struct Key Key;

extern Key* key;

/* 按键有效操作的最短时间，也可以称为消抖系数，单位：tick（KEY_SCAN函数的调用周期） */
#define KEY_VALID_MIN_TIME (2)

/* 按键长按的判别时间，单位：tick */
#define KEY_LONG_PRESSED_TIME (150)

/* 按键长按的重复报告时间，单位：tick */
#define KEY_LONG_PRESSED_REPEAT_TIME (50)

/* 按键双击/多次点击的的最大间隔，单位：tick */
#define KEY_MULTICLICK_INTERVAL_TIME (40)

/* 类型定义---------------------------------------------------------------- */

typedef enum
{
    KEY_EVENT_NONE                 =  0U,   /* 空，按键无响应 */
    KEY_EVENT_PRESSED              =  1U,   /* 按下 */
    KEY_EVENT_RELEASED             =  2U,   /* 释放 */
    KEY_EVENT_MULTI_CLICKED        =  4U,   /* 连按 */
    KEY_EVENT_LONG_PRESSED         =  8U,   /* 长按 */
    KEY_EVENT_LONG_PRESSED_REPEAT  = 16U,   /* 长按重复 */
    KEY_EVENT_LONG_PRESSED_OVER    = 32U    /* 长按结束 */
}KeyEvent;


typedef struct Key
{
    
    /**
     * @brief      按键回调函数
     * @param[in]  e 响应的事件
     * @note       当按键被响应时，会自动调用Callback_Handler所指向的函数，并传入参数e，e包含了按键所响应的事件
     *             可以在回调函数中使用 Key_CheckEvent 函数检查事件，并执行对应的逻辑
     */
    void (*Callback_Handler)(KeyEvent e);

    /**
     * @brief  获取某一按键连击次数
     * @param  key 某一按键
     * @note   通常，只有当按键处于 KEY_EVENT_MULTI_CLICKED 时返回值才有意义，为大于等于2的连击次数。
     * @return 按键连击次数
     */
    Key16BitsType (*getClickCount)(Key* key);
    /* data */
}Key;

/* 按键声明---------------------------------------------------------------- */
extern Key *key1, *key2;



/* 函数声明---------------------------------------------------------------- */

/* 内部私有函数 */
static Key16BitsType getClickCount(Key* key);
static void Key_Update(Key* key, Key8BitsType isPressed);
static void Key_Check(Key* key);


/* 外部可调用函数 */

/**
 * @brief  按键事件检查
 * @param[in] e       响应的事件
 * @param[in] EVENT   按键事件
 * @retval 1 存在该事件
 * @retval 0 不存在该事件
 * @note   通常在按键回调函数中依照固定格式调用，例如：
 *         void key_callback(KeyEvent e){
 *             if(Key_CheckEvent(e,KEY_EVENT_PRESSED)){
 *                  function1();
 *             }
 *         }
 *         上述代码片断为：在按键回调函数中检查 响应事件e 是否为 按键事件KEY_EVENT_PRESSED (按键按下) ，
 *         如果是，则执行 function1
 */
#define Key_CheckEvent(e,EVENT) ((e)&(EVENT))


/**
 * @brief  按键扫描
 * @note   执行按键扫描，并设定按键的响应事件标志，但并不执行按键响应
 * @return 
 */
extern void Key_Scan(void);


/**
 * @brief  按键执行
 * @note   该函数将检查经过Key_Scan函数更新的按键事件标志，并根据响应的事件依次执行按键的回调函数，
 *         通常按键回调函数的传入参数类型为KeyEvent,可以通过检查KeyEvent来判断响应的事件类型
 * @return 
 */
extern void Key_Exec(void);






#endif
