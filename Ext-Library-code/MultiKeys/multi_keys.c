/*
 * @Author       : LuHeQiu
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-09-05 19:07:01
 * @LastEditors  : LuHeQiu
 * @Description  : 
 * @FilePath     : /NBU-RobotDriveController/Ext-Library-code/MultiKeys/multi_keys.c
 * @HomePage     : https://www.luheqiu.com
 */

/* 头文件 --------------------------------------------------------------------*/
#include "multi_keys.h"

/* 用户接口配置 ------------------------------------------------------------------*/
#include "main.h"


/* 外设接口：读取按键IO的电平 */
#define KEY_PRESSED_IO_STATE GPIO_PIN_RESET /* 用于设置按键被有效按下时的IO状态 */
#define GET_IO_STATE_KEY_1() HAL_GPIO_ReadPin(FnKEY1_GPIO_Port, FnKEY1_Pin)
#define GET_IO_STATE_KEY_2() HAL_GPIO_ReadPin(FnKEY2_GPIO_Port, FnKEY2_Pin)

/* 类型定义 ------------------------------------------------------------------*/

typedef struct _Key {
    /* 公有成员 */

    /* 按键响应回调函数 */
    void (*Callback_Handler)(KeyEvent e);
    Key16BitsType (*getClickCount)(Key* this);

    /* 私有成员 */
    KeyEvent eventFlagStack;
    KeyEvent keyState;
    Key16BitsType downTick;
    Key16BitsType upTick;

    Key16BitsType multiClickCount;

} _Key;

/* 按键配置 ------------------------------------------------------------------*/

_Key _key1 = {NULL,getClickCount};
_Key _key2 = {NULL,getClickCount};

Key* key1 = (Key*)&_key1;
Key* key2 = (Key*)&_key2;

/* 核心函数 ------------------------------------------------------------------*/
#define GET_IO_STATE_KEY(key_id) GET_IO_STATE_KEY_##key_id##()




/**
 * @brief  按键扫描任务，需要被周期性执行。
 */
// static const Key8BitsType key_num = (sizeof(_key)/sizeof(_key[0]));
void Key_Scan(void) {
    /* 扫描按键1 */
    Key_Update(key1, (GET_IO_STATE_KEY_1() == KEY_PRESSED_IO_STATE));
    /* 扫描按键2 */
    Key_Update(key2, (GET_IO_STATE_KEY_2() == KEY_PRESSED_IO_STATE));
}


void Key_Exec(void){
    /* 检查按键1并执行 */
    Key_Check(key1);
    /* 检查按键2并执行 */
    Key_Check(key2);
}

void Key_Check(Key* key){
    _Key* _key = (_Key*)key;
    
    if(_key->Callback_Handler != NULL && _key->eventFlagStack != KEY_EVENT_NONE){
        (*_key->Callback_Handler)(_key->eventFlagStack);
        _key->eventFlagStack = KEY_EVENT_NONE;
    }
}

void Key_Update(Key* key, Key8BitsType isPressed) {
    _Key* _key = (_Key*)key;

    switch (_key->keyState) {
        case KEY_EVENT_NONE: {
            if (isPressed) {
                _key->downTick += 1;
                _key->upTick = 0;
                if (_key->downTick >= KEY_VALID_MIN_TIME) {
                    _key->multiClickCount = 0;
                    _key->keyState = KEY_EVENT_PRESSED;
                    _key->eventFlagStack |= KEY_EVENT_PRESSED;
                }
            } else {
                _key->downTick = 0;
                _key->upTick = 0;
            }
            break;
        }

        case KEY_EVENT_PRESSED: {
            if (isPressed) {
                _key->downTick += 1;
                _key->upTick = 0;
                if (_key->downTick >= KEY_LONG_PRESSED_TIME) {
                    _key->downTick = 0;
                    _key->keyState = KEY_EVENT_LONG_PRESSED;
                    _key->eventFlagStack |= KEY_EVENT_LONG_PRESSED;
                }
            } else {
                _key->downTick = 0;
                _key->upTick += 1;
                if (_key->upTick >= KEY_VALID_MIN_TIME) {
                    _key->keyState = KEY_EVENT_RELEASED;
                    _key->eventFlagStack |= KEY_EVENT_RELEASED;
                }
            }
            break;
        }

        case KEY_EVENT_RELEASED: {
            if (isPressed) {
                _key->downTick += 1;
                _key->upTick = 0;

                if (_key->downTick >= KEY_VALID_MIN_TIME) {
                    _key->multiClickCount += 1;
                    _key->keyState = KEY_EVENT_MULTI_CLICKED;
                    _key->eventFlagStack |= KEY_EVENT_MULTI_CLICKED;
                    _key->eventFlagStack |= KEY_EVENT_PRESSED;
                }
            } else {
                _key->downTick = 0;
                _key->upTick += 1;

                if (_key->upTick >= KEY_MULTICLICK_INTERVAL_TIME) {
                    _key->keyState = KEY_EVENT_NONE;
                }
            }
            break;
        }

        case KEY_EVENT_MULTI_CLICKED: {
            if (isPressed) {
                _key->downTick += 1;
                _key->upTick = 0;
                if (_key->downTick >= KEY_LONG_PRESSED_TIME) {
                    _key->keyState = KEY_EVENT_LONG_PRESSED;
                    _key->eventFlagStack |= KEY_EVENT_LONG_PRESSED;
                }
            } else {
                _key->downTick = 0;
                _key->upTick += 1;
                if (_key->upTick >= KEY_VALID_MIN_TIME) {
                    _key->keyState = KEY_EVENT_RELEASED;
                    // _key->eventFlagStack |= KEY_EVENT_RELEASED;
                }
            }
            break;
        }

        case KEY_EVENT_LONG_PRESSED: {
            if (isPressed) {
                _key->downTick += 1;
                _key->upTick = 0;
                if (_key->downTick >= KEY_LONG_PRESSED_REPEAT_TIME) {
                    _key->downTick -= KEY_LONG_PRESSED_REPEAT_TIME;
                    _key->eventFlagStack |= KEY_EVENT_LONG_PRESSED_REPEAT;
                }
            } else {
                _key->downTick = 0;
                _key->upTick += 1;
                if (_key->upTick >= KEY_VALID_MIN_TIME) {
                    _key->keyState = KEY_EVENT_NONE;
                    _key->eventFlagStack |= KEY_EVENT_LONG_PRESSED_OVER;
                }
            }
            break;
        }

        default:
            break;
    }
}

Key16BitsType getClickCount(Key* key){
    return ((_Key*)key)->multiClickCount + 1;
}

