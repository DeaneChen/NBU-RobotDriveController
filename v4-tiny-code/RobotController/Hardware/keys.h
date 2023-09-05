#ifndef _KEYS_H
#define _KEYS_H

#include "main.h"



uint8_t Key_Pressed(uint8_t nKey);  //当有按键按下时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
uint8_t Key_Released(uint8_t nKey); //当有按键按下并释放时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2



#endif

