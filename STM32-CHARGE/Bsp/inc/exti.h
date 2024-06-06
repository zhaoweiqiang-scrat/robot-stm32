#ifndef __EXTI_H
#define __EXTI_H
#include "stm32f10x.h"

#define COL_FL   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)//左前沿 
#define COL_FR   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)//右前沿
#define COL_AL   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)//左后沿 
#define COL_AR   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)//右后沿 

#define WK_UP   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)//读取按键(WK_UP) 

extern uint8_t Flag_Front_Left,Flag_Front_Right,Flag_Later_Left,Flag_Later_Right;//前后碰撞标志

void EXTIX_Init(void);
#endif
