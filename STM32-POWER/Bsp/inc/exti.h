#ifndef __EXTI_H
#define __EXTI_H
#include "stm32f10x.h"

#define COL_FL   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)//��ǰ�� 
#define COL_FR   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)//��ǰ��
#define COL_AL   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)//����� 
#define COL_AR   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//�Һ��� 

#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����(WK_UP) 

extern uint8_t Flag_Front_Left,Flag_Front_Right,Flag_Later_Left,Flag_Later_Right;//ǰ����ײ��־

void EXTIX_Init(void);
#endif
