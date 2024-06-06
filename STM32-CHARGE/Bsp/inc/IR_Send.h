#ifndef __IR_SEND_H__
#define __IR_SEND_H__

#include "stdint.h"
#include "stm32f10x.h"

void IR_head(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void IR_stop(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void IR_Send_0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void IR_Send_1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Nec_remote_IR_Send(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,unsigned int CusCode,unsigned char data);
void IR_Send_ContrlNum(uint8_t state);

#endif


