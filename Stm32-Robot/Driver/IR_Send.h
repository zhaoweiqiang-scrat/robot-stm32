#ifdef __cplusplus
extern "C" {
#endif
#ifndef _IR_Send_H_
#define _IR_Send_H_
#include "stm32f4xx.h" 
	
void TIM3_PWM_Init(u32 arr,u32 psc);
	
void IR_head(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void IR_stop(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void IR_Send_0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void IR_Send_1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Nec_remote_IR_Send(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,unsigned int CusCode,unsigned char data);
void IR_Send_ContrlNum(uint8_t cmd);
#endif
#ifdef __cplusplus
}
#endif