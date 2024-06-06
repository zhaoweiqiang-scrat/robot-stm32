#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"
 void LED_Int(GPIO_TypeDef* GPIOx,uint16_t GPIO_PinX,uint32_t time);
 void DISCHARGE_LED_STATE(void);
 void RECHARGING_LED_STATE(void);
 void CHARGED_FULL_LED_STATE(void);
#endif

