#ifndef __TIMER_H
#define __TIMER_H

#include "stdint.h"

void TIM2_Int_Init(uint16_t arr,uint16_t psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
//void TIM3_PWM_Init(void);
#endif
