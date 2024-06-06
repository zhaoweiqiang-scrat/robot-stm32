#ifdef __cplusplus
extern "C" {
#endif
#ifndef _KS103_H_
#define _KS103_H_
#include "stm32f4xx.h"
//Ä£Ê½¿ØÖÆ


void KS103_Init(u32 bound);
void Sensors_Ultrasonic_task(void);
void KS103_Send(void);

extern u16 Ultr_distance;	


#endif 
#ifdef __cplusplus
}
#endif
