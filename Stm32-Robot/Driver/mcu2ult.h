#ifdef __cplusplus
extern "C" {
#endif
#ifndef _MCU2ULT_H_
#define _MCD2ULT_H_
#include "stm32f4xx.h"
//Ä£Ê½¿ØÖÆ


void UART7_Init(u32 bound);
int Sensors_Ultrasonic_task(void);
	
extern u16 Ultr_distance_F_left,Ultr_distance_F_right,Ultr_distance_A_left,Ultr_distance_A_right,Ultr_distance_left,Ultr_distance_right;	


#endif 
#ifdef __cplusplus
}
#endif
