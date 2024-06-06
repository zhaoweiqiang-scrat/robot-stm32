#ifdef __cplusplus
extern "C" {
#endif
#ifndef _KS103_H_
#define _KS103_H_
#include "stm32f4xx.h"
//Ä£Ê½¿ØÖÆ

#if 0
void KS103_Init(u32 bound);
int Sensors_Ultrasonic_task1(void);
int Sensors_Ultrasonic_task2(void);
int Sensors_Ultrasonic_task3(void);
int Sensors_Ultrasonic_task4(void);
int Sensors_Ultrasonic_task5(void);
int Sensors_Ultrasonic_task6(void);
int Sensors_Ultrasonic_task7(void);
	
void KS103_Send(u8 *buf);
void Change_Address(u8 OldAddress,u8 NewAddress);
extern u16 Ultr_distance_F_left,Ultr_distance_F_middle,Ultr_distance_F_right,Ultr_distance_left,Ultr_distance_right,Ultr_distance_A_left,Ultr_distance_A_right;	
extern u8 Flag_F_left,Flag_F_middle,Flag_F_right,Flag_left,Flag_right,Flag_A_left,Flag_A_right,Rec_Ok;
#endif

#endif 
#ifdef __cplusplus
}
#endif
