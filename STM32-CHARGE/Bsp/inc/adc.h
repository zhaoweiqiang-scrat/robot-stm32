#ifndef __ADC_H
#define __ADC_H	
#include "stdint.h"
#include "stm32f10x_adc.h"

typedef struct
{
	float ADC_Val_Init_Hall1;
	float ADC_Val_Init_Hall2;
	float ADC_Val_Hall1;
	float ADC_Val_Hall2;
	float ADC_Val_CUR;
	float ADC_Val_BAT;
}_ADC_VALUE_;

extern _ADC_VALUE_ ADC_VALUE;

void GetHall_Init(void);
void GetHall(void);
void Adc_Init(void);
uint16_t  Get_Adc(uint8_t ch); 
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times); 
void ADC_DMA_Configuration(void);
void filter(void);
u16 GetVolt(u16 advalue);
void Value(u16 *value);
void GetValue(void);
#endif 
