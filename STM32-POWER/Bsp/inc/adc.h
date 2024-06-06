#ifndef __ADC_H
#define __ADC_H	
#include "stdint.h"
#include "stm32f10x_adc.h"

typedef struct
{
	float ADC_Val_5V;
	float ADC_Val_3V3;
	float ADC_Val_19V1;
	float ADC_Val_19V2;
	float ADC_Val_12V;
	float ADC_Val_CUR;
	float ADC_Val_BAT;
}_ADC_VALUE_;

void Adc_Init(void);
uint16_t  Get_Adc(uint8_t ch); 
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times); 
void ADC_DMA_Configuration(void);
void filter(void);
u16 GetVolt(u16 advalue);
void Value(u16 *value);
void GetValue(void);
#endif 
