#include "stm32f10x_tim.h"
#include "encoder.h"

int32_t position;
/* last read of timer */
uint16_t last_timer;
/* last difference between timer reads */
int16_t last_timer_diff;

void Encoder_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_DeInit(TIM3);  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 

	TIM_TimeBaseStructure.TIM_Period = 0xffff; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;      
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	

	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 6; 
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update); 
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 

	TIM_SetCounter(TIM3, 0);

	TIM_Cmd(TIM3, ENABLE); 
}

int32_t Encoder_read()
{
	uint16_t timer_value = TIM_GetCounter(TIM3);
	last_timer_diff = timer_value - last_timer;
	last_timer = timer_value;
	position += (int32_t) last_timer_diff;

	return position;
}

void Encoder_set_pos(int32_t pos)
{
	position = pos;
}
