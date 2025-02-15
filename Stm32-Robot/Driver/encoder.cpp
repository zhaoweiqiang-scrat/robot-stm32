#include "encoder.h"

GPIO_TypeDef*  ENCODER_PORT[ENCODERn] = {RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER2_GPIO_PORT};
TIM_TypeDef*   ENCODER_TIM[ENCODERn] = {RIKI_ENCODER1_TIM, RIKI_ENCODER2_TIM};
const uint16_t ENCODER_A_PIN[ENCODERn] = {RIKI_ENCODER1_A_PIN, RIKI_ENCODER2_A_PIN};
const uint16_t ENCODER_B_PIN[ENCODERn] = {RIKI_ENCODER1_B_PIN, RIKI_ENCODER2_B_PIN};
const uint32_t ENCODER_PORT_CLK[ENCODERn] = {RIKI_ENCODER1_GPIO_CLK, RIKI_ENCODER2_GPIO_CLK};
const uint32_t ENCODER_TIM_CLK[ENCODERn] = {RIKI_ENCODER1_TIM_CLK, RIKI_ENCODER2_TIM_CLK};
const uint32_t ENCODER_TIM_GPIO_A_SOURCE[MOTORn] = {RIKI_ENCODER1_GPIO_A_SOURCE,RIKI_ENCODER2_GPIO_A_SOURCE};
const uint32_t ENCODER_TIM_GPIO_B_SOURCE[MOTORn] = {RIKI_ENCODER1_GPIO_B_SOURCE,RIKI_ENCODER2_GPIO_B_SOURCE};
const uint32_t ENCODER_TIM_A_AF[MOTORn] = {RIKI_ENCODER1_A_TIM_AF, RIKI_ENCODER2_A_TIM_AF};
const uint32_t ENCODER_TIM_B_AF[MOTORn] = {RIKI_ENCODER2_B_TIM_AF, RIKI_ENCODER2_B_TIM_AF};

Encoder::Encoder(Encoder_TypeDef _encoder, uint32_t _arr, uint32_t _psc, uint32_t _counts_per_rev)
{
	encoder = _encoder;
	arr = _arr;
	psc = _psc;
	counts_per_rev = _counts_per_rev;

	position = 0;
	last_timer = 0;
	last_timer_diff = 0;
}

void Encoder::init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_AHB1PeriphClockCmd(ENCODER_PORT_CLK[this->encoder], ENABLE);

	GPIO_PinAFConfig(ENCODER_PORT[this->encoder],ENCODER_TIM_GPIO_A_SOURCE[this->encoder],ENCODER_TIM_A_AF[this->encoder]); //gpio复用
    GPIO_PinAFConfig(ENCODER_PORT[this->encoder],ENCODER_TIM_GPIO_B_SOURCE[this->encoder],ENCODER_TIM_B_AF[this->encoder]);
	GPIO_InitStructure.GPIO_Pin = ENCODER_A_PIN[this->encoder] | ENCODER_B_PIN[this->encoder];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(ENCODER_PORT[this->encoder], &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(ENCODER_TIM_CLK[this->encoder], ENABLE);
	TIM_DeInit(ENCODER_TIM[this->encoder]);  
	TIM_TimeBaseInit(ENCODER_TIM[this->encoder], &TIM_TimeBaseStructure); 

	TIM_TimeBaseStructure.TIM_Period = this->arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = this->psc;     //设置预分频：  
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1;  //设置时钟分频系数：不分频 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式   

	TIM_TimeBaseInit(ENCODER_TIM[this->encoder], &TIM_TimeBaseStructure);

	if(this->encoder == ENCODER1){
		TIM_EncoderInterfaceConfig(ENCODER_TIM[this->encoder], TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);
	} else {
		TIM_EncoderInterfaceConfig(ENCODER_TIM[this->encoder], TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	}

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6; 
	TIM_ICInit(ENCODER_TIM[this->encoder], &TIM_ICInitStructure);
	
	TIM_ClearFlag(ENCODER_TIM[this->encoder], TIM_FLAG_Update); 
	//TIM_ITConfig(ENCODER_TIM[this->encoder], TIM_IT_Update, ENABLE); 

	TIM_SetCounter(ENCODER_TIM[this->encoder], 0);

	TIM_Cmd(ENCODER_TIM[this->encoder], ENABLE); 
}

int32_t Encoder::read()
{
	uint16_t timer_value = TIM_GetCounter(ENCODER_TIM[this->encoder]);
	
	last_timer_diff = timer_value - last_timer;
	last_timer = timer_value;
	position += (int32_t) last_timer_diff;

	return position;
}

int32_t Encoder::getRPM()
{
		long encoder_ticks = read();
		//this function calculates the motor's RPM based on encoder ticks and delta time
		unsigned long current_time = millis();
		unsigned long dt = current_time - prev_update_time_;

		//convert the time from milliseconds to minutes
		double dtm = (double)dt / 60000;
		double delta_ticks = encoder_ticks - prev_encoder_ticks_;

		//calculate wheel's speed (RPM)

		prev_update_time_ = current_time;
		prev_encoder_ticks_ = encoder_ticks;
		
		return (delta_ticks / counts_per_rev) / dtm;
}


void Encode_TIM7_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器7更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器11
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}




void Encoder::set_pos(int32_t pos)
{
	position = pos;
}
