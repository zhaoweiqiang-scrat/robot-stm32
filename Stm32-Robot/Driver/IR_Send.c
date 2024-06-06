#ifdef __cplusplus
extern "C" {
#endif
#include "IR_Send.h"
#include "delay.h"
	
#define IR_TXD_Pin GPIO_Pin_10
#define IR_TXD_GPIO_Port GPIOI
//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
////84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.  
void TIM3_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOI, ENABLE); 	//使能PORTB时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); //GPIOB0复用为定时器3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           //GPIOB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB0
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           //GPIOB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOI,&GPIO_InitStructure);              //初始化PB0
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM3 Channel3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	//TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3									  
} 
unsigned char IR_Trxing_flag=0;

void Nec_remote_IR_Send(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,unsigned int CusCode,unsigned char data)
{
 unsigned char num_send=16;
 unsigned char data2=0;
 IR_Trxing_flag=1;

 data2=data;
 IR_head(GPIOx,GPIO_Pin);
 //Send customer Code 16bits
	#if 0
  while(num_send)
  {
	 if(CusCode&0x8000)
	 {
		IR_Send_1(GPIOx,GPIO_Pin);
	 }
	 else 
	 {
		IR_Send_0(GPIOx,GPIO_Pin);
	 }
	 CusCode=CusCode<<1;
	 num_send--;
  }
  #endif
//	Send data 8bits
  num_send=8;
  while(num_send)
  {
	 if(data&0x80)
	 {
		IR_Send_1(GPIOx,GPIO_Pin);
	 }
	 else 
	 {
		IR_Send_0(GPIOx,GPIO_Pin);
	 }
	 data=data<<1;
	 num_send--;
  }
//	Send ^data 8bits
	#if 0
  num_send=8;
  while(num_send)
  {
	 if(data2&0x80)
	 {
		IR_Send_0(GPIOx,GPIO_Pin);
	 }
	 else 
	 {
		IR_Send_1(GPIOx,GPIO_Pin);
	 }
	 data2=data2<<1;
	 num_send--;
  }
	#endif
  IR_stop(GPIOx,GPIO_Pin);
  IR_Trxing_flag=0;
}

void IR_head(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_SET);
	delay_us(9000); 
 	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_RESET);
	delay_us(4500); 

}
void IR_stop(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_SET);
	delay_us(560); 
 	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_RESET);
}
void IR_Send_0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_SET);
	delay_us(560); 
 	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_RESET);
	delay_us(560); 
	
}

void IR_Send_1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_SET);
	delay_us(560); 
 	GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_RESET);
	delay_us(1700); 
	
}

void IR_Send_ContrlNum(uint8_t cmd)
{
	Nec_remote_IR_Send(IR_TXD_GPIO_Port,IR_TXD_Pin,0x0000,cmd);//右边红外发射0x39控制码,表示电池已充满，机器人要开始巡检
	//delay_ms(2);
}

#ifdef __cplusplus
}
#endif