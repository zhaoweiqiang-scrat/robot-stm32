#ifdef __cplusplus
extern "C" {
#endif
#include "IR_Send.h"
#include "delay.h"
	
#define IR_TXD_Pin GPIO_Pin_10
#define IR_TXD_GPIO_Port GPIOI
//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
////84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.  
void TIM3_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOI, ENABLE); 	//ʹ��PORTBʱ��	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); //GPIOB0����Ϊ��ʱ��3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           //GPIOB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //��ʼ��PB0
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           //GPIOB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOI,&GPIO_InitStructure);              //��ʼ��PB0
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
	
	//��ʼ��TIM3 Channel3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC3

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	//TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3									  
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
	Nec_remote_IR_Send(IR_TXD_GPIO_Port,IR_TXD_Pin,0x0000,cmd);//�ұߺ��ⷢ��0x39������,��ʾ����ѳ�����������Ҫ��ʼѲ��
	//delay_ms(2);
}

#ifdef __cplusplus
}
#endif