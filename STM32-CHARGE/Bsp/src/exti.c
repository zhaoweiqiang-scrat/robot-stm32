
#include "main.h"

uint8_t Flag_Front_Left,Flag_Front_Right,Flag_Later_Left,Flag_Later_Right;//ǰ����ײ��־

//B�˿��ⲿ�������ж�����
//input��GPIO_Pin_x Ҫ���õ��ж�����
//�ⲿ�ж�0�������
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

	//GPIOA.8 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);

  EXTI_InitStructure.EXTI_Line=EXTI_Line8;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	//GPIOB.6 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);

  EXTI_InitStructure.EXTI_Line=EXTI_Line6;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	//GPIOB.12 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);

  EXTI_InitStructure.EXTI_Line=EXTI_Line12;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	//GPIOB.13 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);

  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


  //GPIOB.14 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);

  EXTI_InitStructure.EXTI_Line=EXTI_Line14;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//��ռ���ȼ�2�� 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
}

//�ⲿ�жϷ������ 
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(COL_AR == 0)	 	 //�Һ�����ײ
		{				 
			Flag_Later_Right = 1;
		}	
		else if(COL_AR == 1)
		{
			Flag_Later_Right = 0;
		}
		Collide_Info(4);
		EXTI_ClearITPendingBit(EXTI_Line6); //���LINE6�ϵ��жϱ�־λ  
	}
	
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(COL_AL == 0)	 	 //�������ײ
		{				 
			Flag_Later_Left = 1;
		}	
		else if(COL_AL == 1)
		{
			Flag_Later_Left = 0;
		}
		Collide_Info(3);
		EXTI_ClearITPendingBit(EXTI_Line8); //���LINE8�ϵ��жϱ�־λ  
	}
	
}

//�ⲿ�жϷ������ 
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(COL_FL == 0)	 	 //��ǰ��
		{				 
			Flag_Front_Left = 1;
		}	
		else if(COL_FL == 1)
		{
			Flag_Front_Left = 0;
		}
		Collide_Info(1);
		EXTI_ClearITPendingBit(EXTI_Line12); //���LINE12�ϵ��жϱ�־λ  
	}
	
	if(EXTI_GetITStatus(EXTI_Line13)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(COL_FR == 0)	 	 //�Һ���
		{				 
			Flag_Front_Right = 1;
		}	
		else if(COL_FR == 1)
		{
			Flag_Front_Right = 0;
		}
		Collide_Info(2);
		EXTI_ClearITPendingBit(EXTI_Line13); //���LINE13�ϵ��жϱ�־λ  
	}
	
	if(EXTI_GetITStatus(EXTI_Line14)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(WK_UP == 0)	 	 //WK_UP��������
		{				 
			POWER_EN;//ʹ��ϵͳ��Դ
		}	
		else if(WK_UP == 1)
		{
			POWER_DIS;
		}
		EXTI_ClearITPendingBit(EXTI_Line14); //���LINE14�ϵ��жϱ�־λ  
	}
	
}
