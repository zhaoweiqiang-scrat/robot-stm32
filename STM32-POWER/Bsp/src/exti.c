
#include "main.h"

uint8_t Flag_Front_Left,Flag_Front_Right,Flag_Later_Left,Flag_Later_Right;//ǰ����ײ��־

//B�˿��ⲿ�������ж�����
//input��GPIO_Pin_x Ҫ���õ��ж�����
//�ⲿ�ж�0�������
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

	//GPIOB.5 �ж����Լ��жϳ�ʼ������   ���½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);

  EXTI_InitStructure.EXTI_Line=EXTI_Line5;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	//GPIOB.4 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);

  EXTI_InitStructure.EXTI_Line=EXTI_Line4;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	//GPIOB.7 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);

  EXTI_InitStructure.EXTI_Line=EXTI_Line7;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	//GPIOB.8 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);

  EXTI_InitStructure.EXTI_Line=EXTI_Line8;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

/*
  //GPIOA.0 �ж����Լ��жϳ�ʼ������   �½��ش���
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

  EXTI_InitStructure.EXTI_Line=EXTI_Line0;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
*/	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ��Ӳ����ײ���ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//ʹ��Ӳ����ײ���ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//��ռ���ȼ�2�� 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
}

//�ⲿ�жϷ������ 
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)//�ж��Ƿ���
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
		EXTI_ClearITPendingBit(EXTI_Line4); //���LINE4�ϵ��жϱ�־λ  
	}
}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7)!=RESET)//�ж��Ƿ���
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
		EXTI_ClearITPendingBit(EXTI_Line7); //���LINE5�ϵ��жϱ�־λ  
	}
	
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(COL_FR == 0)	 	 //��ǰ��
		{				 
			Flag_Front_Right = 1;
		}	
		else if(COL_FR == 1)
		{
			Flag_Front_Right = 0;
		}
		Collide_Info(2);
		EXTI_ClearITPendingBit(EXTI_Line8); //���LINE6�ϵ��жϱ�־λ  
	}
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(COL_AL == 0)	 	  //�������ײ
		{				 
			Flag_Later_Left = 1;
		}	
		else if(COL_AL == 1)
		{
			Flag_Later_Left = 0;
		}
		Collide_Info(3);
		EXTI_ClearITPendingBit(EXTI_Line5); //���LINE7�ϵ��жϱ�־λ  
	}
	
}
/*
//�ⲿ�жϷ������ 
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)//�ж��Ƿ���
	{
		delay_ms(10);//����
		if(WK_UP == 0)	 	 //WK_UP��������
		{				 
			POWER_EN;//ʹ��ϵͳ��Դ
			SYS_12V_EN;
			SYS_19V1_EN;
			SYS_19V2_EN;
		}	
		else if(WK_UP == 1)
		{
			SYS_12V_DIS;
			SYS_19V1_DIS;
			SYS_19V2_DIS;
			POWER_DIS;
		}
		EXTI_ClearITPendingBit(EXTI_Line0); //���LINE14�ϵ��жϱ�־λ  
	}
}
*/
