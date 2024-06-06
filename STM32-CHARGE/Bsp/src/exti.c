
#include "main.h"

uint8_t Flag_Front_Left,Flag_Front_Right,Flag_Later_Left,Flag_Later_Right;//前后碰撞标志

//B端口外部上升沿中断设置
//input：GPIO_Pin_x 要设置的中断引脚
//外部中断0服务程序
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

	//GPIOA.8 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);

  EXTI_InitStructure.EXTI_Line=EXTI_Line8;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	//GPIOB.6 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);

  EXTI_InitStructure.EXTI_Line=EXTI_Line6;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	//GPIOB.12 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);

  EXTI_InitStructure.EXTI_Line=EXTI_Line12;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	//GPIOB.13 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);

  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


  //GPIOB.14 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);

  EXTI_InitStructure.EXTI_Line=EXTI_Line14;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级2， 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
}

//外部中断服务程序 
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(COL_AR == 0)	 	 //右后沿碰撞
		{				 
			Flag_Later_Right = 1;
		}	
		else if(COL_AR == 1)
		{
			Flag_Later_Right = 0;
		}
		Collide_Info(4);
		EXTI_ClearITPendingBit(EXTI_Line6); //清除LINE6上的中断标志位  
	}
	
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(COL_AL == 0)	 	 //左后延碰撞
		{				 
			Flag_Later_Left = 1;
		}	
		else if(COL_AL == 1)
		{
			Flag_Later_Left = 0;
		}
		Collide_Info(3);
		EXTI_ClearITPendingBit(EXTI_Line8); //清除LINE8上的中断标志位  
	}
	
}

//外部中断服务程序 
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(COL_FL == 0)	 	 //左前沿
		{				 
			Flag_Front_Left = 1;
		}	
		else if(COL_FL == 1)
		{
			Flag_Front_Left = 0;
		}
		Collide_Info(1);
		EXTI_ClearITPendingBit(EXTI_Line12); //清除LINE12上的中断标志位  
	}
	
	if(EXTI_GetITStatus(EXTI_Line13)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(COL_FR == 0)	 	 //右后沿
		{				 
			Flag_Front_Right = 1;
		}	
		else if(COL_FR == 1)
		{
			Flag_Front_Right = 0;
		}
		Collide_Info(2);
		EXTI_ClearITPendingBit(EXTI_Line13); //清除LINE13上的中断标志位  
	}
	
	if(EXTI_GetITStatus(EXTI_Line14)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(WK_UP == 0)	 	 //WK_UP按键按下
		{				 
			POWER_EN;//使能系统电源
		}	
		else if(WK_UP == 1)
		{
			POWER_DIS;
		}
		EXTI_ClearITPendingBit(EXTI_Line14); //清除LINE14上的中断标志位  
	}
	
}
