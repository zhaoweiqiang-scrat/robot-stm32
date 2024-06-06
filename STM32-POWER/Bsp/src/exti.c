
#include "main.h"

uint8_t Flag_Front_Left,Flag_Front_Right,Flag_Later_Left,Flag_Later_Right;//前后碰撞标志

//B端口外部上升沿中断设置
//input：GPIO_Pin_x 要设置的中断引脚
//外部中断0服务程序
void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

	//GPIOB.5 中断线以及中断初始化配置   上下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);

  EXTI_InitStructure.EXTI_Line=EXTI_Line5;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	//GPIOB.4 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);

  EXTI_InitStructure.EXTI_Line=EXTI_Line4;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	//GPIOB.7 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);

  EXTI_InitStructure.EXTI_Line=EXTI_Line7;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	//GPIOB.8 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);

  EXTI_InitStructure.EXTI_Line=EXTI_Line8;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

/*
  //GPIOA.0 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

  EXTI_InitStructure.EXTI_Line=EXTI_Line0;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
*/	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能硬件碰撞所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能硬件碰撞所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级2， 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
}

//外部中断服务程序 
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)//中断是否发生
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
		EXTI_ClearITPendingBit(EXTI_Line4); //清除LINE4上的中断标志位  
	}
}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7)!=RESET)//中断是否发生
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
		EXTI_ClearITPendingBit(EXTI_Line7); //清除LINE5上的中断标志位  
	}
	
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(COL_FR == 0)	 	 //右前沿
		{				 
			Flag_Front_Right = 1;
		}	
		else if(COL_FR == 1)
		{
			Flag_Front_Right = 0;
		}
		Collide_Info(2);
		EXTI_ClearITPendingBit(EXTI_Line8); //清除LINE6上的中断标志位  
	}
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(COL_AL == 0)	 	  //左后沿碰撞
		{				 
			Flag_Later_Left = 1;
		}	
		else if(COL_AL == 1)
		{
			Flag_Later_Left = 0;
		}
		Collide_Info(3);
		EXTI_ClearITPendingBit(EXTI_Line5); //清除LINE7上的中断标志位  
	}
	
}
/*
//外部中断服务程序 
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)//中断是否发生
	{
		delay_ms(10);//消抖
		if(WK_UP == 0)	 	 //WK_UP按键按下
		{				 
			POWER_EN;//使能系统电源
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
		EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE14上的中断标志位  
	}
}
*/
