#include "main.h"
#include "stm32f10x_tim.h"

#define IR_MARGIN		((WORD)130UL)
#define time_9000_us	((WORD)9000UL)
#define time_4500_us	((WORD)4500UL)
#define time_2250_us	((WORD)2250UL)
#define time_1125_us	((WORD)1125UL)

// State of the remote key decoder
#define IDLE		0
#define LEADER_ON	1
#define LEADER_OFF	2
#define ADDRESS		3
#define DATA1		4
#define DATA2		5
#define IR_ADDR  0x7F80

#define RISING_EDGE   1
#define FALLING_EDGE  0

uint8_t  Receive_Flag_start,Receive_Flag_stop,Receive_Num1,Receive_Num2;

//外部中断0服务程序
void EXTIX_Init(void)
{
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

    //GPIOE.2 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);

  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//IR_RXD
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键WK_UP所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
}
//设定边沿触发-----上升沿or下降沿  1：上升沿，0：下降沿
void IR_EXTI_Edge_Trigger(uint8_t ring_failing)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	  
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);

  EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//IR_RXD
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  
	if(ring_failing)
	{
	 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	}
	else
	{
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	}
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
}
void deal_irq_end(void)
{
	static BYTE	state = 0;	// State holder
	static BYTE		count;							// bits counter
	unsigned long time_count, t0;
	static BYTE		data1;					// Temporary for holding the decoded data
	/*NEC遥控指令的数据格式为：同步码头、地址码、地址反码、控制码、控制反码
	  同步码由一个9ms的低电平+一个4.5ms的高电平组成，地址码、地址反码、控制
	  码、控制反码均是8位数据格式*/
	switch (state) 
	{
	  case IDLE:		//同步码下降沿，低电平持续9ms
							TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
	            time_count = TIM_GetCounter(TIM2);	
		          IR_EXTI_Edge_Trigger(RISING_EDGE);//修改边沿触发中断为上升沿触发
		          state = LEADER_ON;//下一个状态为同步码上升沿
		          break;
    case LEADER_ON:        //处理同步码
	            time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
		          t0 = time_count;
				      IR_EXTI_Edge_Trigger(FALLING_EDGE);//修改边沿触发中断为下降沿触发
              state =  ((t0>(time_9000_us-8*IR_MARGIN)) && (t0<(time_9000_us+8*IR_MARGIN))) ? LEADER_OFF:IDLE;//低电平持续时间是否为9ms  
		          break;
    case LEADER_OFF:    //处理同步码
	            time_count = TIM_GetCounter(TIM2);
		          TIM_SetCounter(TIM2, 0);	
		          TIM_Cmd(TIM2, ENABLE);	
		          t0 = time_count;
		          if ((t0 > time_4500_us - (4*IR_MARGIN)) && (t0 < time_4500_us + (4*IR_MARGIN)) )//高电平持续时间是否为4.5ms
							{
			          //state = ADDRESS;//准备开始接收地址码、地址反码、控制码、控制反码，下降沿触发中断
			          //address = 0;
			          //count = 0;
								state = DATA1;
			          count = 0;
			          data1 = 0;
		          } 
		          else 
							{
			          IR_EXTI_Edge_Trigger(FALLING_EDGE);
			          state = IDLE;//接收出错，重新开始，下降沿触发
		          }	
		          break;
    case DATA1:
	           time_count = TIM_GetCounter(TIM2);
		         TIM_SetCounter(TIM2, 0);	
		         TIM_Cmd(TIM2, ENABLE);	
		         t0 = time_count ;
		         count++;
		         if ( (t0 > time_1125_us - 3*IR_MARGIN) && (t0 < time_1125_us + 3*IR_MARGIN)) 
						 {
			         data1 <<= 1;	/* a zero bit */
		         } 
		         else 
						 { 
			         if ((t0 > time_2250_us - 3*IR_MARGIN) && (t0 < time_2250_us + 4*IR_MARGIN)) 
							 {
				         data1 = (data1 << 1) | 1;	/* a one bit */
			         } 
			         else 
						   {
			          IR_EXTI_Edge_Trigger(FALLING_EDGE);
				        state = IDLE;
				        break;
               }
			       }
		         if (count == 8) 
						 {      	
			         IR_EXTI_Edge_Trigger(FALLING_EDGE);
			         state = IDLE;
								if(data1 == 0x39)
								{
									Receive_Flag_start = 1;//机器人开始巡逻
									Receive_Flag_stop = 0;//停止充电
									Receive_Num1++;
									Receive_Num2 = 0;
									if(Receive_Num1 == 1)
									{
										CHARGE_DIS;
										AC220_DIS;
										CHARGE_STATE_DIS;
										electromagnetism_OFF;
									}	
								}
								else if(data1 == 0x3A)
								{
									Receive_Flag_start = 0;//停止巡逻
									Receive_Flag_stop = 1;//接收到stm32f429的红外发射信号,机器人开始充电
									Receive_Num2++;
									Receive_Num1 = 0;
									if(Receive_Num2 == 1)
									{
										electromagnetism_ON;
									}
								}
		         }
		         break;
  }	
}
