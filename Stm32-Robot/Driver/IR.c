#ifdef __cplusplus
extern "C" {
#endif
#include "IR.h"
#include "millisecondtimer.h"

u8 IR_LEFT,IR_RIGHT;
void IR_Init()
{
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
	
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//红外避障传感器接口GPIOG
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10; //infrared_obstacle1\2 -->PG9\PG10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource9);//PG9 连接到中断线9
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource10);//PG10 连接到中断线10
	
  /* ÅäÖÃEXTI_Line9,10 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line9|EXTI_Line10;//LINE9,LINE10
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //边沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能中断线
  EXTI_Init(&EXTI_InitStructure);//配置

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断9
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断
  NVIC_Init(&NVIC_InitStructure);//配置

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断10
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断
  NVIC_Init(&NVIC_InitStructure);//配置
	   
}
void EXTI9_5_IRQHandler(void)
{
  //
	int i = 255;
	while(i--);//消抖
	if(EXTI_GetITStatus(EXTI_Line9)!=RESET)//中断是否发生
	{
		if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9))
		{
			IR_LEFT = 1;//小于13cm
		}
		else
		{
			IR_LEFT = 0;//大于13cm
		}
    EXTI_ClearITPendingBit(EXTI_Line9);//清除LINE9上的中断标志
	}
}
#if 0
void EXTI15_10_IRQHandler(void)
{
  //
	int i = 255;
	while(i--);//消抖
	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)//中断是否发生
	{
		if(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10))
		{
			IR_RIGHT = 1;//小于13cm
		}
		else
		{
			IR_RIGHT = 0;//大于13cm
		}
    EXTI_ClearITPendingBit(EXTI_Line10);//清除LINE10上的中断标志
	}
}
#endif
#ifdef __cplusplus
}
#endif

