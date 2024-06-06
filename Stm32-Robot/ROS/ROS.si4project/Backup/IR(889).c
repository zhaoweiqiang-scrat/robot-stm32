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

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ºìÍâ´«¸ÐÆ÷½Ó¿ÚGPIO
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10; //infrared_obstacle1\2 -->PG9\PG10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//ÆÕÍ¨ÊäÈëÄ£Ê½
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//ÏÂÀ­
  GPIO_Init(GPIOG, &GPIO_InitStructure);
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//Ê¹ÄÜSYSCFGÊ±ÖÓ
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource9);//PG9 Á¬½Óµ½ÖÐ¶ÌÏß9
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource10);//PG10Á¬½Óµ½ÖÐ¶ÌÏß10
	
  /* ÅäÖÃEXTI_Line9,10 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line9|EXTI_Line10;//LINE9,LINE10
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//ÖÐ¶ÏÊÂ¼þ
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //ÉÏÉýÑØ´¥·¢
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//Ê¹ÄÜÖÐ¶ÏÏß
  EXTI_Init(&EXTI_InitStructure);//ÅäÖÃ

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//Íâ²¿ÖÐ¶Ï9
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//ÇÀÕ¼ÓÅÏÈ¼¶0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//×ÓÓÅÏÈ¼¶2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//Ê¹ÄÜÍâ²¿ÖÐ¶Ï
  NVIC_Init(&NVIC_InitStructure);//ÅäÖÃ

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//Íâ²¿ÖÐ¶Ï10
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//ÇÀÕ¼ÓÅÏÈ¼¶0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//×ÓÓÅÏÈ¼¶2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//Ê¹ÄÜÍâ²¿ÖÐ¶Ï
  NVIC_Init(&NVIC_InitStructure);//ÅäÖÃ
	   
}
void EXTI9_5_IRQHandler(void)
{
  //
	delay(10);//Ïû¶¶
	if(EXTI_GetITStatus(EXTI_Line9)!=RESET)//ÖÐ¶ÏÊÇ·ñ·¢Éú
	{
		IR_LEFT = 1;
    EXTI_ClearITPendingBit(EXTI_Line9);//Çå³ýLINE9ÉÏµÄÖÐ¶Ï±êÖ¾Î» 
	}
}

void EXTI15_10_IRQHandler(void)
{
  //
  delay(10);//Ïû¶¶
	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)//ÖÐ¶ÏÊÇ·ñ·¢Éú
	{
		IR_RIGHT= 1;
    EXTI_ClearITPendingBit(EXTI_Line10);//Çå³ýLINE10ÉÏµÄÖÐ¶Ï±êÖ¾Î» 
	}
}
#ifdef __cplusplus
}
#endif

