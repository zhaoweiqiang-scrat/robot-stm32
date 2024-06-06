/*该工程的作用有两点：1、发射红外信号，2、控制充电开关，*/

#include "main.h"

#define THREHOLD 0.25f
#define CURRENT_CHARGED_FULL_ON_HOLD 1.5f //电池充满门限值（机器人开机状态下充满）---- 绿灯亮 红灯灭
#define CURRENT_CHARGED_FULL_OFF_HOLD 0.02f //电池充满门限值（机器人关机状态下充满）---- 绿灯亮 红灯灭
#define CURRENT_RECHARGING_HOLD 2.5f //充电中的电流门限值 ----绿灯亮 红灯闪
#define DISCHARGE_HOLD 0.01 //未充电 ----绿灯亮 红灯亮

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
  /*Configure GPIO pinA5 : AC220使能 */
  GPIO_InitStruct.GPIO_Pin = PMCU_AC_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_AC_ON_GPIO_Port, &GPIO_InitStruct); 
	
	/*Configure GPIO pinC14 : 24V充电使能 */
  GPIO_InitStruct.GPIO_Pin = PMCU_CHARGE_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_CHARGE_ON_GPIO_Port, &GPIO_InitStruct); 
	
	AC220_DIS; //默认关闭220V
	CHARGE_DIS;//默认关闭充电
	
	CHARGE_STATE_DIS;//默认关闭LED
	WORK_DIS;//默认关闭LED
	LED_OFF;//默认关闭LED
	
	/*Configure GPIO pinB10 : 充电状态 */
  GPIO_InitStruct.GPIO_Pin = PMCU_CHARGE_STATE_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_CHARGE_STATE_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB11 : 工作状态 */
  GPIO_InitStruct.GPIO_Pin = PMCU_WORK_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_WORK_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB12 : 运行状态*/
  GPIO_InitStruct.GPIO_Pin = SYSTEM_RUN_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SYSTEM_RUN_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinA4 : 电磁铁使能IO*/
  GPIO_InitStruct.GPIO_Pin = electromagnetism_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(electromagnetism_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinA11 : 红外发射灯1*/
  GPIO_InitStruct.GPIO_Pin = IR_TXD1_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IR_TXD1_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB7 : 红外发射灯2*/
  GPIO_InitStruct.GPIO_Pin = IR_TXD2_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IR_TXD2_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB6 : 红外发射灯3*/
  GPIO_InitStruct.GPIO_Pin = IR_TXD3_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IR_TXD3_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinC13 : 红外接收*/
  GPIO_InitStruct.GPIO_Pin = IR_RXD_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(IR_RXD_GPIO_Port, &GPIO_InitStruct);
}
RCC_ClocksTypeDef RCC_Clocks;
bool Charging_Flag;
void Charge_State_Led()
{
	if(ADC_VALUE.ADC_Val_CUR < DISCHARGE_HOLD)//未充电
	{
		Charging_Flag = 0;
		DISCHARGE_LED_STATE();
	}
	else if(ADC_VALUE.ADC_Val_CUR > CURRENT_RECHARGING_HOLD)//充电中
	{
		Charging_Flag = 1;
		RECHARGING_LED_STATE();
	}
		
	//else if(((ADC_VALUE.ADC_Val_CUR > CURRENT_CHARGED_FULL_OFF_HOLD) && (ADC_VALUE.ADC_Val_CUR < 2*CURRENT_CHARGED_FULL_OFF_HOLD)));//||((ADC_VALUE.ADC_Val_CUR > CURRENT_CHARGED_FULL_ON_HOLD) && (ADC_VALUE.ADC_Val_CUR < 2*CURRENT_CHARGED_FULL_ON_HOLD)))
		//CHARGED_FULL_LED_STATE();
}
int main(void)
{
	SystemInit();
	RCC_GetClocksFreq(&RCC_Clocks);
	delay_init(72);//72M 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MX_GPIO_Init();
	TIM2_Int_Init(10000,72-1);//10ms定时
	TIM3_PWM_Init(315,5);	 //6分频。PWM频率=12000000/316=37.97Khz
	//TIM_SetCompare2(TIM3,157);
	Adc_Init();
	//GetHall_Init();
	ADC_VALUE.ADC_Val_Init_Hall1 = 1.194;
	ADC_VALUE.ADC_Val_Init_Hall2 = 1.231;
	//WORK_EN;//打开工作状态指示灯
	LED_ON;
	while(1)
	{
		LED_ON;
		delay_ms(10);
		//delay_ms(10);
		GetValue();//获取充电器输出电压电流值
		GetHall();//读取霍尔传感器值
		Charge_State_Led();
		LED_OFF;
		AC220_EN;
		CHARGE_EN;
		if((fabs(ADC_VALUE.ADC_Val_Init_Hall1 - ADC_VALUE.ADC_Val_Hall1) > THREHOLD) || (fabs(ADC_VALUE.ADC_Val_Init_Hall2 - ADC_VALUE.ADC_Val_Hall2) > THREHOLD) || Charging_Flag)
		{
			IR_Send_ContrlNum(0);//发送抵达值	
			//CHARGE_STATE_EN;//打开充电状态指示灯			
		}
		
		else
		{
			IR_Send_ContrlNum(1);//发送中间值
//			CHARGE_DIS;
//			AC220_DIS;
//			//CHARGE_STATE_DIS;//关闭充电状态指示灯
		}
	}
}


