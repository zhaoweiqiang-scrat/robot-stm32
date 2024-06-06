/*�ù��̵����������㣺1����������źţ�2�����Ƴ�翪�أ�*/

#include "main.h"

#define THREHOLD 0.25f
#define CURRENT_CHARGED_FULL_ON_HOLD 1.5f //��س�������ֵ�������˿���״̬�³�����---- �̵��� �����
#define CURRENT_CHARGED_FULL_OFF_HOLD 0.02f //��س�������ֵ�������˹ػ�״̬�³�����---- �̵��� �����
#define CURRENT_RECHARGING_HOLD 2.5f //����еĵ�������ֵ ----�̵��� �����
#define DISCHARGE_HOLD 0.01 //δ��� ----�̵��� �����

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
  /*Configure GPIO pinA5 : AC220ʹ�� */
  GPIO_InitStruct.GPIO_Pin = PMCU_AC_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_AC_ON_GPIO_Port, &GPIO_InitStruct); 
	
	/*Configure GPIO pinC14 : 24V���ʹ�� */
  GPIO_InitStruct.GPIO_Pin = PMCU_CHARGE_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_CHARGE_ON_GPIO_Port, &GPIO_InitStruct); 
	
	AC220_DIS; //Ĭ�Ϲر�220V
	CHARGE_DIS;//Ĭ�Ϲرճ��
	
	CHARGE_STATE_DIS;//Ĭ�Ϲر�LED
	WORK_DIS;//Ĭ�Ϲر�LED
	LED_OFF;//Ĭ�Ϲر�LED
	
	/*Configure GPIO pinB10 : ���״̬ */
  GPIO_InitStruct.GPIO_Pin = PMCU_CHARGE_STATE_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_CHARGE_STATE_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB11 : ����״̬ */
  GPIO_InitStruct.GPIO_Pin = PMCU_WORK_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_WORK_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB12 : ����״̬*/
  GPIO_InitStruct.GPIO_Pin = SYSTEM_RUN_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SYSTEM_RUN_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinA4 : �����ʹ��IO*/
  GPIO_InitStruct.GPIO_Pin = electromagnetism_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(electromagnetism_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinA11 : ���ⷢ���1*/
  GPIO_InitStruct.GPIO_Pin = IR_TXD1_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IR_TXD1_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB7 : ���ⷢ���2*/
  GPIO_InitStruct.GPIO_Pin = IR_TXD2_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IR_TXD2_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB6 : ���ⷢ���3*/
  GPIO_InitStruct.GPIO_Pin = IR_TXD3_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(IR_TXD3_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinC13 : �������*/
  GPIO_InitStruct.GPIO_Pin = IR_RXD_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(IR_RXD_GPIO_Port, &GPIO_InitStruct);
}
RCC_ClocksTypeDef RCC_Clocks;
bool Charging_Flag;
void Charge_State_Led()
{
	if(ADC_VALUE.ADC_Val_CUR < DISCHARGE_HOLD)//δ���
	{
		Charging_Flag = 0;
		DISCHARGE_LED_STATE();
	}
	else if(ADC_VALUE.ADC_Val_CUR > CURRENT_RECHARGING_HOLD)//�����
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
	TIM2_Int_Init(10000,72-1);//10ms��ʱ
	TIM3_PWM_Init(315,5);	 //6��Ƶ��PWMƵ��=12000000/316=37.97Khz
	//TIM_SetCompare2(TIM3,157);
	Adc_Init();
	//GetHall_Init();
	ADC_VALUE.ADC_Val_Init_Hall1 = 1.194;
	ADC_VALUE.ADC_Val_Init_Hall2 = 1.231;
	//WORK_EN;//�򿪹���״ָ̬ʾ��
	LED_ON;
	while(1)
	{
		LED_ON;
		delay_ms(10);
		//delay_ms(10);
		GetValue();//��ȡ����������ѹ����ֵ
		GetHall();//��ȡ����������ֵ
		Charge_State_Led();
		LED_OFF;
		AC220_EN;
		CHARGE_EN;
		if((fabs(ADC_VALUE.ADC_Val_Init_Hall1 - ADC_VALUE.ADC_Val_Hall1) > THREHOLD) || (fabs(ADC_VALUE.ADC_Val_Init_Hall2 - ADC_VALUE.ADC_Val_Hall2) > THREHOLD) || Charging_Flag)
		{
			IR_Send_ContrlNum(0);//���͵ִ�ֵ	
			//CHARGE_STATE_EN;//�򿪳��״ָ̬ʾ��			
		}
		
		else
		{
			IR_Send_ContrlNum(1);//�����м�ֵ
//			CHARGE_DIS;
//			AC220_DIS;
//			//CHARGE_STATE_DIS;//�رճ��״ָ̬ʾ��
		}
	}
}


