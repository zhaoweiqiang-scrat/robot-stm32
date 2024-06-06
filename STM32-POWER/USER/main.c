/*�ù��̵��������ĵ㣺1������Դ������2�����5V\3.3V\12V\19V\��ص�ѹ\ϵͳ������3��ʵʱ�ϱ���ص�����Ϣ��429��4�����ؽ���*/

#include "main.h"
uint8_t SHUT_UP_FLAG,SHUT_DOWN_FLAG;
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
  /*Configure GPIO pinB2 : ���ʹ�� */
  GPIO_InitStruct.GPIO_Pin = PMCU_CHARGE_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_CHARGE_ON_GPIO_Port, &GPIO_InitStruct); 
	
	CHARGE_DIS;//Ĭ�Ϲرճ��
	
	/*Configure GPIO pinB7 : ��ǰ�� */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET1_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//���ó���������
  GPIO_Init(COLLIDE_DET1_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB8 : ��ǰ�� */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET2_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//���ó���������
  GPIO_Init(COLLIDE_DET2_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB5 : ����� */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET3_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//���ó���������
  GPIO_Init(COLLIDE_DET3_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB4 : �Һ��� */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET4_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//���ó���������
  GPIO_Init(COLLIDE_DET4_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinA0 : ��Դ���� */
  GPIO_InitStruct.GPIO_Pin = PWRSW_KEY_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//���ó���������
  GPIO_Init(PWRSW_KEY_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB15 : ϵͳ��Դ���� */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//���ó�����
  GPIO_Init(POW_SYS_ON_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB14 : ϵͳ��Դ���ָʾ�� */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_LED_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//���ó���������
  GPIO_Init(POW_SYS_LED_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB13 : ϵͳ�����*/
  GPIO_InitStruct.GPIO_Pin = POW_SYS_CHA_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;//���ó���������
  GPIO_Init(POW_SYS_CHA_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB9 : ϵͳ��ԴMCU����ָʾ�� */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_RUN_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//���ó���������
  GPIO_Init(POW_SYS_RUN_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinA 15 8 B2 3 : ϵͳ��Դ12V\19V1\19V2\���RS485��Դʹ�� */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_12V_Pin|POW_SYS_19V1_Pin|POW_SYS_19V2_Pin|POW_BAT_485_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//���ó��������
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pinB6 : ���ȿ��ƹܽ� */
  GPIO_InitStruct.GPIO_Pin = FENGSHAN_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//���ó���������
  GPIO_Init(FENGSHAN_GPIO_Port, &GPIO_InitStruct);
}

void LED_Toggle(void)
{
	POWER_LED_ON;
	if(CHA_FLAG &&(Current >0 ))
		BAT_LED_ON;
	delay_ms(250);
	if(CHA_FLAG && (Surplus_capacity_percent >= 1.0))
		BAT_LED_ON;
	else
		BAT_LED_OFF;
	POWER_LED_OFF;
	delay_ms(250);	
}
void Shut_Down()
{
	uint8_t Shut_Down_Cmd[4] = {0XEE,0X05,0X00,0X77};
	USART_Send_bytes(USART3,Shut_Down_Cmd,4);
}
void Query_Switch(void)
{
	  if(WK_UP == 0)	 	 //WK_UP��������
		{	
			delay_ms(200);//����
			if((WK_UP == 0)&&!SHUT_UP_FLAG)
			{
				SHUT_DOWN_FLAG = 0;
				SHUT_UP_FLAG = 1;
				POWER_EN;//ʹ��ϵͳ��Դ
				SYS_12V_EN;
				SYS_19V1_EN;
				SYS_19V2_EN;
				TIM_Cmd(TIM2, DISABLE);  //ʹ��TIMx	
			}
		}	
		else if(WK_UP == 1)
		{
			delay_ms(200);//����
			if((WK_UP == 1)&&!SHUT_DOWN_FLAG)
			{
				SHUT_UP_FLAG = 0;
				SHUT_DOWN_FLAG = 1;
				Shut_Down();
				SYS_12V_DIS;
		    SYS_19V1_DIS;
		    SYS_19V2_DIS;
		    POWER_DIS;
				//TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx	
			}
		}
}
int main(void)
{
	//uint8_t CMD_Info_Sta[7] = {0XDD,0XA5,0X03,0X00,0XFF,0XFD,0X77};//��03 ��ȡ������Ϣ��״̬
	delay_init(72);//72M 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MX_GPIO_Init();
	Usart1_Int(9600);//���ؽ�����������9600
	//Usart2_Int(115200);
	Usart3_Int(115200);
	EXTIX_Init();
	//TIM2_Int_Init(9999,35999);//2Khz�ļ���Ƶ�ʣ�������10000Ϊ5000ms  
	TIM3_Int_Init(1999,35999);//2Khz�ļ���Ƶ�ʣ�������2000Ϊ1000ms  
	Adc_Init();
	//OPEN_ON;
	while(1)
	{
		Query_Switch();//��ѯ��Դ�����Ƿ���
		Send_T_429();//��ȡ�������
		GetValue();//��ȡ���ӵ�ѹ��Ϣ
		LED_Toggle();
    Charing_Detection();		
	}
}
