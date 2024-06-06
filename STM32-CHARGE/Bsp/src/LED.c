 #include "main.h"
 
void LED_Int(GPIO_TypeDef* GPIOx,uint16_t GPIO_PinX,uint32_t time)
{
	GPIO_InitTypeDef GPIO_X; 
	RCC_APB2PeriphClockCmd(time,ENABLE);
	GPIO_X.GPIO_Pin = GPIO_PinX;//LED
	GPIO_X.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_X.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_X);
	GPIO_SetBits(GPIOx,GPIO_PinX);
}

void DISCHARGE_LED_STATE(void)//����������׮δ�Ӵ�״̬
{
	WORK_EN;//�̵���
	CHARGE_STATE_EN;//�����
}
void RECHARGING_LED_STATE(void)//�����״ָ̬ʾ��
{
	WORK_EN;//�̵���
	CHARGE_STATE_EN;//�����
	delay_ms(10);
	CHARGE_STATE_DIS;//�����
	delay_ms(10);
}
void CHARGED_FULL_LED_STATE(void)//������״ָ̬ʾ��
{
	WORK_EN;//�̵���
	CHARGE_STATE_DIS;//�����
}
