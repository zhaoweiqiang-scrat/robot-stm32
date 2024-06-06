
#include "led.h"

void Led::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RIKI_LED_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin     = RIKI_LED_PIN1 | RIKI_LED_PIN2;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�Խṹ���GPIO_OType����ֵ������IO�ڵĽṹ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//�Խṹ���GPIO_Speed����ֵ�������ٶ���100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //�Խṹ���GPIO_PuPd����ֵ�������ڲ�����
	GPIO_Init(RIKI_LED_GPIO_PORT, &GPIO_InitStructure);
}

void Led::on_off(bool status)
{
	if(status == true){
		GPIO_SetBits(RIKI_LED_GPIO_PORT, RIKI_LED_PIN1);
		GPIO_ResetBits(RIKI_LED_GPIO_PORT, RIKI_LED_PIN2);
	}else{
		GPIO_SetBits(RIKI_LED_GPIO_PORT, RIKI_LED_PIN2);
		GPIO_ResetBits(RIKI_LED_GPIO_PORT, RIKI_LED_PIN1);
	}
}
