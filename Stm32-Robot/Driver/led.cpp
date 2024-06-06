
#include "led.h"

void Led::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RIKI_LED_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin     = RIKI_LED_PIN1 | RIKI_LED_PIN2;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//对结构体的GPIO_OType对象赋值，声明IO口的结构是推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//对结构体的GPIO_Speed对象赋值，声明速度是100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //对结构体的GPIO_PuPd对象赋值，声明内部上拉
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
