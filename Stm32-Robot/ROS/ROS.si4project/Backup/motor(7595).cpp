#include "motor.h"
#include "usart.h"

GPIO_TypeDef* MOTOR_A_PORT[MOTORn] = {RIKI_MOTOR1_A_GPIO_PORT, RIKI_MOTOR2_A_GPIO_PORT};
GPIO_TypeDef* MOTOR_B_PORT[MOTORn] = {RIKI_MOTOR1_B_GPIO_PORT, RIKI_MOTOR2_B_GPIO_PORT};
const uint32_t  MOTOR_A_PORT_CLK[MOTORn] = {RIKI_MOTOR1_A_GPIO_CLK, RIKI_MOTOR2_A_GPIO_CLK};
const uint32_t  MOTOR_B_PORT_CLK[MOTORn] = {RIKI_MOTOR1_B_GPIO_CLK, RIKI_MOTOR2_B_GPIO_CLK};

GPIO_TypeDef* MOTOR_PWM_PORT[MOTORn] = {RIKI_MOTOR1_PWM_PORT, RIKI_MOTOR2_PWM_PORT};
TIM_TypeDef*  MOTOR_PWM_TIM[MOTORn] = {RIKI_MOTOR1_PWM_TIM, RIKI_MOTOR2_PWM_TIM};
const uint8_t   MOTOR_PWM_GPIO_SOURCE[MOTORn] = {RIKI_MOTOR1_PWM_GPIO_SOURCE,RIKI_MOTOR2_PWM_GPIO_SOURCE};
const uint16_t  MOTOR_PWM_PIN[MOTORn] = {RIKI_MOTOR1_PWM_PIN, RIKI_MOTOR2_PWM_PIN};
const uint32_t  MOTOR_PWM_GPIO_AF[MOTORn] = {RIKI_MOTOR1_PWM_GPIO_AF, RIKI_MOTOR2_PWM_GPIO_AF};
const uint32_t  MOTOR_PWM_PORT_CLK[MOTORn] = {RIKI_MOTOR1_PWM_CLK, RIKI_MOTOR2_PWM_CLK};
const uint32_t  MOTOR_PWM_TIM_CLK[MOTORn] = {RIKI_MOTOR1_PWM_TIM_CLK, RIKI_MOTOR2_PWM_TIM_CLK};
const uint16_t  MOTOR_A_PIN[MOTORn] = {RIKI_MOTOR1_A_PIN, RIKI_MOTOR2_A_PIN};
const uint16_t  MOTOR_B_PIN[MOTORn] = {RIKI_MOTOR1_B_PIN, RIKI_MOTOR2_B_PIN};



Motor::Motor(Motor_TypeDef _motor, uint32_t _arr, uint32_t _psc)
{
	motor = _motor;
	arr = _arr;
	psc = _psc;
}

void Motor::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(MOTOR_A_PORT_CLK[this->motor]|MOTOR_B_PORT_CLK[this->motor]|MOTOR_PWM_PORT_CLK[this->motor], ENABLE);
	//GPIOX复用

	GPIO_PinAFConfig(MOTOR_PWM_PORT[this->motor],MOTOR_PWM_GPIO_SOURCE[this->motor],MOTOR_PWM_GPIO_AF[this->motor]); 
	 /** init motor gpio **/
	GPIO_InitStructure.GPIO_Pin     = MOTOR_A_PIN[this->motor] ;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //AB初始化1 1，电机处于悬空状态      
	GPIO_Init(MOTOR_A_PORT[this->motor], &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin     = MOTOR_B_PIN[this->motor];
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //AB初始1 1，电机处于悬空状态   
	GPIO_Init(MOTOR_B_PORT[this->motor], &GPIO_InitStructure);

	/** init motor pwm gpio **/
	GPIO_InitStructure.GPIO_Pin     = MOTOR_PWM_PIN[this->motor];
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       //PWM初始为x，AB ->11,电机处于悬空状态
	GPIO_Init(MOTOR_PWM_PORT[this->motor], &GPIO_InitStructure);

	motor_pwm_init();
}

void Motor::motor_pwm_init()
{
	//pwm value ((1 + psc)/72M)*(1+arr)
	//eg: ((1+143)/72M)*(1+9999) = 0.02s --10000 count use 0.02s
	//set arduino pwm value 490hz 255 count 
	//((1 + 575)/72M)(1 + 254) = (1 / 490)
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	if(this->motor == MOTOR1 )
	{
	  RCC_APB2PeriphClockCmd(MOTOR_PWM_TIM_CLK[this->motor], ENABLE);
	}
	else
	{
	  RCC_APB1PeriphClockCmd(MOTOR_PWM_TIM_CLK[this->motor], ENABLE);
	}
	
	TIM_BaseInitStructure.TIM_Period                = this->arr;//预装载器
	TIM_BaseInitStructure.TIM_Prescaler             = this->psc;//预分频器
	TIM_BaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter     = 0;

	TIM_TimeBaseInit(MOTOR_PWM_TIM[this->motor], &TIM_BaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	if(this->motor == MOTOR1){
		TIM_OC1Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}

	if(this->motor == MOTOR2) {
		TIM_OC2Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}
	TIM_ARRPreloadConfig(MOTOR_PWM_TIM[this->motor], ENABLE);

	TIM_CtrlPWMOutputs(MOTOR_PWM_TIM[this->motor], ENABLE);
	TIM_Cmd(MOTOR_PWM_TIM[this->motor], ENABLE);
}

void Motor::spin(int pwm)
{
	if(pwm > 0){
		 GPIO_ResetBits(MOTOR_A_PORT[this->motor], MOTOR_A_PIN[this->motor]);
		 GPIO_SetBits(MOTOR_B_PORT[this->motor], MOTOR_B_PIN[this->motor]);
	 }
	 else if(pwm < 0) {
		 GPIO_ResetBits(MOTOR_B_PORT[this->motor], MOTOR_B_PIN[this->motor]);
		 GPIO_SetBits(MOTOR_A_PORT[this->motor], MOTOR_A_PIN[this->motor]);
	 }
//}
	//printf("pwm=:%d\r\n",pwm);
	if(this->motor == MOTOR1){
		TIM_SetCompare1(MOTOR_PWM_TIM[this->motor], abs(pwm));
	}
	if(this->motor == MOTOR2){
		TIM_SetCompare2(MOTOR_PWM_TIM[this->motor], abs(pwm));
	}
}
