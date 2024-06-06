/*该工程的作用有四点：1、检测电源按键，2、监测5V\3.3V\12V\19V\电池电压\系统电流，3、实时上报电池电量信息至429，4、与电池交互*/

#include "main.h"
uint8_t SHUT_UP_FLAG,SHUT_DOWN_FLAG;
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
  /*Configure GPIO pinB2 : 充电使能 */
  GPIO_InitStruct.GPIO_Pin = PMCU_CHARGE_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PMCU_CHARGE_ON_GPIO_Port, &GPIO_InitStruct); 
	
	CHARGE_DIS;//默认关闭充电
	
	/*Configure GPIO pinB7 : 左前沿 */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET1_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
  GPIO_Init(COLLIDE_DET1_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB8 : 右前沿 */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET2_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
  GPIO_Init(COLLIDE_DET2_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB5 : 左后沿 */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET3_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
  GPIO_Init(COLLIDE_DET3_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pinB4 : 右后沿 */
  GPIO_InitStruct.GPIO_Pin = COLLIDE_DET4_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
  GPIO_Init(COLLIDE_DET4_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinA0 : 电源按键 */
  GPIO_InitStruct.GPIO_Pin = PWRSW_KEY_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
  GPIO_Init(PWRSW_KEY_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB15 : 系统电源开关 */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_ON_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//设置成上拉
  GPIO_Init(POW_SYS_ON_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB14 : 系统电源充电指示灯 */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_LED_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//设置成上拉输入
  GPIO_Init(POW_SYS_LED_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB13 : 系统充电检测*/
  GPIO_InitStruct.GPIO_Pin = POW_SYS_CHA_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;//设置成下拉输入
  GPIO_Init(POW_SYS_CHA_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinB9 : 系统电源MCU运行指示灯 */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_RUN_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//设置成上拉输入
  GPIO_Init(POW_SYS_RUN_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pinA 15 8 B2 3 : 系统电源12V\19V1\19V2\电池RS485电源使能 */
  GPIO_InitStruct.GPIO_Pin = POW_SYS_12V_Pin|POW_SYS_19V1_Pin|POW_SYS_19V2_Pin|POW_BAT_485_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//设置成上拉输出
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pinB6 : 风扇控制管脚 */
  GPIO_InitStruct.GPIO_Pin = FENGSHAN_Pin;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//设置成上拉输入
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
	  if(WK_UP == 0)	 	 //WK_UP按键按下
		{	
			delay_ms(200);//消抖
			if((WK_UP == 0)&&!SHUT_UP_FLAG)
			{
				SHUT_DOWN_FLAG = 0;
				SHUT_UP_FLAG = 1;
				POWER_EN;//使能系统电源
				SYS_12V_EN;
				SYS_19V1_EN;
				SYS_19V2_EN;
				TIM_Cmd(TIM2, DISABLE);  //使能TIMx	
			}
		}	
		else if(WK_UP == 1)
		{
			delay_ms(200);//消抖
			if((WK_UP == 1)&&!SHUT_DOWN_FLAG)
			{
				SHUT_UP_FLAG = 0;
				SHUT_DOWN_FLAG = 1;
				Shut_Down();
				SYS_12V_DIS;
		    SYS_19V1_DIS;
		    SYS_19V2_DIS;
		    POWER_DIS;
				//TIM_Cmd(TIM2, ENABLE);  //使能TIMx	
			}
		}
}
int main(void)
{
	//uint8_t CMD_Info_Sta[7] = {0XDD,0XA5,0X03,0X00,0XFF,0XFD,0X77};//读03 读取基本信息及状态
	delay_init(72);//72M 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MX_GPIO_Init();
	Usart1_Int(9600);//与电池交互，波特率9600
	//Usart2_Int(115200);
	Usart3_Int(115200);
	EXTIX_Init();
	//TIM2_Int_Init(9999,35999);//2Khz的计数频率，计数到10000为5000ms  
	TIM3_Int_Init(1999,35999);//2Khz的计数频率，计数到2000为1000ms  
	Adc_Init();
	//OPEN_ON;
	while(1)
	{
		Query_Switch();//查询电源按键是否按下
		Send_T_429();//读取电池数据
		GetValue();//获取板子电压信息
		LED_Toggle();
    Charing_Detection();		
	}
}
