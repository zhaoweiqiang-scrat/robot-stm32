#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "delay.h"
#include "usart.h"
#include "battery.h"
#include "exti.h"
#include "timer.h"
#include "adc.h"
#include "collide.h"
#include "string.h"

//������ʹ��IO
#define PMCU_CHARGE_ON_Pin GPIO_Pin_12
#define PMCU_CHARGE_ON_GPIO_Port GPIOB

//�����ѹ����
#define PWRSW_KEY_Pin GPIO_Pin_0
#define PWRSW_KEY_GPIO_Port GPIOA

//����ϵͳ��Դ����
#define POW_SYS_ON_Pin GPIO_Pin_15
#define POW_SYS_ON_GPIO_Port GPIOB

//����ϵͳָʾ��
#define POW_SYS_LED_Pin GPIO_Pin_14
#define POW_SYS_LED_GPIO_Port GPIOB

//����ϵͳ�����
#define POW_SYS_CHA_Pin GPIO_Pin_13
#define POW_SYS_CHA_GPIO_Port GPIOB

//���帴λF429��IO
#define MCU429RST_Pin GPIO_Pin_13
#define MCU429RST_GPIO_Port GPIOC

//����ϵͳ12Vʹ��
#define POW_SYS_12V_Pin GPIO_Pin_15
#define POW_SYS_12V_GPIO_Port GPIOA

//����ϵͳ19V1ʹ��
#define POW_SYS_19V1_Pin GPIO_Pin_8
#define POW_SYS_19V1_GPIO_Port GPIOA

//����ϵͳ19V2ʹ��
#define POW_SYS_19V2_Pin GPIO_Pin_2
#define POW_SYS_19V2_GPIO_Port GPIOB

//������485��Դʹ��
#define POW_BAT_485_Pin GPIO_Pin_3
#define POW_BAT_485_GPIO_Port GPIOB

//����ϵͳ��Դģ��MCU����ָʾ��
#define POW_SYS_RUN_Pin GPIO_Pin_9
#define POW_SYS_RUN_GPIO_Port GPIOB

//������̵���
#define CHARGE_EN  GPIO_SetBits(PMCU_CHARGE_ON_GPIO_Port, PMCU_CHARGE_ON_Pin)   
#define CHARGE_DIS GPIO_ResetBits(PMCU_CHARGE_ON_GPIO_Port, PMCU_CHARGE_ON_Pin)

//����ϵͳ��Դʹ��
#define POWER_EN  GPIO_SetBits(POW_SYS_ON_GPIO_Port, POW_SYS_ON_Pin)   
#define POWER_DIS GPIO_ResetBits(POW_SYS_ON_GPIO_Port, POW_SYS_ON_Pin)

//����ϵͳ��Դģ��MCU����ָʾ��
#define POWER_LED_ON  GPIO_SetBits(POW_SYS_RUN_GPIO_Port, POW_SYS_RUN_Pin)   
#define POWER_LED_OFF GPIO_ResetBits(POW_SYS_RUN_GPIO_Port, POW_SYS_RUN_Pin)

//����ϵͳ12Vʹ��
#define SYS_12V_EN  GPIO_ResetBits(POW_SYS_12V_GPIO_Port, POW_SYS_12V_Pin)   
#define SYS_12V_DIS GPIO_SetBits(POW_SYS_12V_GPIO_Port, POW_SYS_12V_Pin)
//����ϵͳ19V1ʹ��
#define SYS_19V1_EN  GPIO_ResetBits(POW_SYS_19V1_GPIO_Port, POW_SYS_19V1_Pin)   
#define SYS_19V1_DIS GPIO_SetBits(POW_SYS_19V1_GPIO_Port, POW_SYS_19V1_Pin)
//����ϵͳ19V2ʹ��
#define SYS_19V2_EN  GPIO_ResetBits(POW_SYS_19V2_GPIO_Port, POW_SYS_19V2_Pin)   
#define SYS_19V2_DIS GPIO_SetBits(POW_SYS_19V2_GPIO_Port, POW_SYS_19V2_Pin)
//����ϵͳ���485ʹ��
#define BAT_485_EN  GPIO_ResetBits(POW_BAT_485_GPIO_Port, POW_BAT_485_Pin)   
#define BAT_485_DIS GPIO_SetBits(POW_BAT_485_GPIO_Port, POW_BAT_485_Pin)
//����ϵͳ���485ʹ��
#define BAT_LED_ON  GPIO_SetBits(POW_SYS_LED_GPIO_Port, POW_SYS_LED_Pin)   
#define BAT_LED_OFF GPIO_ResetBits(POW_SYS_LED_GPIO_Port, POW_SYS_LED_Pin)

//���5V\3.3V\12V\19V\��ص�ѹ\ϵͳ����,��Ӧ��GPIO
#define V5V_DET_Pin GPIO_Pin_0
#define V5V_DET_GPIO_Port GPIOB

#define V3V3_DET_Pin GPIO_Pin_1
#define V3V3_DET_GPIO_Port GPIOA
#define V12_DET_Pin GPIO_Pin_4
#define V12_DET_GPIO_Port GPIOA
#define V19V_1_DET_Pin GPIO_Pin_5
#define V19V_1_DET_GPIO_Port GPIOA
#define V19V_2_DET_Pin GPIO_Pin_6
#define V19V_2_DET_GPIO_Port GPIOA
#define ADC_Batt_V_Pin GPIO_Pin_7
#define ADC_Batt_V_GPIO_Port GPIOA
#define CUR_DET_Pin GPIO_Pin_1
#define CUR_DET_GPIO_Port GPIOB

//��ǰ����ײIO
#define COLLIDE_DET1_Pin GPIO_Pin_7
#define COLLIDE_DET1_GPIO_Port GPIOB
//��ǰ����ײIO
#define COLLIDE_DET2_Pin GPIO_Pin_8
#define COLLIDE_DET2_GPIO_Port GPIOB

//�������ײIO
#define COLLIDE_DET3_Pin GPIO_Pin_5
#define COLLIDE_DET3_GPIO_Port GPIOB
//�Һ�����ײIO
#define COLLIDE_DET4_Pin GPIO_Pin_4
#define COLLIDE_DET4_GPIO_Port GPIOB

#define FENGSHAN_Pin GPIO_Pin_6
#define FENGSHAN_GPIO_Port GPIOB
#define OPEN_ON  GPIO_SetBits(FENGSHAN_GPIO_Port, FENGSHAN_Pin)   
#define OPEN_OFF GPIO_ResetBits(FENGSHAN_GPIO_Port, FENGSHAN_Pin)
#define CHA_INSERT   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)//��ȡ���

#endif
