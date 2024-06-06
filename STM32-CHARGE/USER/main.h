#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "battery.h"
#include "timer.h"
#include "adc.h"
#include "string.h"
#include "math.h"
#include "ir_send.h"
#include "IR_Recevie.h"
#include "stm32f10x_tim.h"
#include "LED.h"
//������׮���״ָ̬ʾ��
#define PMCU_CHARGE_STATE_Pin GPIO_Pin_10
#define PMCU_CHARGE_STATE_GPIO_Port GPIOB
#define CHARGE_STATE_EN  GPIO_SetBits(PMCU_CHARGE_STATE_GPIO_Port, PMCU_CHARGE_STATE_Pin)   
#define CHARGE_STATE_DIS GPIO_ResetBits(PMCU_CHARGE_STATE_GPIO_Port, PMCU_CHARGE_STATE_Pin)

//������׮����״ָ̬ʾ��
#define PMCU_WORK_Pin GPIO_Pin_11
#define PMCU_WORK_GPIO_Port GPIOB
#define WORK_EN  GPIO_SetBits(PMCU_WORK_GPIO_Port, PMCU_WORK_Pin)   
#define WORK_DIS GPIO_ResetBits(PMCU_WORK_GPIO_Port, PMCU_WORK_Pin)

//������׮AC220V��翪��
#define PMCU_AC_ON_Pin GPIO_Pin_5
#define PMCU_AC_ON_GPIO_Port GPIOA
//������̵���
#define AC220_EN  GPIO_SetBits(PMCU_AC_ON_GPIO_Port, PMCU_AC_ON_Pin)   
#define AC220_DIS GPIO_ResetBits(PMCU_AC_ON_GPIO_Port, PMCU_AC_ON_Pin)
//������׮24V��翪��------������Ĭ��ΪOSC-IN����Ϊ�������ʹ��ʱ��ֻ�ܹ�����2MHZģʽ��
#define PMCU_CHARGE_ON_Pin GPIO_Pin_14
#define PMCU_CHARGE_ON_GPIO_Port GPIOC
//������̵���
#define CHARGE_EN  GPIO_SetBits(PMCU_CHARGE_ON_GPIO_Port, PMCU_CHARGE_ON_Pin)   
#define CHARGE_DIS GPIO_ResetBits(PMCU_CHARGE_ON_GPIO_Port, PMCU_CHARGE_ON_Pin)
//����ϵͳ����״ָ̬ʾ��
#define SYSTEM_RUN_Pin GPIO_Pin_12
#define SYSTEM_RUN_GPIO_Port GPIOB
//����ϵͳ����״ָ̬ʾ��״̬
#define LED_ON  GPIO_ResetBits(SYSTEM_RUN_GPIO_Port, SYSTEM_RUN_Pin)  
#define LED_OFF GPIO_SetBits(SYSTEM_RUN_GPIO_Port, SYSTEM_RUN_Pin) 
//��������ʹ��IO
#define electromagnetism_Pin GPIO_Pin_4
#define electromagnetism_GPIO_Port GPIOA
#define electromagnetism_ON  GPIO_ResetBits(electromagnetism_GPIO_Port, electromagnetism_Pin)  
#define electromagnetism_OFF GPIO_SetBits(electromagnetism_GPIO_Port, electromagnetism_Pin) 

//��������źŽ���IO
#define IR_RXD_Pin GPIO_Pin_13
#define IR_RXD_GPIO_Port GPIOC

#define IR_RXD_EXTI_IRQn EXTI15_10_IRQn

//��������źŷ���IO1
#define IR_TXD1_Pin GPIO_Pin_11
#define IR_TXD1_GPIO_Port GPIOA
//��������źŷ���IO2
#define IR_TXD2_Pin GPIO_Pin_7
#define IR_TXD2_GPIO_Port GPIOB
//��������źŷ���IO3
#define IR_TXD3_Pin GPIO_Pin_6
#define IR_TXD3_GPIO_Port GPIOB

#endif
