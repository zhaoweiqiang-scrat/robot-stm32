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

//定义充电使能IO
#define PMCU_CHARGE_ON_Pin GPIO_Pin_12
#define PMCU_CHARGE_ON_GPIO_Port GPIOB

//定义电压按键
#define PWRSW_KEY_Pin GPIO_Pin_0
#define PWRSW_KEY_GPIO_Port GPIOA

//定义系统电源开关
#define POW_SYS_ON_Pin GPIO_Pin_15
#define POW_SYS_ON_GPIO_Port GPIOB

//定义系统指示灯
#define POW_SYS_LED_Pin GPIO_Pin_14
#define POW_SYS_LED_GPIO_Port GPIOB

//定义系统充电检测
#define POW_SYS_CHA_Pin GPIO_Pin_13
#define POW_SYS_CHA_GPIO_Port GPIOB

//定义复位F429的IO
#define MCU429RST_Pin GPIO_Pin_13
#define MCU429RST_GPIO_Port GPIOC

//定义系统12V使能
#define POW_SYS_12V_Pin GPIO_Pin_15
#define POW_SYS_12V_GPIO_Port GPIOA

//定义系统19V1使能
#define POW_SYS_19V1_Pin GPIO_Pin_8
#define POW_SYS_19V1_GPIO_Port GPIOA

//定义系统19V2使能
#define POW_SYS_19V2_Pin GPIO_Pin_2
#define POW_SYS_19V2_GPIO_Port GPIOB

//定义电池485电源使能
#define POW_BAT_485_Pin GPIO_Pin_3
#define POW_BAT_485_GPIO_Port GPIOB

//定义系统电源模块MCU运行指示灯
#define POW_SYS_RUN_Pin GPIO_Pin_9
#define POW_SYS_RUN_GPIO_Port GPIOB

//定义充电继电器
#define CHARGE_EN  GPIO_SetBits(PMCU_CHARGE_ON_GPIO_Port, PMCU_CHARGE_ON_Pin)   
#define CHARGE_DIS GPIO_ResetBits(PMCU_CHARGE_ON_GPIO_Port, PMCU_CHARGE_ON_Pin)

//定义系统电源使能
#define POWER_EN  GPIO_SetBits(POW_SYS_ON_GPIO_Port, POW_SYS_ON_Pin)   
#define POWER_DIS GPIO_ResetBits(POW_SYS_ON_GPIO_Port, POW_SYS_ON_Pin)

//定义系统电源模块MCU运行指示灯
#define POWER_LED_ON  GPIO_SetBits(POW_SYS_RUN_GPIO_Port, POW_SYS_RUN_Pin)   
#define POWER_LED_OFF GPIO_ResetBits(POW_SYS_RUN_GPIO_Port, POW_SYS_RUN_Pin)

//定义系统12V使能
#define SYS_12V_EN  GPIO_ResetBits(POW_SYS_12V_GPIO_Port, POW_SYS_12V_Pin)   
#define SYS_12V_DIS GPIO_SetBits(POW_SYS_12V_GPIO_Port, POW_SYS_12V_Pin)
//定义系统19V1使能
#define SYS_19V1_EN  GPIO_ResetBits(POW_SYS_19V1_GPIO_Port, POW_SYS_19V1_Pin)   
#define SYS_19V1_DIS GPIO_SetBits(POW_SYS_19V1_GPIO_Port, POW_SYS_19V1_Pin)
//定义系统19V2使能
#define SYS_19V2_EN  GPIO_ResetBits(POW_SYS_19V2_GPIO_Port, POW_SYS_19V2_Pin)   
#define SYS_19V2_DIS GPIO_SetBits(POW_SYS_19V2_GPIO_Port, POW_SYS_19V2_Pin)
//定义系统电池485使能
#define BAT_485_EN  GPIO_ResetBits(POW_BAT_485_GPIO_Port, POW_BAT_485_Pin)   
#define BAT_485_DIS GPIO_SetBits(POW_BAT_485_GPIO_Port, POW_BAT_485_Pin)
//定义系统电池485使能
#define BAT_LED_ON  GPIO_SetBits(POW_SYS_LED_GPIO_Port, POW_SYS_LED_Pin)   
#define BAT_LED_OFF GPIO_ResetBits(POW_SYS_LED_GPIO_Port, POW_SYS_LED_Pin)

//监测5V\3.3V\12V\19V\电池电压\系统电流,对应的GPIO
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

//左前沿碰撞IO
#define COLLIDE_DET1_Pin GPIO_Pin_7
#define COLLIDE_DET1_GPIO_Port GPIOB
//右前沿碰撞IO
#define COLLIDE_DET2_Pin GPIO_Pin_8
#define COLLIDE_DET2_GPIO_Port GPIOB

//左后沿碰撞IO
#define COLLIDE_DET3_Pin GPIO_Pin_5
#define COLLIDE_DET3_GPIO_Port GPIOB
//右后沿碰撞IO
#define COLLIDE_DET4_Pin GPIO_Pin_4
#define COLLIDE_DET4_GPIO_Port GPIOB

#define FENGSHAN_Pin GPIO_Pin_6
#define FENGSHAN_GPIO_Port GPIOB
#define OPEN_ON  GPIO_SetBits(FENGSHAN_GPIO_Port, FENGSHAN_Pin)   
#define OPEN_OFF GPIO_ResetBits(FENGSHAN_GPIO_Port, FENGSHAN_Pin)
#define CHA_INSERT   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)//读取充电

#endif
