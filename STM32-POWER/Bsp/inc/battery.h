#ifndef __BATTERY_H
#define __BATTERY_H

#include "stdint.h"

typedef struct
{
	float Voltage;
	float Current;
	float Surplus_capacity;
	float Total_capacity;
	float State_protection;
	float Surplus_capacity_percent;
}_Battery_Info_;

extern uint8_t Charge_Flag;
extern uint8_t CHA_FLAG,INSERT_FLAG;
extern float Surplus_capacity_percent,Current;
void Transmit_Cmd(void);//发送读取电池信息命令
void Deal_Data(void);//处理电池信息
void Send_T_429(void);//发送数据至429
void Charing_Detection(void);
#endif
