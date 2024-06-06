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
void Transmit_Cmd(void);//���Ͷ�ȡ�����Ϣ����
void Deal_Data(void);//��������Ϣ
void Send_T_429(void);//����������429
void Charing_Detection(void);
#endif
