#ifdef __cplusplus
extern "C" {
#endif
#ifndef _KS103_H_
#define _KS103_H_
#include "stm32f4xx.h"
//ģʽ����
#define RS485_RX_EN		PEout(9)	//485ģʽ����.0,ʹ�ܽ���
#define RS485_TX_EN		PEout(8)	//485ģʽ����.1,ʹ�ܷ���.
void KS103_Init(u32 bound);
void Sensors_Ultrasonic_task(void)

#endif 
#ifdef __cplusplus
}
#endif
