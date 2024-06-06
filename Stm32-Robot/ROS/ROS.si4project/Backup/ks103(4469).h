#ifdef __cplusplus
extern "C" {
#endif
#ifndef _KS103_H_
#define _KS103_H_
#include "stm32f4xx.h"
//模式控制
#define RS485_RX_EN		PEout(9)	//485模式控制.0,使能接收
#define RS485_TX_EN		PEout(8)	//485模式控制.1,使能发送.
void KS103_Init(u32 bound);
void Sensors_Ultrasonic_task(void)

#endif 
#ifdef __cplusplus
}
#endif
