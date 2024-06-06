#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"	
void MCU2DY(u8 state);
void COL2MCU(void);
void BAT2MCU(void);	
void USART3_Init(u32 bound);
extern u8 Collision_Front_L,Collision_Front_R,Collision_Later_L,Collision_Later_R,SD_Rx_Flag;
extern float Voltage,Current;
extern float Surplus_capacity,Total_capacity;//电池剩余容量、总容量
extern float Surplus_capacity_percent;//剩余容量百分比
extern u16 State_protection;//保护状态
#ifdef __cplusplus
}
#endif