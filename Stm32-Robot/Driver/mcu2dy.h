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
extern float Surplus_capacity,Total_capacity;//���ʣ��������������
extern float Surplus_capacity_percent;//ʣ�������ٷֱ�
extern u16 State_protection;//����״̬
#ifdef __cplusplus
}
#endif