#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
void UART4_Init(u32 bound);
void LIFTER_Contr(void);
void YT_Contr(void);	
void MCU2YT(uint8_t *Buffer, uint8_t Length);
extern float Height,V_angle,H_angle;
extern	u8 uart4_tx_buf[20],Height_Flag1,Height_Flag2,Yt_Flag1,Yt_Flag2,Led_Ack;
#ifdef __cplusplus
}
#endif