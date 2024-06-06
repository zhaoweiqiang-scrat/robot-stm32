#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"

void JXBS3001GZ_Init(u32 bound)	;
void numinit();
void JXBS3001GZ_Send(u8 *buf);
int calculate(void);
extern u32 Light;
#ifdef __cplusplus
}
#endif