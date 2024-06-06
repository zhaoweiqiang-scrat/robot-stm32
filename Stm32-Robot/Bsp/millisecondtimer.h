#ifdef __cplusplus
extern "C" {
#endif
#ifndef _MILLISECONDTIMER_H_
#define _MILLISECONDTIMER_H_
#include "stm32f4xx.h"

void initialise(void);
void delay(uint32_t millis_);
uint32_t millis(void);
void reset(void);
extern volatile uint32_t _counter;

#endif // _MILLISECONDTIMER_H_ 
#ifdef __cplusplus
}
#endif



