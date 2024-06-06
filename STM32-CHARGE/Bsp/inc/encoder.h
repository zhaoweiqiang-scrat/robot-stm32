#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f10x.h"
void Encoder_init(void);
int32_t Encoder_read(void);
void Encoder_set_pos(int32_t pos);
#endif
