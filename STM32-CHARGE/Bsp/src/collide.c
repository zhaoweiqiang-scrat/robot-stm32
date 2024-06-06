#include "main.h"

uint8_t COL_FRONT_LEFT[4] = {0XFF,0X01,0X00,0X77};
uint8_t COL_FRONT_RIGHT[4] = {0XFF,0X02,0X00,0X77};
uint8_t COL_LATER_LEFT[4] = {0XFF,0X03,0X00,0X77};
uint8_t COL_LATER_RIGHT[4] = {0XFF,0X04,0X00,0X77};

//发送碰撞信息至429
void Collide_Info(uint8_t state)
{
	switch(state)
	{
		case 1:
					//左前沿
					if(Flag_Front_Left)
					{
						COL_FRONT_LEFT[2] = 0X01;
						USART_Send_bytes(USART3,COL_FRONT_LEFT,4);
						COL_FRONT_LEFT[2] = 0X00;
					}
					else
					{
						USART_Send_bytes(USART3,COL_FRONT_LEFT,4);
					}
					break;
		case 2:
					//右前沿
					if(Flag_Front_Right)
					{
						COL_FRONT_RIGHT[2] = 0X01;
						USART_Send_bytes(USART3,COL_FRONT_RIGHT,4);
						COL_FRONT_RIGHT[2] = 0X00;
					}
					else
					{
						USART_Send_bytes(USART3,COL_FRONT_RIGHT,4);	
					}
					break;
		case 3:
					//左后沿
					if(Flag_Later_Left)
					{
						COL_LATER_LEFT[2] = 0X01;
						USART_Send_bytes(USART3,COL_LATER_LEFT,4);	
						COL_LATER_LEFT[2] = 0X00;
					}
					else
					{
						USART_Send_bytes(USART3,COL_LATER_LEFT,4);
					}
					break;
		case 4:
					//右后沿
					if(Flag_Later_Right)
					{
						COL_LATER_RIGHT[2] = 0X01;
						USART_Send_bytes(USART3,COL_LATER_RIGHT,4);	
						COL_LATER_RIGHT[2] = 0X00;
					}
					else
					{
						USART_Send_bytes(USART3,COL_LATER_RIGHT,4);
					}
					break;
		default:
					break;
	}
	
	#if 1
	
	
	
	#endif
}
