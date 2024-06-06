#include "interrupt.h"
#include "mcu2yt.h"
//#ifdef  USE_SERIAL1
//HardwareSerial *Serial1=0 ;
//#endif  

#ifdef  USE_SERIAL2
HardwareSerial *Serial2=0 ;
#endif 
u8  usart1_len;
char usart1_ch;
u8 usart1_rx_buf[4];
//#ifdef  USE_SERIAL3
//HardwareSerial *Serial3=0 ;
//#endif 

void USART1_IRQHandler(void) 
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		usart1_ch=USART_ReceiveData(USART1);
		usart1_rx_buf[usart1_len++]=usart1_ch;
		if(usart1_len >= 4)
		{
			printf("数据1：0x%x",usart1_rx_buf[0]);
			printf("数据2：0x%x",usart1_rx_buf[1]);
			printf("数据3：0x%x",usart1_rx_buf[2]);
			printf("数据4：0x%x\r\n",usart1_rx_buf[3]);
			MCU2YT(usart1_rx_buf,usart1_len);
			usart1_len = 0;
		}	 
	}	
}
#if 0
void USART2_IRQHandler(void) 
{
#ifdef  USE_SERIAL2
		Serial2->irq();
#endif
}
#endif



