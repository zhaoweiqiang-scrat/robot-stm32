#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stdbool.h"

#define BAT_485_DE_Pin GPIO_Pin_11
#define BAT_485_RE_Pin GPIO_Pin_12
#define BAT_485_DE_GPIO_Port GPIOA
#define BAT_485_RE_GPIO_Port GPIOA

#define TX_EN  GPIO_SetBits(BAT_485_DE_GPIO_Port, BAT_485_DE_Pin)
#define TX_DIS GPIO_ResetBits(BAT_485_DE_GPIO_Port, BAT_485_DE_Pin)
#define RX_EN  GPIO_ResetBits(BAT_485_RE_GPIO_Port, BAT_485_RE_Pin)
#define RX_DIS GPIO_SetBits(BAT_485_RE_GPIO_Port, BAT_485_RE_Pin)

#define UART_RX_LEN 20
#define SEND_BUF_SIZE 20	//发送数据长度

#define RX_LEN 100  

extern u8 SendBuff3[SEND_BUF_SIZE];	//串口3发送数据缓冲区

typedef struct  
{  
uint8_t  RX_flag:1;        //IDLE receive flag
uint16_t RX_Size;          //receive length
uint8_t  RX_pData[RX_LEN]; //DMA receive buffer
}USART_RECEIVETYPE;  

extern USART_RECEIVETYPE Usart1Type;
extern USART_RECEIVETYPE Usart2Type;
extern USART_RECEIVETYPE Usart3Type;

void Usart1_Int(uint32_t BaudRatePrescaler);
void Usart2_Int(uint32_t BaudRatePrescaler);
void Usart3_Int(uint32_t BaudRatePrescaler);
void USART_send_byte(USART_TypeDef* USARTx,uint8_t byte);
void USART_Send_bytes(USART_TypeDef* USARTx,uint8_t *Buffer, uint8_t Length);
void send_out(uint8_t *data,uint8_t action,uint8_t function);
void Send_out(uint8_t *data1,uint8_t *data2,uint8_t action,uint8_t function);

void DMA_Configuration(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);
extern uint8_t stata;
extern uint8_t YT_Flag_Angle,YT_Flag_Angle1,YT_Flag_Angle2;
extern uint8_t RX_BUF[15],RX_BUF1[15],RX_BUF2[15];
extern uint8_t Uart_Rx[UART_RX_LEN];
extern u8 SendBuff1[SEND_BUF_SIZE];	//串口2发送数据缓冲区
#endif
