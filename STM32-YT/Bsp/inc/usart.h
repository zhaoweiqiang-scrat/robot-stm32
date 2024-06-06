#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stdbool.h"
#define LIGHT_CONTR   0X01//控制LED灯
#define PAN_CONTR     0X02//控制云台俯仰
#define TILT_CONTR    0X03//控制云台横转
#define DOU_CONTR     0X04//同时下发俯仰横转


#define WRITE_DATA    0x01//给设备写数据
#define READ_DATA     0x02//读取设备数据 
#define INITIAL_SYS   0x03//初始化设备
#define FEEDBACK_DATA 0x04//云台进入中间状态
#define PAUSE_RUNING  0x05//云台暂停当前运行
#define WARRING       0x06//云台有异常
#define CW_AUTO       0x08//顺时针自动旋转
#define CCW_AUTO      0x09//逆时针自动旋转
#define STOP_AUTO     0x0A//停止自转
#define UART_RX_LEN 20
#define SEND_BUF_SIZE 20	//发送数据长度

#define D_TYPE 				0X00//标准pelco-d协议,类型为0
#define SET_HV_TYPE 	0XE3//设置云台水平垂直基准0
#define ANGLE_ON_TYPE 0XE1//设置角度回传

#define H_POS_ANG			0X4B//水平角度定位类型
#define V_POS_ANG			0X4D//垂直角度定位类型
#define H_ANG_BACK		0X51//云台水平角度查询
#define V_ANG_BACK		0X53//云台垂直角度查询
#define ANG_BACK_ON		0X01//角度自动回传
#define SET_HV				0X03//设置基准0

void HY_Send_out(uint8_t type, uint8_t content1,uint8_t content2, uint8_t content3);
void Usart1_Int(uint32_t BaudRatePrescaler);
void Usart2_Int(uint32_t BaudRatePrescaler);
void Usart3_Int(uint32_t BaudRatePrescaler);
void USART_send_byte(USART_TypeDef* USARTx,uint8_t byte);
void USART_Send_bytes(USART_TypeDef* USARTx,uint8_t *Buffer, uint8_t Length);
void send_out(uint8_t *data,uint8_t action,uint8_t function);
void Send_out(uint8_t *data1,uint8_t *data2,uint8_t action,uint8_t function);
void set_rx_tx(uint8_t rx_state,uint8_t tx_state);
void DMA_Configuration(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);
extern uint8_t stata;
extern uint8_t YT_Flag_Angle,YT_Flag_Angle1,YT_Flag_Angle2;
extern uint8_t RX_BUF[15],RX_BUF1[15],RX_BUF2[15];
extern uint8_t Uart_Rx[UART_RX_LEN];
extern u8 SendBuff1[SEND_BUF_SIZE];	//串口2发送数据缓冲区
#endif
