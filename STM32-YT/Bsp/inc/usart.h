#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stdbool.h"
#define LIGHT_CONTR   0X01//����LED��
#define PAN_CONTR     0X02//������̨����
#define TILT_CONTR    0X03//������̨��ת
#define DOU_CONTR     0X04//ͬʱ�·�������ת


#define WRITE_DATA    0x01//���豸д����
#define READ_DATA     0x02//��ȡ�豸���� 
#define INITIAL_SYS   0x03//��ʼ���豸
#define FEEDBACK_DATA 0x04//��̨�����м�״̬
#define PAUSE_RUNING  0x05//��̨��ͣ��ǰ����
#define WARRING       0x06//��̨���쳣
#define CW_AUTO       0x08//˳ʱ���Զ���ת
#define CCW_AUTO      0x09//��ʱ���Զ���ת
#define STOP_AUTO     0x0A//ֹͣ��ת
#define UART_RX_LEN 20
#define SEND_BUF_SIZE 20	//�������ݳ���

#define D_TYPE 				0X00//��׼pelco-dЭ��,����Ϊ0
#define SET_HV_TYPE 	0XE3//������̨ˮƽ��ֱ��׼0
#define ANGLE_ON_TYPE 0XE1//���ýǶȻش�

#define H_POS_ANG			0X4B//ˮƽ�Ƕȶ�λ����
#define V_POS_ANG			0X4D//��ֱ�Ƕȶ�λ����
#define H_ANG_BACK		0X51//��̨ˮƽ�ǶȲ�ѯ
#define V_ANG_BACK		0X53//��̨��ֱ�ǶȲ�ѯ
#define ANG_BACK_ON		0X01//�Ƕ��Զ��ش�
#define SET_HV				0X03//���û�׼0

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
extern u8 SendBuff1[SEND_BUF_SIZE];	//����2�������ݻ�����
#endif
