#include "main.h"

unsigned char IR_Trxing_flag=0;

void Nec_remote_IR_Send(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,unsigned int CusCode,unsigned char data)
{
	unsigned char num_send=16;
	unsigned char data2=0;
 IR_Trxing_flag=1;

 data2=data;
 IR_head(GPIOx,GPIO_Pin);
 //Send customer Code 16bits
	#if 0
  while(num_send)
  {
	 if(CusCode&0x8000)
	 {
		IR_Send_1(GPIOx,GPIO_Pin);
	 }
	 else 
	 {
		IR_Send_0(GPIOx,GPIO_Pin);
	 }
	 CusCode=CusCode<<1;
	 num_send--;
  }
  #endif
//	Send data 8bits
  num_send=8;
  while(num_send)
  {
	 if(data&0x80)
	 {
		IR_Send_1(GPIOx,GPIO_Pin);
	 }
	 else 
	 {
		IR_Send_0(GPIOx,GPIO_Pin);
	 }
	 data=data<<1;
	 num_send--;
  }
//	Send ^data 8bits
	#if 0
  num_send=8;
  while(num_send)
  {
	 if(data2&0x80)
	 {
		IR_Send_0(GPIOx,GPIO_Pin);
	 }
	 else 
	 {
		IR_Send_1(GPIOx,GPIO_Pin);
	 }
	 data2=data2<<1;
	 num_send--;
  }
	#endif
  IR_stop(GPIOx,GPIO_Pin);
  IR_Trxing_flag=0;
}

void IR_head(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_SetBits(GPIOx,GPIO_Pin);
	delay_us(9000); 
 	GPIO_ResetBits(GPIOx,GPIO_Pin);
	delay_us(4500); 

}
void IR_stop(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_SetBits(GPIOx,GPIO_Pin);
	delay_us(560); 
 	GPIO_ResetBits(GPIOx,GPIO_Pin);
}
void IR_Send_0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_SetBits(GPIOx,GPIO_Pin);
	delay_us(560); 
 	GPIO_ResetBits(GPIOx,GPIO_Pin);
	delay_us(560); 
	
}

void IR_Send_1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_SetBits(GPIOx,GPIO_Pin);
	delay_us(560); 
 	GPIO_ResetBits(GPIOx,GPIO_Pin);
	delay_us(1700); 
	
}

void IR_Send_ContrlNum(uint8_t state)
{
	if(state)
	{
		//Nec_remote_IR_Send(IR_TXD1_GPIO_Port,IR_TXD1_Pin,0x7f80,0x35);//右边红外发射0x35控制码
	  //delay_ms(2);
	  //Nec_remote_IR_Send(IR_TXD2_GPIO_Port,IR_TXD2_Pin,0x7f80,0x36);//左边红外发射0x36控制码
	  //delay_ms(2);
		Nec_remote_IR_Send(IR_TXD3_GPIO_Port,IR_TXD3_Pin,0x7f80,0x37);//中间红外发射0x37控制码
		delay_ms(2);
	}
	else
	{
		Nec_remote_IR_Send(IR_TXD1_GPIO_Port,IR_TXD1_Pin,0x7f80,0x38);//右边红外发射0x35控制码
	  delay_ms(2);
	  Nec_remote_IR_Send(IR_TXD2_GPIO_Port,IR_TXD2_Pin,0x7f80,0x38);//左边红外发射0x36控制码
	  delay_ms(2);
		Nec_remote_IR_Send(IR_TXD3_GPIO_Port,IR_TXD3_Pin,0x7f80,0x38);//中间红外发射0x38控制码	抵达充电位置
	  delay_ms(2);
	}
}
