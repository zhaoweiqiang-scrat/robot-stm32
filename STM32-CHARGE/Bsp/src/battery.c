//该模块是读取电池电量信息
#include "main.h"
#include "battery.h"
#include "usart.h"

_Battery_Info_ Battery_Info;//电池电量信息：电压、电流、剩余电量、总电量、保护状态、剩余电量百分比
uint8_t Battery_data[12];//要发布至429的数据
uint8_t Charge_Flag;//电池电量读取成功标志

uint8_t CHeck(uint8_t num)
{
	uint16_t sum=0,number=0,i=0;
	number=Usart1Type.RX_pData[3]+7;
	if(number>num)//超过上传数据 0x03指令收到34个值，0x04指令收到21个值，0x05指令收到27个值
		return 0;
	for(i=2;i<number-3;i++)
	 sum+=Usart1Type.RX_pData[i];
	sum = ~sum + 1;//取反+1
	if(sum==(Usart1Type.RX_pData[number-3]<<8|Usart1Type.RX_pData[number-2]))
	{
	//	memcpy(data,RX_BUF,number);
		return 1;
	}
	else
    return 0;
}

void Transmit_Cmd(void)
{
	uint8_t CMD_Info_Sta[7] = {0XDD,0XA5,0X03,0X00,0XFF,0XFD,0X77};//读03 读取基本信息及状态
	//设置为发送模式
	TX_EN;
	RX_DIS;
	USART_Send_bytes(USART1,CMD_Info_Sta,7);
	//修改为接收模式
	RX_EN;
	TX_DIS;
}
void Deal_Data(void)
{
		if(Usart1Type.RX_flag)    	// Receive flag
		{  
			Usart1Type.RX_flag=0;	// clean flag
			/*处理0x03命令*/
			if(Usart1Type.RX_Size == 0x22)
			{
				if(CHeck(0x22))
				{
					Battery_data[0] = 0XDD;
					Battery_data[1] = Usart1Type.RX_pData[4];//电池电压高字节
					Battery_data[2] = Usart1Type.RX_pData[5];//电池电压低字节
					Battery_data[3] = Usart1Type.RX_pData[6];//电池总电流高字节 最高位为1，为放电
					Battery_data[4] = Usart1Type.RX_pData[7];//电池总电流低字节
					Battery_data[5] = Usart1Type.RX_pData[8];//电池剩余容量高字节
					Battery_data[6] = Usart1Type.RX_pData[9];//电池剩余容量低字节
					Battery_data[7] = Usart1Type.RX_pData[10];//电池标称容量高字节
					Battery_data[8] = Usart1Type.RX_pData[11];//电池标称容量低字节
					Battery_data[9] = Usart1Type.RX_pData[20];//保护状态
					Battery_data[10] = Usart1Type.RX_pData[21];//保护状态
					Battery_data[11] = Usart1Type.RX_pData[23];//剩余容量百分比
					Charge_Flag = 1;//标示已接收到电池基本信息
				}
			}
		}
	
}
//电池电量信息发送至429
void Send_T_429(void)
{
	Deal_Data();
	if(Charge_Flag)
	{
		USART_Send_bytes(USART3,Battery_data,12);
		//memcpy(SendBuff3,Battery_data,12); 
		//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送 
		//MYDMA_Enable(DMA1_Channel2);     //开始一次DMA传输！
	}
}
