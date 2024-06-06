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

uint8_t CHA_FLAG,INSERT_FLAG;
float Surplus_capacity_percent,Surplus_capacity,Total_capacity,Current;//电池剩余容量百分比、剩余容量、总容量
//电池电量信息发送至429
void Send_T_429(void)
{
	Deal_Data();
	if(Charge_Flag)
	{
		Current = ((Battery_data[3]<<8|Battery_data[4])&0x8000) ? ((0xffff - (Battery_data[3]<<8|Battery_data[4]))/(-100.0)):((Battery_data[3]<<8|Battery_data[4])/100.0);
		Total_capacity = (Battery_data[7]<<8|Battery_data[8])/100.0;//单位10mAH,变为AH
		Surplus_capacity = (Battery_data[5]<<8|Battery_data[6])/100.0;//单位10mAH,变为AH
		Surplus_capacity_percent =  Battery_data[11]/100.0;//剩余容量百分比
		#if 0
		printf("**********************TEST START**********************\n\r");
		printf("电池总量:%fmAh\n\r",Total_capacity*1000);
		printf("电池剩余容量:%fmAh\n\r",Surplus_capacity*1000);
		printf("电池剩余容量百分比:%f\n\r",Surplus_capacity_percent);
		printf("**********************TEST STOP***********************\n\r");
		if(Current > 0)
			BAT_LED_ON;
		else
			BAT_LED_OFF;
		#endif
		USART_Send_bytes(USART3,Battery_data,12);
		Charge_Flag = 0;
		
		//memcpy(SendBuff3,Battery_data,12); 
		//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送 
		//MYDMA_Enable(DMA1_Channel2);     //开始一次DMA传输！
	}
}
void Charing_Detection()
{
	if(CHA_INSERT)//检测到充电
	{
		  if(INSERT_FLAG)
			{
				if((Surplus_capacity_percent >= 1.0))
				{
					CHARGE_DIS;
					return;
				}
				if((Surplus_capacity_percent > 0.90))
				{
					return;
				}
			}
			
			if(!INSERT_FLAG)
			{
				CHARGE_DIS;//先断继电器，判别此处检测的充电是充电桩给的还是电池给的
				delay_ms(100);
				if(CHA_INSERT)//断开充电继电器后，判断是否依然有电-----说明是充电桩充电
				{
					CHARGE_EN;//打开充电继电器
					INSERT_FLAG = 1;//充电桩接触标志
					//CHA_FLAG = 1;//标记已充电
				}
				else//说明是电池供电
				{
					CHARGE_DIS;//未充电，关闭继电器
					CHA_FLAG = 0;
					INSERT_FLAG =0;
				}
			}
			else
			{
				if(Current >= 0)//4A充电器
				//if(Current > 0)//6A充电器
				{
					CHA_FLAG = 1;//标记已充电
					//INSERT_FLAG = 1;
				}	
				else 
				{	
					if(CHA_FLAG)//已充电
				   INSERT_FLAG = 0;
					BAT_LED_OFF;
					//CHARGE_DIS;//关闭继电器
					CHA_FLAG = 0; 
				}
			}
			
		}
		else
		{ 
			//CHARGE_DIS;//关闭继电器
			BAT_LED_OFF;
			CHA_FLAG = 0; 
			INSERT_FLAG = 0;
		}
}
