//��ģ���Ƕ�ȡ��ص�����Ϣ
#include "main.h"
#include "battery.h"
#include "usart.h"

_Battery_Info_ Battery_Info;//��ص�����Ϣ����ѹ��������ʣ��������ܵ���������״̬��ʣ������ٷֱ�
uint8_t Battery_data[12];//Ҫ������429������
uint8_t Charge_Flag;//��ص�����ȡ�ɹ���־

uint8_t CHeck(uint8_t num)
{
	uint16_t sum=0,number=0,i=0;
	number=Usart1Type.RX_pData[3]+7;
	if(number>num)//�����ϴ����� 0x03ָ���յ�34��ֵ��0x04ָ���յ�21��ֵ��0x05ָ���յ�27��ֵ
		return 0;
	for(i=2;i<number-3;i++)
	 sum+=Usart1Type.RX_pData[i];
	sum = ~sum + 1;//ȡ��+1
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
	uint8_t CMD_Info_Sta[7] = {0XDD,0XA5,0X03,0X00,0XFF,0XFD,0X77};//��03 ��ȡ������Ϣ��״̬
	//����Ϊ����ģʽ
	TX_EN;
	RX_DIS;
	USART_Send_bytes(USART1,CMD_Info_Sta,7);
	//�޸�Ϊ����ģʽ
	RX_EN;
	TX_DIS;
}
void Deal_Data(void)
{
		if(Usart1Type.RX_flag)    	// Receive flag
		{  
			Usart1Type.RX_flag=0;	// clean flag
			/*����0x03����*/
			if(Usart1Type.RX_Size == 0x22)
			{
				if(CHeck(0x22))
				{
					Battery_data[0] = 0XDD;
					Battery_data[1] = Usart1Type.RX_pData[4];//��ص�ѹ���ֽ�
					Battery_data[2] = Usart1Type.RX_pData[5];//��ص�ѹ���ֽ�
					Battery_data[3] = Usart1Type.RX_pData[6];//����ܵ������ֽ� ���λΪ1��Ϊ�ŵ�
					Battery_data[4] = Usart1Type.RX_pData[7];//����ܵ������ֽ�
					Battery_data[5] = Usart1Type.RX_pData[8];//���ʣ���������ֽ�
					Battery_data[6] = Usart1Type.RX_pData[9];//���ʣ���������ֽ�
					Battery_data[7] = Usart1Type.RX_pData[10];//��ر���������ֽ�
					Battery_data[8] = Usart1Type.RX_pData[11];//��ر���������ֽ�
					Battery_data[9] = Usart1Type.RX_pData[20];//����״̬
					Battery_data[10] = Usart1Type.RX_pData[21];//����״̬
					Battery_data[11] = Usart1Type.RX_pData[23];//ʣ�������ٷֱ�
					Charge_Flag = 1;//��ʾ�ѽ��յ���ػ�����Ϣ
				}
			}
		}
	
}

uint8_t CHA_FLAG,INSERT_FLAG;
float Surplus_capacity_percent,Surplus_capacity,Total_capacity,Current;//���ʣ�������ٷֱȡ�ʣ��������������
//��ص�����Ϣ������429
void Send_T_429(void)
{
	Deal_Data();
	if(Charge_Flag)
	{
		Current = ((Battery_data[3]<<8|Battery_data[4])&0x8000) ? ((0xffff - (Battery_data[3]<<8|Battery_data[4]))/(-100.0)):((Battery_data[3]<<8|Battery_data[4])/100.0);
		Total_capacity = (Battery_data[7]<<8|Battery_data[8])/100.0;//��λ10mAH,��ΪAH
		Surplus_capacity = (Battery_data[5]<<8|Battery_data[6])/100.0;//��λ10mAH,��ΪAH
		Surplus_capacity_percent =  Battery_data[11]/100.0;//ʣ�������ٷֱ�
		#if 0
		printf("**********************TEST START**********************\n\r");
		printf("�������:%fmAh\n\r",Total_capacity*1000);
		printf("���ʣ������:%fmAh\n\r",Surplus_capacity*1000);
		printf("���ʣ�������ٷֱ�:%f\n\r",Surplus_capacity_percent);
		printf("**********************TEST STOP***********************\n\r");
		if(Current > 0)
			BAT_LED_ON;
		else
			BAT_LED_OFF;
		#endif
		USART_Send_bytes(USART3,Battery_data,12);
		Charge_Flag = 0;
		
		//memcpy(SendBuff3,Battery_data,12); 
		//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA���� 
		//MYDMA_Enable(DMA1_Channel2);     //��ʼһ��DMA���䣡
	}
}
void Charing_Detection()
{
	if(CHA_INSERT)//��⵽���
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
				CHARGE_DIS;//�ȶϼ̵������б�˴����ĳ���ǳ��׮���Ļ��ǵ�ظ���
				delay_ms(100);
				if(CHA_INSERT)//�Ͽ����̵������ж��Ƿ���Ȼ�е�-----˵���ǳ��׮���
				{
					CHARGE_EN;//�򿪳��̵���
					INSERT_FLAG = 1;//���׮�Ӵ���־
					//CHA_FLAG = 1;//����ѳ��
				}
				else//˵���ǵ�ع���
				{
					CHARGE_DIS;//δ��磬�رռ̵���
					CHA_FLAG = 0;
					INSERT_FLAG =0;
				}
			}
			else
			{
				if(Current >= 0)//4A�����
				//if(Current > 0)//6A�����
				{
					CHA_FLAG = 1;//����ѳ��
					//INSERT_FLAG = 1;
				}	
				else 
				{	
					if(CHA_FLAG)//�ѳ��
				   INSERT_FLAG = 0;
					BAT_LED_OFF;
					//CHARGE_DIS;//�رռ̵���
					CHA_FLAG = 0; 
				}
			}
			
		}
		else
		{ 
			//CHARGE_DIS;//�رռ̵���
			BAT_LED_OFF;
			CHA_FLAG = 0; 
			INSERT_FLAG = 0;
		}
}
