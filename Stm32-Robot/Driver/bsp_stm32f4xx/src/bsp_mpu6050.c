/*
*********************************************************************************************************
*
*	ģ������ : ����������MPU-6050����ģ��
*	�ļ����� : bsp_mpu6050.c
*	��    �� : V1.0
*	˵    �� : ʵ��MPU-6050�Ķ�д������
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

/*
	Ӧ��˵��������MPU-6050ǰ�����ȵ���һ�� bsp_InitI2C()�������ú�I2C��ص�GPIO.
*/

#include "stm32f4xx.h"
#include "bsp_i2c_gpio.h"
#include "bsp_mpu6050.h"
#include "millisecondtimer.h"
MPU6050_T g_tMPU6050;		/* ����һ��ȫ�ֱ���������ʵʱ���� */

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitMPU6050
*	����˵��: ��ʼ��MPU-6050
*	��    ��:  ��
*	�� �� ֵ: 1 ��ʾ������ 0 ��ʾ������
*********************************************************************************************************
*/
void bsp_InitMPU6050(void)
{
	MPU6050_WriteByte(PWR_MGMT_1, 0x80);
	delay(100);
	MPU6050_WriteByte(PWR_MGMT_1, 0x00);	//�������״̬
	MPU6050_WriteByte(SMPLRT_DIV, 0x13);
	MPU6050_WriteByte(CONFIG, 0x04);
	MPU6050_WriteByte(GYRO_CONFIG, 0x18);
	MPU6050_WriteByte(ACCEL_CONFIG, 0x01);
}

/*
*********************************************************************************************************
*	�� �� ��: MPU6050_WriteByte
*	����˵��: �� MPU-6050 �Ĵ���д��һ������
*	��    ��: _ucRegAddr : �Ĵ�����ַ
*			  _ucRegData : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MPU6050_WriteByte(uint8_t _ucRegAddr, uint8_t _ucRegData)
{
    i2c_Start();							/* ���߿�ʼ�ź� */

    i2c_SendByte(MPU6050_SLAVE_ADDRESS);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();

    i2c_SendByte(_ucRegAddr);				/* �ڲ��Ĵ�����ַ */
	i2c_WaitAck();

    i2c_SendByte(_ucRegData);				/* �ڲ��Ĵ������� */
	i2c_WaitAck();

    i2c_Stop();                   			/* ����ֹͣ�ź� */
}

/*
*********************************************************************************************************
*	�� �� ��: MPU6050_ReadByte
*	����˵��: ��ȡ MPU-6050 �Ĵ���������
*	��    ��: _ucRegAddr : �Ĵ�����ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t MPU6050_ReadByte(uint8_t _ucRegAddr)
{
	uint8_t ucData;

	i2c_Start();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();
	i2c_SendByte(_ucRegAddr);     			/* ���ʹ洢��Ԫ��ַ */
	i2c_WaitAck();

	i2c_Start();                  			/* ���߿�ʼ�ź� */

	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1); 	/* �����豸��ַ+���ź� */
	i2c_WaitAck();

	ucData = i2c_ReadByte();       			/* �����Ĵ������� */
	i2c_NAck();
	i2c_Stop();                  			/* ����ֹͣ�ź� */
	return ucData;
}


/*
*********************************************************************************************************
*	�� �� ��: MPU6050_ReadData
*	����˵��: ��ȡ MPU-6050 ���ݼĴ����� ���������ȫ�ֱ��� g_tMPU6050.  ��������Զ�ʱ���øó���ˢ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MPU6050_ReadData(void)
{
	uint8_t ucReadBuf[14];
	uint8_t i;
	short raw;
	int AX,AY,AZ,GX,GY,GZ;
	
#if 0 /* ������ */
	i2c_Start();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();
	i2c_SendByte(ACCEL_XOUT_H);     		/* ���ʹ洢��Ԫ��ַ  */
	i2c_WaitAck();

	i2c_Start();                  			/* ���߿�ʼ�ź� */

	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1); /* �����豸��ַ+���ź� */
	i2c_WaitAck();

	for (i = 0; i < 13; i++)
	{
		ucReadBuf[i] = i2c_ReadByte();       			/* �����Ĵ������� */
		i2c_Ack();
	}

	/* �����һ���ֽڣ�ʱ�� NAck */
	ucReadBuf[13] = i2c_ReadByte();
	i2c_NAck();

	i2c_Stop();                  			/* ����ֹͣ�ź� */

#else	/* ���ֽڶ� */
	for (i = 0 ; i < 14; i++)
	{
		ucReadBuf[i] = MPU6050_ReadByte(ACCEL_XOUT_H + i);
	}
#endif

	/* �����������ݱ��浽ȫ�ֽṹ����� */
	#if 1
	AX = (ucReadBuf[0] << 8) + ucReadBuf[1];
	AY = (ucReadBuf[2] << 8) + ucReadBuf[3];
	AZ = (ucReadBuf[4] << 8) + ucReadBuf[5];

	GX = (ucReadBuf[8] << 8) + ucReadBuf[9];
	GY = (ucReadBuf[10] << 8) + ucReadBuf[11];
	GZ = (ucReadBuf[12] << 8) + ucReadBuf[13];
	if(AX & 0X8000)
	{	
	  AX ^= 0XFFFF;
	  g_tMPU6050.Accel_X = -AX*0.000061035;
	}
	else
	{
		g_tMPU6050.Accel_X = AX*0.000061035;
	}
	if(AY & 0X8000)
	{	
	  AY ^= 0XFFFF;
	  g_tMPU6050.Accel_Y = -AY*0.000061035;
	}
	else
	{
		g_tMPU6050.Accel_Y = AY*0.000061035;
	}
	if(AZ & 0X8000)
	{	
	  AZ ^= 0XFFFF;
	  g_tMPU6050.Accel_Z = -AZ*0.000061035;
	}
	else
	{
		g_tMPU6050.Accel_Z = AZ*0.000061035;
	}
  
	raw=(ucReadBuf[6]<<8)|ucReadBuf[7]; 
	g_tMPU6050.Temp = 36.53+((double)raw)/340;

	if(GX & 0X8000)
	{	
	  GX ^= 0XFFFF;
	  g_tMPU6050.GYRO_X = -GX*0.06103515625;
	}
	else
	{
		g_tMPU6050.GYRO_X = GX*0.06103515625;
	}
	if(GY & 0X8000)
	{	
	  GY ^= 0XFFFF;
	  g_tMPU6050.GYRO_Y = -GY*0.06103515625;
	}
	else
	{
		g_tMPU6050.GYRO_Y = GY*0.06103515625;
	}
	if(GZ & 0X8000)
	{	
	  GZ ^= 0XFFFF;
	  g_tMPU6050.GYRO_Z = -GZ*0.06103515625;
	}
	else
	{
		g_tMPU6050.GYRO_Z = GZ*0.06103515625;
	}

	#else
	/* �����������ݱ��浽ȫ�ֽṹ����� */
	g_tMPU6050.Accel_X = (ucReadBuf[0] << 8) + ucReadBuf[1];
	g_tMPU6050.Accel_Y = (ucReadBuf[2] << 8) + ucReadBuf[3];
	g_tMPU6050.Accel_Z = (ucReadBuf[4] << 8) + ucReadBuf[5];

	g_tMPU6050.Temp = (int16_t)((ucReadBuf[6] << 8) + ucReadBuf[7]);

	g_tMPU6050.GYRO_X = (ucReadBuf[8] << 8) + ucReadBuf[9];
	g_tMPU6050.GYRO_Y = (ucReadBuf[10] << 8) + ucReadBuf[11];
	g_tMPU6050.GYRO_Z = (ucReadBuf[12] << 8) + ucReadBuf[13];
	#endif
}
void MPU6050(void)
{
	long int delay_num=0,num;
	float Accel_X,Accel_Y,Accel_Z;
	float GYRO_X,GYRO_Y,GYRO_Z;
	float AX,AY,AZ,GX,GY,GZ;
	AX=0.0237487201;
	AY=-0.0218688361;;
	AZ=0.971055;
	GX=-1.63574219;
	GY=1.44042969;
	GZ=-0.640869141;
	MPU6050_ReadByte(WHO_AM_I);
	if (i2c_CheckDevice(MPU6050_SLAVE_ADDRESS) == 0)
	{
		printf("MPU-6050 Ok (0x%02X)\r\n", MPU6050_SLAVE_ADDRESS);
	}
	else
	{
		printf("MPU-6050 Err (0x%02X)\r\n", MPU6050_SLAVE_ADDRESS);
	}
# if 0
	Accel_X = 0.0;
  Accel_Y = 0.0;
	Accel_Z = 0.0;
	GYRO_X = 0.0;
	GYRO_Y = 0.0;
	GYRO_Z = 0.0;
	for(num=20;num>0;num--)
	{
		MPU6050_ReadData();		/* ��ȡ MPU-6050�����ݵ�ȫ�ֱ��� g_tMPU6050 */
		Accel_X += g_tMPU6050.Accel_X;
		Accel_Y += g_tMPU6050.Accel_Y;
		Accel_Z += g_tMPU6050.Accel_Z;
		GYRO_X += g_tMPU6050.GYRO_X;
		GYRO_Y += g_tMPU6050.GYRO_Y;
		GYRO_Z += g_tMPU6050.GYRO_Z;
	}
		
	  g_tMPU6050.Accel_X = Accel_X/20.0 - AX;
		g_tMPU6050.Accel_Y = Accel_Y/20.0 - AY;
		g_tMPU6050.Accel_Z = Accel_Z/20.0 ;

		g_tMPU6050.GYRO_X = GYRO_X/20.0 - GX;
		g_tMPU6050.GYRO_Y = GYRO_Y/20.0 - GY;
		g_tMPU6050.GYRO_Z = GYRO_Z/20.0 - GZ;	
#else
	  MPU6050_ReadData();	
	  g_tMPU6050.Accel_X -= AX;
		g_tMPU6050.Accel_Y -= AY;
		//g_tMPU6050.Accel_Z = Accel_Z ;

		g_tMPU6050.GYRO_X -= GX;
		g_tMPU6050.GYRO_Y -= GY;
		g_tMPU6050.GYRO_Z -= GZ;
#endif
	  if((g_tMPU6050.Accel_X < 0.01) || ((g_tMPU6050.Accel_X > -0.01)&&(g_tMPU6050.Accel_X <0)))
			g_tMPU6050.Accel_X = 0;
		if((g_tMPU6050.Accel_Y < 0.01) || ((g_tMPU6050.Accel_Y > -0.01)&&(g_tMPU6050.Accel_Y < 0)))
			g_tMPU6050.Accel_Y = 0;
		if((g_tMPU6050.Accel_Z < 0.01) || ((g_tMPU6050.Accel_Z > -0.01))&&(g_tMPU6050.Accel_Z < 0))
			g_tMPU6050.Accel_Z = 0;
		if((g_tMPU6050.GYRO_X < 0.01) || ((g_tMPU6050.GYRO_X > -0.01)&&(g_tMPU6050.GYRO_X < 0)))
			g_tMPU6050.GYRO_X = 0;
		if((g_tMPU6050.GYRO_Y < 0.01) || ((g_tMPU6050.GYRO_Y > -0.01)&&(g_tMPU6050.GYRO_Y < 0)))
			g_tMPU6050.GYRO_Y = 0;
		if((g_tMPU6050.GYRO_Z < 0.01) || ((g_tMPU6050.GYRO_Z > -0.01)&&(g_tMPU6050.GYRO_Z < 0)))
			g_tMPU6050.GYRO_Z = 0;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
