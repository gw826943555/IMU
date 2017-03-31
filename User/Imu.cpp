#include "imu.h"
#include "cspi.h"

//uint8_t Cimu::ReadReg(uint8_t addr,SPI_TypeDef* SPIx,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x)
//{
//	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_RESET);
//	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
//		;
//	SPI_I2S_SendData(SPIx, addr|0x80);
//	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
//		;
//	SPI_I2S_ReceiveData(SPIx);
//	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
//		;
//	SPI_I2S_SendData(SPIx, 0xff);
//	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
//		;
//	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_SET);
//	return SPI_I2S_ReceiveData(SPIx);
//}

uint8_t Cimu::ReadReg(uint8_t addr,uint16_t IMUx)
{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin_x;
	
	switch(IMUx)
	{
		case IMU0:SPIx=SPI1,GPIOx=SPI1_CS1_Port,GPIO_Pin_x=SPI1_CS1_Pinx;break;
		case IMU1:SPIx=SPI1,GPIOx=SPI1_CS2_Port,GPIO_Pin_x=SPI1_CS2_Pinx;break;
		case IMU2:SPIx=SPI2,GPIOx=SPI2_CS1_Port,GPIO_Pin_x=SPI2_CS1_Pinx;break;
		case IMU3:SPIx=SPI2,GPIOx=SPI2_CS2_Port,GPIO_Pin_x=SPI2_CS2_Pinx;break;
		case IMU4:SPIx=SPI3,GPIOx=SPI3_CS1_Port,GPIO_Pin_x=SPI3_CS1_Pinx;break;
		case IMU5:SPIx=SPI3,GPIOx=SPI3_CS2_Port,GPIO_Pin_x=SPI3_CS2_Pinx;break;
		default: return 0;
	}
	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_RESET);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPIx, addr|0x80);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		;
	SPI_I2S_ReceiveData(SPIx);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPIx, 0xff);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		;
	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_SET);
	return SPI_I2S_ReceiveData(SPIx);
}

uint8_t Cimu::WriteReg(uint8_t addr,uint8_t data,uint16_t IMUx)
{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin_x;
	
	switch(IMUx)
	{
		case IMU0:SPIx=SPI1,GPIOx=SPI1_CS1_Port,GPIO_Pin_x=SPI1_CS1_Pinx;break;
		case IMU1:SPIx=SPI1,GPIOx=SPI1_CS2_Port,GPIO_Pin_x=SPI1_CS2_Pinx;break;
		case IMU2:SPIx=SPI2,GPIOx=SPI2_CS1_Port,GPIO_Pin_x=SPI2_CS1_Pinx;break;
		case IMU3:SPIx=SPI2,GPIOx=SPI2_CS2_Port,GPIO_Pin_x=SPI2_CS2_Pinx;break;
		case IMU4:SPIx=SPI3,GPIOx=SPI3_CS1_Port,GPIO_Pin_x=SPI3_CS1_Pinx;break;
		case IMU5:SPIx=SPI3,GPIOx=SPI3_CS2_Port,GPIO_Pin_x=SPI3_CS2_Pinx;break;
		default: return 0;
	}
	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_RESET);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPIx, addr);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		;
	SPI_I2S_ReceiveData(SPIx);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPIx, data);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		;
	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_SET);
	return SPI_I2S_ReceiveData(SPIx);
}

void Cimu::Init(uint8_t IMUx)
{
	WriteReg(PWR_MGMT_1,0x00,IMUx);			//解除休眠
	WriteReg(CONFIG,0x07,IMUx);					//低通滤波频率，典型值：0x07（3600Hz） 决定Internal_Sample_Rate==8K
	
	/**********************Init INT**********************************/	
	WriteReg(INT_PIN_CFG ,0xF0,IMUx);// INT PIN
	WriteReg(INT_ENABLE,0x01,IMUx);
	
	/*******************Init GYRO and ACCEL******************************/	
	WriteReg(SMPLRT_DIV, 0x07,IMUx);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	WriteReg(GYRO_CONFIG, 0x18,IMUx); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	WriteReg(ACCEL_CONFIG, 0x18,IMUx);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
	WriteReg(ACCEL_CONFIG2, 0x08,IMUx);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）	
}

void Cimu::Update(uint8_t IMUx)
{
	IMUData[IMUx].Accel.x=ReadReg(ACCEL_XOUT_L,IMUx);
	IMUData[IMUx].Accel.x+=(((uint16_t)ReadReg(ACCEL_XOUT_H,IMUx))<<8);
	IMUData[IMUx].Accel.y=ReadReg(ACCEL_YOUT_L,IMUx);
	IMUData[IMUx].Accel.y+=(((uint16_t)ReadReg(ACCEL_YOUT_H,IMUx))<<8);
	IMUData[IMUx].Accel.z=ReadReg(ACCEL_ZOUT_L,IMUx);
	IMUData[IMUx].Accel.z+=(((uint16_t)ReadReg(ACCEL_ZOUT_H,IMUx))<<8);
	
	IMUData[IMUx].GYRO.x=ReadReg(GYRO_XOUT_L,IMUx);
	IMUData[IMUx].GYRO.x+=(((uint16_t)ReadReg(GYRO_XOUT_H,IMUx))<<8);
	IMUData[IMUx].GYRO.y=ReadReg(GYRO_YOUT_L,IMUx);
	IMUData[IMUx].GYRO.y+=(((uint16_t)ReadReg(GYRO_YOUT_H,IMUx))<<8);
	IMUData[IMUx].GYRO.z=ReadReg(GYRO_ZOUT_L,IMUx);
	IMUData[IMUx].GYRO.z+=(((uint16_t)ReadReg(GYRO_ZOUT_H,IMUx))<<8);
}

IMU_Data Cimu::GetData(uint8_t IMUx)
{
	return IMUData[IMUx];
}

void  Cimu::Init_IT(void)
{
}

