#include "imu.h"
#include "printf_.h"
#include "timer.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "arm_math.h"

uint8_t Tx_Buf[16]={ACCEL_XOUT_H|0x80,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

void delay_ms(uint16_t nms)
{
	Timer delay(0,nms);
	while(delay.isAbsoluteTimeUp()!=true) ;
}

void mget_ms(unsigned long *time)
{
}

uint8_t Cimu::ReadReg(uint8_t addr,uint16_t IMUx)
{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin_x;
	
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,DISABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,DISABLE);
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
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	return SPI_I2S_ReceiveData(SPIx);
}

uint8_t MPU_ReadReg(uint8_t IMUx,uint8_t reg)
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
	SPI_I2S_SendData(SPIx, reg|0x80);
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

uint8_t MPU_WriteReg(uint8_t reg,uint8_t data,uint16_t IMUx)
{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin_x;
	
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,DISABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,DISABLE);
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
	SPI_I2S_SendData(SPIx, reg);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		;
	SPI_I2S_ReceiveData(SPIx);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPIx, data);
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		;
	GPIO_WriteBit(GPIOx,GPIO_Pin_x,Bit_SET);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	return SPI_I2S_ReceiveData(SPIx);
}

uint8_t MPU_Read(uint8_t IMUx,uint8_t reg,uint8_t len,uint8_t * buf)
{
	for(uint8_t i=0;i<len;++i)
	{
		buf[i]=MPU_ReadReg(IMUx,reg+i);
	}
	return 0;
}

//鍐欏瘎瀛樺櫒
uint8_t MPU_Write(uint8_t IMUx,uint8_t reg,uint8_t len, uint8_t *buf)
{
	for(uint8_t i=0;i<len;++i)
	{
		MPU_WriteReg(reg+i,buf[i],IMUx);
	}
	return 0;
}

uint8_t Cimu::WriteReg(uint8_t addr,uint8_t data,uint16_t IMUx)
{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin_x;
	
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,DISABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,DISABLE);
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
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	return SPI_I2S_ReceiveData(SPIx);
}

//初始化SPI1、SPI2、SPI3接口
void Cimu::Init_SPI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2|RCC_APB1Periph_SPI3,ENABLE);
	
	//SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=SPI1_CS1_Pinx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI1_CS1_Port,&GPIO_InitStructure);
	GPIO_WriteBit(SPI1_CS1_Port,SPI1_CS1_Pinx,Bit_SET);
	
	GPIO_InitStructure.GPIO_Pin=SPI1_CS2_Pinx;
	GPIO_Init(SPI1_CS2_Port,&GPIO_InitStructure);
	GPIO_WriteBit(SPI1_CS2_Port,SPI1_CS2_Pinx,Bit_SET);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;           //Full Duplex
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                            //
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                                    //keep SCK high when free
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                                  //
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                     //NSS软件管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;           //波特率
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                             //大端模式
	SPI_InitStructure.SPI_CRCPolynomial = 7;                                       //CRC多项式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                                  //主机模式
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
	//SPI2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=SPI2_CS1_Pinx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI2_CS1_Port,&GPIO_InitStructure);
	GPIO_WriteBit(SPI2_CS1_Port,SPI2_CS1_Pinx,Bit_SET);
	
	GPIO_InitStructure.GPIO_Pin=SPI2_CS2_Pinx;
	GPIO_Init(SPI2_CS2_Port,&GPIO_InitStructure);
	GPIO_WriteBit(SPI2_CS2_Port,SPI2_CS2_Pinx,Bit_SET);
	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;           //波特率
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
	//SPI3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=SPI3_CS1_Pinx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI3_CS1_Port,&GPIO_InitStructure);
	GPIO_WriteBit(SPI3_CS1_Port,SPI3_CS1_Pinx,Bit_SET);
	
	GPIO_InitStructure.GPIO_Pin=SPI3_CS2_Pinx;
	GPIO_Init(SPI3_CS2_Port,&GPIO_InitStructure);
	GPIO_WriteBit(SPI3_CS2_Port,SPI3_CS2_Pinx,Bit_SET);
	
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
}

void Cimu::Init_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//SPI1 DMA
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //??DMA??
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //????
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;        //???0??
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //???????
	DMA_InitStructure.DMA_BufferSize = 32;                                  //??????
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //???????
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //??????
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //?????8?
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //?????8?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //???????
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //??????
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //???FIFO??
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //???FIFO??,???????
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //????
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //????
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //??DMA??
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //????
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;        //???0??
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //???????
	DMA_InitStructure.DMA_BufferSize = 32;                                  //??????
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //???????
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //??????
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //?????8?
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //?????8?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //???????
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //??????
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //???FIFO??
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           //???FIFO??,???????
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //????
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //????
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,DISABLE);
}

void Cimu::Init(uint8_t IMUx)
{
	WriteReg(PWR_MGMT_1,0x01,IMUx);			//解除休眠，自动选择最佳的时钟源
	WriteReg(CONFIG,0x00,IMUx);					//低通滤波频率，典型值：0x07（3600Hz） 决定Internal_Sample_Rate==8K
	
	/**********************Init INT**********************************/	
	WriteReg(INT_PIN_CFG ,0x10,IMUx);// INT PIN
	WriteReg(INT_ENABLE,0x00,IMUx);
	
	/*******************Init GYRO and ACCEL******************************/	
	WriteReg(SMPLRT_DIV, 0x17,IMUx);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
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
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);														//IMU0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource7);
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI->IMR&=(~EXTI_Line7);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //????? 2,
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //????? 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //????????
	NVIC_Init(&NVIC_InitStructure); //??????????
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOE,&GPIO_InitStructure);														//IMU1
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource10);
	EXTI_InitStructure.EXTI_Line=EXTI_Line10;
	EXTI_Init(&EXTI_InitStructure);
	EXTI->IMR&=(~EXTI_Line10);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure); //??????????
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_Init(GPIOD,&GPIO_InitStructure);														//IMU2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource12);
	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_Init(&EXTI_InitStructure);
	EXTI->IMR&=(~EXTI_Line12);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure); //??????????
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_Init(GPIOD,&GPIO_InitStructure);														//IMU3
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource14);
	EXTI_InitStructure.EXTI_Line=EXTI_Line14;
	EXTI_Init(&EXTI_InitStructure);
	EXTI->IMR&=(~EXTI_Line14);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure); //??????????
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOD,&GPIO_InitStructure);														//IMU4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource5);
	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
	EXTI_Init(&EXTI_InitStructure);
	EXTI->IMR&=(~EXTI_Line5);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure); //??????????
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_Init(GPIOD,&GPIO_InitStructure);														//IMU5
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource4);
	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
	EXTI_Init(&EXTI_InitStructure);
	EXTI->IMR&=(~EXTI_Line4);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure); //??????????
}

//使能数据自动更新，1kHz
void Cimu::AutoUpdate(FunctionalState state)
{
	uint32_t mask=0;
	mask=IMU0_IT_Linex|IMU1_IT_Linex|IMU2_IT_Linex|IMU3_IT_Linex|IMU4_IT_Linex|IMU5_IT_Linex;
	
	DMA_Cmd(DMA2_Stream2,DISABLE);
	DMA_Cmd(DMA2_Stream3,DISABLE);
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TCIF2,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	DMA2_Stream2->NDTR=16;
	DMA2_Stream3->NDTR=16;
	DMA2_Stream2->CR|=DMA_MemoryInc_Enable;
	DMA2_Stream3->CR|=DMA_MemoryInc_Enable;
	DMA2_Stream2->M0AR=(uint32_t)DMA_Buf[0];					//锛侊紒锛両MU0绗竴甯ф暟鎹彲鑳藉叾浠朓MU鏁版嵁
	DMA2_Stream3->M0AR=(uint32_t)Tx_Buf;
	DMA2_Stream2->M1AR=SPI1_DR_ADDR;
	DMA2_Stream3->M1AR=SPI1_DR_ADDR;
	
	if(state==ENABLE)
	{
		DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
		EXTI->IMR|=mask;
	}else{
		DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,DISABLE);
		EXTI->IMR&=(~mask);
	}
	IMU0_CS=1;
	IMU1_CS=1;
}

void Cimu::DMARead(SPI_TypeDef* SPIx,uint8_t* tx_buf,uint8_t* rx_buf,uint16_t lenth)
{
	if(SPIx==SPI1)
	{
		IMU0_CS=0;
		
		DMA_ITConfig(DMA2_Stream2,DMA_IT_TCIF2,DISABLE);
		DMA_ITConfig(DMA2_Stream3,DMA_IT_TCIF3,DISABLE); 
			/* ??DMA?? */
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);
		
		/* ??????? */
		DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)lenth);
		DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)lenth);
		/* ???????????? */
		DMA2_Stream2->CR |= (1 << 10);
		DMA2_Stream3->CR |= (1 << 10);
		
		/* ???????????? */
		DMA2_Stream2->M0AR = (uint32_t)rx_buf;
		DMA2_Stream3->M0AR = (uint32_t)tx_buf;

		/* ????DR,???? */
		SPIx->DR;
		
		/* ??????? */
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
		
		/* ??DMA?? */
		DMA_Cmd(DMA2_Stream2, ENABLE);
		DMA_Cmd(DMA2_Stream3, ENABLE);
		
		/* ???? */
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		
		/* ??DMA?? */
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		
		/* ??DMA?????? */
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
		
		DMA_ITConfig(DMA2_Stream2,DMA_IT_TCIF2,ENABLE);
		IMU0_CS=1;
	}
}

void Cimu::EnableITx(uint8_t IMUx,FunctionalState state)
{
	if(state==ENABLE)
	{
		WriteReg(INT_ENABLE,0x01,IMUx);
	}else{
		WriteReg(INT_ENABLE,0x00,IMUx);
	}
}

//闄�铻轰华涓柇澶勭悊鍑芥暟
void Cimu::IT_Handler(void)
{
	if(EXTI_GetITStatus(IMU0_IT_Linex)==SET)
	{
		if(IMU1_CS==0) 
		{
			EXTI_ClearITPendingBit(IMU0_IT_Linex);
			return ;
		}
		//while(DMA2_Stream2->CR&=DMA_SxCR_EN) ;
		//while(DMA2_Stream3->CR&=DMA_SxCR_EN) ;
		IMU0_CS=0;
		DMA2_Stream2->M0AR=(uint32_t)DMA_Buf[0];
		DMA2_Stream2->NDTR=16;
		DMA2_Stream3->NDTR=16;
		DMA2->LIFCR|=DMA_IT_TCIF2;							//锛侊紒锛佸繀闇�鍏堟竻闄ゆ爣蹇椾綅锛屾墠鑳戒娇鑳斤激锛肌浼犺緭
		DMA2->LIFCR|=DMA_IT_TCIF3;
		DMA2->LIFCR|=DMA_IT_FEIF3;
		DMA2_Stream2->CR|=DMA_SxCR_EN;
		DMA2_Stream3->CR|=DMA_SxCR_EN;
		EXTI_ClearITPendingBit(IMU0_IT_Linex);
		++SampleCount[0];
		//myprintf("IMU0\r\n");
	}
	
	if(EXTI_GetITStatus(IMU1_IT_Linex)==SET)
	{
		if(IMU0_CS==0) 
		{
			EXTI_ClearITPendingBit(IMU1_IT_Linex);
			return ;
		}
		IMU1_CS=0;
		DMA2_Stream2->NDTR=16;
		DMA2_Stream3->NDTR=16;
		DMA2_Stream2->M0AR=(uint32_t)DMA_Buf[1];
		DMA2->LIFCR|=DMA_IT_TCIF2;							//锛侊紒锛佸繀闇�鍏堟竻闄ゆ爣蹇椾綅锛屾墠鑳戒娇鑳斤激锛肌浼犺緭
		DMA2->LIFCR|=DMA_IT_TCIF3;
		DMA2->LIFCR|=DMA_IT_FEIF3;
		DMA2_Stream2->CR|=DMA_SxCR_EN;
		DMA2_Stream3->CR|=DMA_SxCR_EN;
		EXTI_ClearITPendingBit(IMU1_IT_Linex);
		++SampleCount[1];
		//myprintf("IMU1\r\n");
	}
	
	if(EXTI_GetITStatus(IMU2_IT_Linex)==SET)
	{
		EXTI_ClearITPendingBit(IMU2_IT_Linex);
	}
	if(EXTI_GetITStatus(IMU3_IT_Linex)==SET)
	{
		EXTI_ClearITPendingBit(IMU3_IT_Linex);
	}
	if(EXTI_GetITStatus(IMU4_IT_Linex)==SET)
	{
		EXTI_ClearITPendingBit(IMU4_IT_Linex);
	}
	if(EXTI_GetITStatus(IMU5_IT_Linex)==SET)
	{
		EXTI_ClearITPendingBit(IMU5_IT_Linex);
	}
}

uint32_t Cimu::GetSampleCount(uint8_t IMUx)
{
	uint32_t temp=SampleCount[IMUx];
	SampleCount[IMUx]=0;
	return temp;
}

uint16_t Cimu::GetAccel()
{
	return ((uint16_t)DMA_Buf[0][2]<<8)+DMA_Buf[0][3];
}

//////////////////////////////////////////////////////////////////////////
//DMP鐩稿叧鍑芥暟
/////////////////////////////////////////////////////////////////////////

//q30鏍煎紡,long杞琭loat鏃剁殑闄ゆ暟.
#define q30  1073741824.0f

//闄�铻轰华鏂瑰悜璁剧疆
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
//MPU6050鑷祴璇�
//杩斿洖鍊�:0,姝ｅ父
//    鍏朵粬,澶辫触
u8 run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}
//闄�铻轰华鏂瑰悜鎺у埗
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar; 
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
//鏂瑰悜杞崲
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//mpu6050,dmp鍒濆鍖�
//杩斿洖鍊�:0,姝ｅ父
//    鍏朵粬,澶辫触
u8 mpu_dmp_init(void)
{
	u8 res=0;
	if(mpu_init(0)==0)	//鍒濆鍖朚PU6050
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//璁剧疆鎵�闇�瑕佺殑浼犳劅鍣�
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//璁剧疆FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(1000);	//璁剧疆閲囨牱鐜�
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//鍔犺浇dmp鍥轰欢
		myprintf("load DMP:%d\r\n",res);
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//璁剧疆闄�铻轰华鏂瑰悜
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//璁剧疆dmp鍔熻兘
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(200);	//璁剧疆DMP杈撳嚭閫熺巼(鏈�澶т笉瓒呰繃200Hz)
		if(res)return 7;   
		res=run_self_test();		//鑷
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//浣胯兘DMP
		if(res)return 9;     
	}
	return 0;
}

int MPU_NormalInit(void)
{
	mpu_init(0);
	mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);
	mpu_set_sample_rate(1000);
	return mpu_run_self_test(0,0);
}
//寰楀埌dmp澶勭悊鍚庣殑鏁版嵁(娉ㄦ剰,鏈嚱鏁伴渶瑕佹瘮杈冨鍫嗘爤,灞�閮ㄥ彉閲忔湁鐐瑰)
//pitch:淇话瑙� 绮惧害:0.1掳   鑼冨洿:-90.0掳 <---> +90.0掳
//roll:妯粴瑙�  绮惧害:0.1掳   鑼冨洿:-180.0掳<---> +180.0掳
//yaw:鑸悜瑙�   绮惧害:0.1掳   鑼冨洿:-180.0掳<---> +180.0掳
//杩斿洖鍊�:0,姝ｅ父
//    鍏朵粬,澶辫触
u8 mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30鏍煎紡杞崲涓烘诞鐐规暟
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		//璁＄畻寰楀埌淇话瑙�/妯粴瑙�/鑸悜瑙�
		*pitch = arm_sin_f32(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else return 2;
	return 0;
}

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void EXTI9_5_IRQHandler(void)
{
	Imu::Instance()->IT_Handler();
}

void EXTI15_10_IRQHandler(void)
{
	Imu::Instance()->IT_Handler();
}

void EXTI4_IRQHandler(void)
{
	Imu::Instance()->IT_Handler();
}

void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2)==SET)
	{
		IMU0_CS=1;
		IMU1_CS=1;
		DMA2_Stream2->CR&=(~DMA_SxCR_EN);
		DMA2_Stream3->CR&=(~DMA_SxCR_EN);
		//娓呴櫎涓柇鏍囧織浣�
		DMA2->LIFCR|=DMA_IT_TCIF2;
	}
}

#ifdef __cplusplus
}
#endif

