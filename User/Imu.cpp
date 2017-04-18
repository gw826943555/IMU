#include "imu.h"
#include "printf_.h"

uint8_t Tx_Buf[16]={ACCEL_XOUT_H|0x80,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

namespace
{
	//´ÖÂÔÑÓÊ±
	void delay_us(uint32_t nus)
	{
		int32_t temp=SysTick->VAL;
		temp=temp-nus*SystickUsBase;
		while(SysTick->VAL> temp) ;
	}

	void delay_ms(uint32_t nms)
	{
		nms*=500;
		for(uint32_t i=0;i<nms;++i)
		{
			delay_us(1);
		}
	}
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

//³õÊ¼»¯SPI1¡¢SPI2¡¢SPI3½Ó¿Ú
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
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                     //NSSÈí¼þ¹ÜÀí
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;           //²¨ÌØÂÊ
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                             //´ó¶ËÄ£Ê½
	SPI_InitStructure.SPI_CRCPolynomial = 7;                                       //CRC¶àÏîÊ½
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                                  //Ö÷»úÄ£Ê½
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
	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;           //²¨ÌØÂÊ
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,DISABLE);
}

void Cimu::Init(uint8_t IMUx)
{
	WriteReg(PWR_MGMT_1,0x01,IMUx);			//½â³ýÐÝÃß£¬×Ô¶¯Ñ¡Ôñ×î¼ÑµÄÊ±ÖÓÔ´
	WriteReg(CONFIG,0x07,IMUx);					//µÍÍ¨ÂË²¨ÆµÂÊ£¬µäÐÍÖµ£º0x07£¨3600Hz£© ¾ö¶¨Internal_Sample_Rate==8K
	
	/**********************Init INT**********************************/	
	WriteReg(INT_PIN_CFG ,0x10,IMUx);// INT PIN
	WriteReg(INT_ENABLE,0x00,IMUx);
	
	/*******************Init GYRO and ACCEL******************************/	
	WriteReg(SMPLRT_DIV, 0x07,IMUx);  //ÍÓÂÝÒÇ²ÉÑùÂÊ£¬µäÐÍÖµ£º0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	WriteReg(GYRO_CONFIG, 0x18,IMUx); //ÍÓÂÝÒÇ×Ô¼ì¼°²âÁ¿·¶Î§£¬µäÐÍÖµ£º0x18(²»×Ô¼ì£¬2000deg/s)
	WriteReg(ACCEL_CONFIG, 0x18,IMUx);//¼ÓËÙ¼Æ×Ô¼ì¡¢²âÁ¿·¶Î§¼°¸ßÍ¨ÂË²¨ÆµÂÊ£¬µäÐÍÖµ£º0x18(²»×Ô¼ì£¬16G)
	WriteReg(ACCEL_CONFIG2, 0x08,IMUx);//¼ÓËÙ¼Æ¸ßÍ¨ÂË²¨ÆµÂÊ µäÐÍÖµ £º0x08  £¨1.13kHz£©	
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

//Ê¹ÄÜÊý¾Ý×Ô¶¯¸üÐÂ£¬1kHz
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
	DMA2_Stream2->M0AR=(uint32_t)DMA_Buf[0];
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
	myprintf("INT_ENABLE:0x%X\r\n",ReadReg(INT_ENABLE,IMUx));
}

//é™€èžºä»ªä¸­æ–­å¤„ç†å‡½æ•°
void Cimu::IT_Handler(void)
{
	if(EXTI_GetITStatus(IMU0_IT_Linex)==SET)
	{
		while(IMU1_CS==0) ;
		IMU0_CS=0;
		DMA2->LIFCR|=DMA_IT_TCIF2;							//ï¼ï¼ï¼å¿…éœ€å…ˆæ¸…é™¤æ ‡å¿—ä½ï¼Œæ‰èƒ½ä½¿èƒ½ï¼¤ï¼­ï¼¡ä¼ è¾“
		DMA2->LIFCR|=DMA_IT_TCIF3;
		DMA2_Stream2->CR|=DMA_SxCR_EN;
		DMA2_Stream3->CR|=DMA_SxCR_EN;
		EXTI_ClearITPendingBit(IMU0_IT_Linex);
	}
	
	if(EXTI_GetITStatus(IMU1_IT_Linex)==SET)
	{
		while(IMU0_CS==0) ;
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);
		IMU1_CS=0;
		DMA2_Stream2->NDTR=16;
		DMA2_Stream3->NDTR=16;
		DMA2_Stream2->CR|=DMA_MemoryInc_Enable;
		DMA2_Stream3->CR|=DMA_MemoryInc_Enable;
		DMA2_Stream2->M0AR=(uint32_t)DMA_Buf[0];
		DMA2_Stream3->M0AR=(uint32_t)Tx_Buf;
		DMA2_Stream2->CR|=DMA_SxCR_EN;
		DMA2_Stream3->CR|=DMA_SxCR_EN;
		EXTI_ClearITPendingBit(IMU1_IT_Linex);
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
		//æ¸…é™¤ä¸­æ–­æ ‡å¿—ä½
		DMA2->LIFCR|=DMA_IT_TCIF2;
	}
}
	 
#ifdef __cplusplus
}
#endif

