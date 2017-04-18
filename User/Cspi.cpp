#include "cspi.h"

#define SPI1_DR_ADDR    (uint32_t)0x4001300C
void Cspi::Init(SPI_TypeDef* SPIx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	if(SPIx==SPI1)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
		
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
	}else if(SPIx==SPI2)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
		
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
		
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;           //Full Duplex
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                            //
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                                    //keep SCK high when free
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                                  //
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                     //NSS软件管理
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;           //波特率
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                             //大端模式
		SPI_InitStructure.SPI_CRCPolynomial = 7;                                       //CRC多项式
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                                  //主机模式
		SPI_Init(SPI2, &SPI_InitStructure);
		SPI_Cmd(SPI2, ENABLE);
	}else if(SPIx==SPI3)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
		
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
		
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;           //Full Duplex
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                            //
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                                    //keep SCK high when free
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                                  //
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                     //NSS软件管理
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;           //波特率
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                             //大端模式
		SPI_InitStructure.SPI_CRCPolynomial = 7;                                       //CRC多项式
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                                  //主机模式
		SPI_Init(SPI3, &SPI_InitStructure);
		SPI_Cmd(SPI3, ENABLE);
	}else{
		;
	}
}

void Cspi::InitDMA(SPI_TypeDef * SPIx)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	if(SPIx==SPI1)
	{
		SPI_I2S_DMACmd(SPIx,SPI_I2S_DMAReq_Tx,ENABLE);
		SPI_I2S_DMACmd(SPIx,SPI_I2S_DMAReq_Rx,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
		/* DMA RX config */
		DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          //??DMA??
		DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                //????
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_buf;        //???0??
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
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_buf;        //???0??
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
		
		DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
	}
}
