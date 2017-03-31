#include "cspi.h"

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

uint8_t Cspi::RWByte(uint8_t data,SPI_TypeDef* SPIx,GPIO_TypeDef* SPIx_CSx_Port,uint16_t SPIx_CSx_Pinx)
{
	while(SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_TXE)==RESET) ;
	GPIO_WriteBit(SPIx_CSx_Port,SPIx_CSx_Pinx,Bit_RESET);
	SPI_I2S_SendData(SPIx,data);
	while(SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_RXNE)==RESET) ;
	GPIO_WriteBit(SPIx_CSx_Port,SPIx_CSx_Pinx,Bit_SET);
	return SPI_I2S_ReceiveData(SPIx);
}
