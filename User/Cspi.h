#ifndef __CSPI__H
#define __CSPI__H
#include "stm32f4xx.h"
#include "singleton.h"
#define SPI1_CS1_Port 		GPIOA
#define SPI1_CS1_Pinx			GPIO_Pin_3
#define SPI1_FSYNC1_Port	GPIOE
#define SPI1_FSYNC1_Pinx	GPIO_Pin_8
#define SPI1_Int1_Port		GPIOE
#define SPI1_Int1_Pinx		GPIO_Pin_7

#define SPI1_CS2_Port 		GPIOE
#define SPI1_CS2_Pinx			GPIO_Pin_9
#define SPI1_FSYNC2_Port	GPIOE
#define SPI1_FSYNC2_Pinx	GPIO_Pin_11
#define SPI1_Int2_Port		GPIOE
#define SPI1_Int2_Pinx		GPIO_Pin_10

#define SPI2_CS1_Port 		GPIOB
#define SPI2_CS1_Pinx			GPIO_Pin_11
#define SPI2_FSYNC1_Port	GPIOD
#define SPI2_FSYNC1_Pinx	GPIO_Pin_13
#define SPI2_Int1_Port		GPIOD
#define SPI2_Int1_Pinx		GPIO_Pin_12

#define SPI2_CS2_Port 		GPIOD
#define SPI2_CS2_Pinx			GPIO_Pin_8
#define SPI2_FSYNC2_Port	GPIOD
#define SPI2_FSYNC2_Pinx	GPIO_Pin_15
#define SPI2_Int2_Port		GPIOD
#define SPI2_Int2_Pinx		GPIO_Pin_14

#define SPI3_CS1_Port 		GPIOB
#define SPI3_CS1_Pinx			GPIO_Pin_7
#define SPI3_FSYNC1_Port	GPIOD
#define SPI3_FSYNC1_Pinx	GPIO_Pin_6
#define SPI3_Int1_Port		GPIOD
#define SPI3_Int1_Pinx		GPIO_Pin_5

#define SPI3_CS2_Port 		GPIOB
#define SPI3_CS2_Pinx			GPIO_Pin_6
#define SPI3_FSYNC2_Port	GPIOD
#define SPI3_FSYNC2_Pinx	GPIO_Pin_3
#define SPI3_Int2_Port		GPIOD
#define SPI3_Int2_Pinx		GPIO_Pin_4

class Cspi
{
public:
	void Init(SPI_TypeDef *SPIx);
	void InitDMA(SPI_TypeDef *SPIx);
	void Bsp_Init(void);
private:
	uint8_t Tx_buf[100],Rx_buf[100];
};

typedef NormalSingleton<Cspi> SPI;
#endif




