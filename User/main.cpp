#include "stm32f4xx.h"
#include "timer.h"
#include "cled.h"
#include "imu.h"
#include "console.h"
#include "cspi.h"
#include "printf_.h"

#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(GPIOB, GPIO_Pin_11)    //????
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(GPIOB, GPIO_Pin_11)  

#define SPI_FLASH_CS2_LOW()       GPIO_ResetBits(GPIOD, GPIO_Pin_8)    //????
#define SPI_FLASH_CS2_HIGH()      GPIO_SetBits(GPIOD, GPIO_Pin_8)  

uint16_t SPI_SendHalfWord(uint16_t data);
int main(void)
{
	uint8_t temp;
	SystemCoreClockUpdate();
	BaseTimer::Instance()->initialize();
	SPI::Instance()->Init(SPI1);
	SPI::Instance()->Init(SPI2);
	SPI::Instance()->Init(SPI3);
	Imu::Instance()->Init(IMU1);
	LED::Instance()->OFF();
	printf_init();
	//Imu_Spi_Init();
	Timer HeartBeat(500,100);
	
	while(1)
	{
		if(HeartBeat.isAbsoluteTimeUp())
		{
			Imu::Instance()->Update(IMU1);
			myprintf("ACCEL_x:%4x ",Imu::Instance()->GetData(IMU1).Accel.x);
			myprintf("ACCEL_y:%4x ",Imu::Instance()->GetData(IMU1).Accel.y);
			myprintf("ACCEL_z:%4x \r\n",Imu::Instance()->GetData(IMU1).Accel.z);
			LED::Instance()->Toggle();
		}
	}
}

uint16_t SPI_SendHalfWord(uint16_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI2, data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	return SPI_I2S_ReceiveData(SPI2);
}


