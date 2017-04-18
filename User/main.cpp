#include "stm32f4xx.h"
#include "timer.h"
#include "cled.h"
#include "imu.h"
#include "console.h"
#include "cspi.h"
#include "printf_.h"

uint8_t Tx_buf[100]={	ACCEL_XOUT_L|0x80,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
										};
uint8_t Rx_buf[100];
uint8_t WhoAmI[4]={WHO_AM_I|0x80,0xff,0xff,0xFF};
uint8_t _WhoAmI[4];
int main(void)
{
	BaseTimer::Instance()->initialize();
	printf_init();
	Imu::Instance()->Init_SPI();
	Imu::Instance()->Init_DMA();
	Imu::Instance()->Init(IMU0);
	Imu::Instance()->Init(IMU1);
	Imu::Instance()->Init(IMU2);
	Imu::Instance()->Init(IMU3);
	Imu::Instance()->Init(IMU4);
	Imu::Instance()->Init(IMU5);
	Imu::Instance()->Init_IT();
	Imu::Instance()->AutoUpdate(ENABLE);
	Imu::Instance()->EnableITx(IMU0,ENABLE);
	LED::Instance()->OFF();
	Timer HeartBeat(500,100);
	Timer Delay(500,10);
	uint32_t count=0;
	uint32_t count1=0;
	while(1)
	{
		if(HeartBeat.isAbsoluteTimeUp())
		{
			//myprintf("ID1:%2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X\r\n",Rx_buf[1],Rx_buf[2],Rx_buf[3],Rx_buf[4],Rx_buf[5],Rx_buf[6],Rx_buf[7],Rx_buf[8],Rx_buf[9],Rx_buf[10],Rx_buf[11],Rx_buf[12],Rx_buf[13]);
			//myprintf("ID1:%2X\r\n",Imu::Instance()->ReadReg(ACCEL_XOUT_L,IMU0));
			//myprintf("ID2:%2X\r\n",Imu::Instance()->ReadReg(ACCEL_XOUT_H,IMU0));
			//Imu::Instance()->DMARead(SPI1,Tx_buf,Rx_buf,2);
			//myprintf("ID3:%2X\r\n",Imu::Instance()->DMA_Buf[0][1]);
			myprintf("ID4:%4X\r\n",(Imu::Instance()->DMA_Buf[0][1]<<8)+Imu::Instance()->DMA_Buf[0][2]);
			LED::Instance()->Toggle();
		}
		if(Delay.isAbsoluteTimeUp())
		{
			WhoAmI[1]=0xff;
			
		}
	}
}




