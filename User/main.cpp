#include "stm32f4xx.h"
#include "timer.h"
#include "cled.h"
#include "imu.h"
#include "console.h"
#include "cspi.h"
#include "printf_.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

uint8_t Tx_buf[100]={0x01,0x02,0x03,0x04,0x05,0x06,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
										};
uint8_t Rx_buf[100];
uint8_t WhoAmI[4]={WHO_AM_I|0x80,0xff,0xff,0xFF};
uint8_t _WhoAmI[4];
int16_t RxBuf[10];
int main(void)
{
	BaseTimer::Instance()->initialize();
	printf_init();
	Imu::Instance()->Init_SPI();
	//Imu::Instance()->Init_DMA();
	Imu::Instance()->Init(IMU0);
	Imu::Instance()->Init(IMU1);
	Imu::Instance()->Init(IMU2);
	Imu::Instance()->Init(IMU3);
	Imu::Instance()->Init(IMU4);
	Imu::Instance()->Init(IMU5);
	//Imu::Instance()->Init_IT();
	//Imu::Instance()->EnableITx(IMU0,ENABLE);
	Imu::Instance()->EnableITx(IMU1,DISABLE);
	Imu::Instance()->AutoUpdate(DISABLE);
	LED::Instance()->OFF();
	Timer HeartBeat(500,1000);
	Timer Delay(500,10);
	uint32_t count=0;
	uint32_t count1=0;
	myprintf("Test:0x%2X\r\n",MPU_NormalInit());
	//MPU_NormalInit();
	while(1)
	{
		if(HeartBeat.isAbsoluteTimeUp())
		{
			//myprintf("mpu_init:%d\r\n",mpu_get_accel_reg(RxBuf,0));
			//myprintf("data:%d %d %d\r\n",RxBuf[0],RxBuf[1],RxBuf[2]);
			LED::Instance()->Toggle();
		}
		if(Delay.isAbsoluteTimeUp())
		{
			WhoAmI[1]=0xff;
		}
	}
}






