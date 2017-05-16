#include "stm32f4xx.h"
#include "timer.h"
#include "cled.h"
#include "imu.h"
#include "console.h"
#include "cspi.h"
#include "printf_.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ahrs.h"
#include "math.h"

uint8_t Tx_buf[100]={0x01,0x02,0x03,0x04,0x05,0x06,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
										};
float Rx_buf[10];
uint8_t WhoAmI[4]={WHO_AM_I|0x80,0xff,0xff,0xFF};
uint8_t _WhoAmI[4];
short RxBuf[10];

int main(void)
{
	BaseTimer::Instance()->initialize();
	printf_init();
	Imu::Instance()->Init_SPI();
	Imu::Instance()->Init_IT();
	Imu::Instance()->EnableITx(IMU1,DISABLE);
	Imu::Instance()->AutoUpdate(DISABLE);
	LED::Instance()->OFF();
	Timer HeartBeat(500,1000);
	uint32_t count=0;
	uint32_t count1=0;
	
	AHRS0.Init_Tim();
	AHRS0.Init();
	AHRS1.Init();
	AHRS2.Init();
	AHRS3.Init();
	AHRS4.Init();
	AHRS5.Init();
	short gyro[3],accl[3],sensors;
	long quat[4];
	uint8_t more;
	int res=0;
	while(1)
	{
		++count;
		AHRS0.getYawPitchRoll(Rx_buf);
		AHRS1.getYawPitchRoll(Rx_buf);
		AHRS2.getYawPitchRoll(Rx_buf);
		AHRS3.getYawPitchRoll(Rx_buf);
		AHRS4.getYawPitchRoll(Rx_buf);
		AHRS5.getYawPitchRoll(Rx_buf);
		if(HeartBeat.isAbsoluteTimeUp())
		{
			myprintf("loop:%d\r\n",count);
			count=0;
			myprintf("angle:%f %f %f \r\n",Rx_buf[0],Rx_buf[1],Rx_buf[2]);
			LED::Instance()->Toggle();
		}
	}
}






