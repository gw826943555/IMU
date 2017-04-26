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

uint8_t Tx_buf[100]={0x01,0x02,0x03,0x04,0x05,0x06,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
										};
float Rx_buf[10];
uint8_t WhoAmI[4]={WHO_AM_I|0x80,0xff,0xff,0xFF};
uint8_t _WhoAmI[4];
short RxBuf[10];
int main(void)
{
	uint32_t system_micrsecond;
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
	Imu::Instance()->Init_IT();
	Imu::Instance()->EnableITx(IMU1,DISABLE);
	Imu::Instance()->AutoUpdate(DISABLE);
	LED::Instance()->OFF();
	Timer HeartBeat(500,1000);
	Timer Delay(500,10);
	uint32_t count=0;
	uint32_t count1=0;
	
//	Imu::Instance()->EnableITx(IMU0,ENABLE);
	//MPU_NormalInit();
	system_micrsecond=micros();
	AHRS_init();
	Imu::Instance()->EnableITx(IMU0,ENABLE);
	while(1)
	{
		AHRS_getYawPitchRoll(Rx_buf);
		if(HeartBeat.isAbsoluteTimeUp())
		{
//			MPU_GetAccel(IMU0,RxBuf,0);
			myprintf("ACCEL0:%f %f %f \r\n",Rx_buf[0],Rx_buf[1],Rx_buf[2]);
//			MPU_GetGyro(IMU0,RxBuf,0);
//			myprintf("GRRO0:%6d %6d %6d \r\n",RxBuf[0],RxBuf[1],RxBuf[2]);
//			MPU_GetAccel(IMU1,RxBuf,0);
//			myprintf("ACCEL1:%6d %6d %6d ",RxBuf[0],RxBuf[1],RxBuf[2]);
//			MPU_GetGyro(IMU1,RxBuf,0);
//			myprintf("GRRO1:%6d %6d %6d \r\n",RxBuf[0],RxBuf[1],RxBuf[2]);
//			MPU_GetAccel(IMU2,RxBuf,0);
//			myprintf("ACCEL2:%6d %6d %6d ",RxBuf[0],RxBuf[1],RxBuf[2]);
//			MPU_GetGyro(IMU2,RxBuf,0);
//			myprintf("GRRO2:%6d %6d %6d \r\n",RxBuf[0],RxBuf[1],RxBuf[2]);
//			myprintf("Int:0x%2x\r\n",MPU_ReadReg(0,I2C_MST_CTRL));
			
			LED::Instance()->Toggle();
		}
		if(Delay.isAbsoluteTimeUp())
		{
			WhoAmI[1]=0xff;
		}
	}
}






