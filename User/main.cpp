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

void DMP_Init(void)
{
	const signed char gyro_orientation[9] = {1, 0, 0,0, 1, 0,0, 0, 1};
	int res=0;
	res=mpu_init(0);
	myprintf("mpu_init:%d\r\n",res);
	res=mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	myprintf("mpu_set_sensors:%d\r\n",res);
	res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	myprintf("mpu_configure_fifo:%d\r\n",res);
	res=mpu_set_sample_rate(200);
	myprintf("mpu_set_sample_rate:%d\r\n",res);
	res=dmp_load_motion_driver_firmware();
	myprintf("dmp_load_motion_driver_firmware:%d\r\n",res);
	res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	myprintf("dmp_set_orientation:%d\r\n",res);					
	res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
                     DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	myprintf("dmp_enable_feature:%d\r\n",res);		
	res=dmp_set_fifo_rate(200);
	myprintf("dmp_set_fifo_rate:%d\r\n",res);	
	res=mpu_set_dmp_state(1);
	myprintf("mpu_set_dmp_state:%d\r\n",res);
	res=run_self_test();
	myprintf("run_self_test:%d\r\n",res);
}

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
	Imu::Instance()->Init_IT();
	Imu::Instance()->EnableITx(IMU1,DISABLE);
	Imu::Instance()->AutoUpdate(DISABLE);
	LED::Instance()->OFF();
	Timer HeartBeat(500,1000);
	Timer Delay(500,10);
	uint32_t count=0;
	uint32_t count1=0;
	
	AHRS0.Init_Tim();
	AHRS0.Init();
	Imu::Instance()->EnableITx(IMU0,ENABLE);
	short gyro[3],accl[3],sensors;
	long quat[4];
	uint8_t more;
	int res=0;
	while(1)
	{
		++count;
		AHRS0.getYawPitchRoll(Rx_buf);
		if(HeartBeat.isAbsoluteTimeUp())
		{
			myprintf("%d\r\n",count);
			count=0;
			AHRS0.getMotion6(&accl[0],&accl[1],&accl[2],&gyro[0],&gyro[1],&gyro[2]);
			myprintf("gyro:%d %d %d\r\n",gyro[0],gyro[1],gyro[2]);
			myprintf("gyro:%f %f %f \r\n",Rx_buf[0],Rx_buf[1],Rx_buf[2]);
			LED::Instance()->Toggle();
		}
		if(Delay.isAbsoluteTimeUp())
		{
			WhoAmI[1]=0xff;
		}
	}
}






