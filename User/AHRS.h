#ifndef __AHRS__H
#define __AHRS__H
#include "stm32f4xx.h"
#include "imu.h"

#define M_PI 	(float)3.1415926535

uint32_t micros(void);
void AHRS_init(void);
void AHRS_getYawPitchRoll(float * angles);
class _AHRS
{
public:
	void Init(void);
	void getYawPitchRoll(float * angles);
	void Init_Tim(void);
	void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
	void newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz);
	uint8_t _isRdy(void);
	void InitGyro_Offset(void);
	
private:
	uint8_t sensoraddr;
	volatile int16_t  FIFO[6][11];
	volatile float _exInt, _eyInt, _ezInt;  // 误差积分
	volatile uint32_t _lastUpdate, _now; // 采样周期计数 单位 us
	volatile float _q0, _q1, _q2, _q3; // 全局四元数
	int16_t _Gx_offset,_Gy_offset,_Gz_offset;
};


#endif


