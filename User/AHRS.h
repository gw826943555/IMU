#ifndef __AHRS__H
#define __AHRS__H
#include "stm32f4xx.h"
#include "imu.h"

#define M_PI 	(float)3.1415926535

uint32_t micros(void);
void AHRS_init(void);
void AHRS_getYawPitchRoll(float * angles);


#endif


