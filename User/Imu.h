#ifndef _IMU_H
#define _IMU_H
#include "stm32f4xx.h"
#include "singleton.h"
#define IMU0 						0
#define IMU0_IT_PORT		GPIOE
#define IMU0_IT_Pinx		GPIO_Pin_7

#define IMU1 						1
#define IMU1_IT_PORT		GPIOE
#define IMU1_IT_Pinx		GPIO_Pin_10

#define IMU2 						2
#define IMU2_IT_PORT		GPIOB
#define IMU2_IT_Pinx		GPIO_Pin_11

#define IMU3 						3
#define IMU3_IT_PORT		GPIOD
#define IMU3_IT_Pinx		GPIO_Pin_8

#define IMU4 						4
#define IMU4_IT_PORT		GPIOB
#define IMU4_IT_Pinx		GPIO_Pin_7

#define IMU5 						5
#define IMU5_IT_PORT		GPIOB
#define IMU5_IT_Pinx		GPIO_Pin_6

struct _ACCEL
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
};

struct _GYRO
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
};

struct _IMU_Data
{
	struct _ACCEL Accel;
	struct _GYRO GYRO;
	uint16_t Temp;
	uint8_t timestamp;
};

typedef struct _IMU_Data IMU_Data;

class Cimu
{
public:
//	uint8_t ReadReg(uint8_t addr,SPI_TypeDef* SPIx,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x);
	uint8_t ReadReg(uint8_t addr,uint16_t IMUx);
	uint8_t WriteReg(uint8_t addr,uint8_t data,uint16_t IMUx);
	void Init(uint8_t IMUx);
	void Init_IT(void);
	void Update(uint8_t IMUx);
	IMU_Data GetData(uint8_t IMUx);
private:
	IMU_Data IMUData[6];
};

typedef NormalSingleton<Cimu> Imu;

// 定义MPU9250内部寄存器
#define SELF_TEST_X_GYRO								0x00
#define SELF_TEST_Y_GYRO								0x01
#define SELF_TEST_Z_GYRO								0x02
#define SELF_TEST_X_ACCEL								0x0D
#define SELF_TEST_Y_ACCEL								0x0E
#define SELF_TEST_Z_ACCEL								0x0F
#define XG_OFFSET_H											0x13			
#define XG_OFFSET_L											0x14		
#define YG_OFFSET_H											0x15
#define YG_OFFSET_L											0x16		
#define ZG_OFFSET_H											0x17			
#define ZG_OFFSET_L											0x18
#define SMPLRT_DIV											0x19
#define CONFIG													0x1A
#define GYRO_CONFIG											0x1B
#define ACCEL_CONFIG										0x1c
#define ACCEL_CONFIG2										0x1d
#define LP_ACCEL_ODR										0x1e
#define WOM_THR													0x1f
#define	FIFO_EN													0x23
#define I2C_MST_CTRL										0x24
#define I2C_SLV0_ADDR										0x25
#define I2C_SLV0_REG										0x26
#define I2C_SLV0_CTRL										0x27
#define I2C_SLV1_ADDR										0x28
#define I2C_SLV1_REG										0x29
#define I2C_SLV1_CTRL										0x2a
#define I2C_SLV2_ADDR										0x2b
#define I2C_SLV2_REG										0x2c
#define I2C_SLV2_CTRL										0x2d
#define I2C_SLV3_ADDR										0x2e
#define I2C_SLV3_REG										0x2f
#define I2C_SLV3_CTRL										0x30
#define I2C_SLV4_ADDR										0x31
#define I2C_SLV4_REG										0x32
#define I2C_SLV4_DO											0x33
#define I2C_SLV4_CTRL										0x34
#define I2C_SLV4_DI											0x35
#define I2C_MST_STATUS									0x36
#define INT_PIN_CFG											0x37
#define INT_ENABLE											0x38
#define INT_STATUS											0x3A
#define ACCEL_XOUT_H										0x3b		
#define ACCEL_XOUT_L										0x3c			
#define ACCEL_YOUT_H										0x3d			
#define ACCEL_YOUT_L										0x3e						
#define ACCEL_ZOUT_H										0x3f						
#define ACCEL_ZOUT_L										0x40							
#define TEMP_OUT_H											0x41						
#define TEMP_OUT_L											0x42							
#define GYRO_XOUT_H											0x43					
#define GYRO_XOUT_L											0x44						
#define GYRO_YOUT_H											0x45						
#define GYRO_YOUT_L											0x46			
#define GYRO_ZOUT_H											0x47			
#define GYRO_ZOUT_L											0x48					
#define EXT_SENS_DATA_00								0x49									
#define EXT_SENS_DATA_01								0x4a									
#define EXT_SENS_DATA_02								0x4b									
#define EXT_SENS_DATA_03								0x4c								
#define EXT_SENS_DATA_04								0x4d									
#define EXT_SENS_DATA_05								0x4e							
#define EXT_SENS_DATA_06								0x4f										
#define EXT_SENS_DATA_07								0x50										
#define EXT_SENS_DATA_08								0x51									
#define EXT_SENS_DATA_09								0x52									
#define EXT_SENS_DATA_10								0x53							
#define EXT_SENS_DATA_11								0x54								
#define EXT_SENS_DATA_12								0x55												
#define EXT_SENS_DATA_13								0x56									
#define EXT_SENS_DATA_14								0x57										
#define EXT_SENS_DATA_15								0x58										
#define EXT_SENS_DATA_16								0x59								
#define EXT_SENS_DATA_17								0x5a									
#define EXT_SENS_DATA_18								0x5b										
#define EXT_SENS_DATA_19								0x5c										
#define EXT_SENS_DATA_20								0x5d												
#define EXT_SENS_DATA_21								0x5e									
#define EXT_SENS_DATA_22								0x5f									
#define EXT_SENS_DATA_23								0x60										
#define I2C_SLV0_DO											0x63									
#define I2C_SLV1_DO											0x64									
#define I2C_SLV2_DO											0x65							
#define I2C_SLV3_DO											0x66										
#define I2C_MST_DELAY_CTRL							0x67														
#define SIGNAL_PATH_RESET								0x68													
#define ACCEL_INTEL_CTRL								0x69													
#define USER_CTRL												0x6a										
#define PWR_MGMT_1											0x6b									
#define PWR_MGMT_2											0x6c										
#define FIFO_COUNT_H										0x72												
#define FIFO_COUNT_L										0x73													
#define FIFO_R_W												0x74								
#define WHO_AM_I												0x75							
#define XA_OFFSET_H											0x77							
#define XA_OFFSET_L											0x78									
#define YA_OFFSET_H											0x7a								
#define YA_OFFSET_L											0x7b											
#define ZA_OFFSET_H											0x7d									
#define ZA_OFFSET_L											0x7e										
#endif
//end of file
