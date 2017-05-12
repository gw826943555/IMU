#include "AHRS.h"
#include "arm_math.h"
#include "matrix.h"
#include "printf_.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0, q1, q2, q3; // 全局四元数
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
extern int16_t Gz_offset;
#define HISTORY_YAW_SIZE 10   //yaw length, short time update
#define _HISTORY_ERROR_TIME 20 //long time update 
#define GET_EZ -100 //get ez
#define LSB 16.03556f //32.8f
#define DSP2RAD 1.088412E-03//16.4f

static float Gz_Buf_132E[400];
static float Gz_Last_CEC;
static float sum_Gz_CE4;
static int16_t offset_Cnt_210 = 0;
static float _Threshold = 1*DSP2RAD;
static int16_t ram_234;

void getetint(int16_t* x, int16_t* y, int16_t* z)
{
	*x = (int16_t)(exInt*1000.0);
	*y = (int16_t)(eyInt*1000.0);
	*z = (int16_t)(ezInt*1000.0);
}

/**************************实现函数********************************************
*函数原型:		void Initial_Timer2(void)
*功　　能:	  初始化Tim2以产生一个32位的定时器来提供系统us 级的计时	
输入参数：		无
输出参数：		没有	
*******************************************************************************/
void Initial_Timer2(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
	/* TIM2 configuration*/ 
  /* Time Base configuration 基本配置 配置定时器的时基单元*/
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffffffff; //自动重装值         
  TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000-1;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  
	TIM_Cmd(TIM2, ENABLE);                  
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：		无
输出参数：		处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
	
 	return TIM2->CNT;
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:	   void AHRS_init(void)
*功　　能:	  初始化IMU相关	
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void AHRS_init(void)
{
	Initial_Timer2();
	MPU_NormalInit();
	//	HMC5883L_SetUp();
	delay_ms(50);
	//	MPU6050_initialize();

	//陀螺仪偏差
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;

	lastUpdate = micros();//更新时间
	now = micros();

	q0=1.0;
	q1=0;
	q2=0;
	q3=0;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getValues(float * values) {  
	int16_t accgyroval[6];
	int i;
	//读取加速度和陀螺仪的当前ADC
    MPU_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
    for(i = 0; i<6; i++) {
      if(i < 3) {
        values[i] = (float) accgyroval[i];
      }
      else {
				if ( accgyroval[i] <3 && accgyroval[i]> -3 ) {
					values[i] = 0;
				} else {
					values[i] = ((float) accgyroval[i]) / LSB; //转成度每秒
				}
		//这里已经将量程改成了 360度每秒  LSB 对应 1度每秒
      }
   }
}

void least_square(float* x, float* y,float* a, float* b, float* error){
	int length = HISTORY_YAW_SIZE;
	int i = 0;
	float t1, t2, t3, t4;
	float xbar, ybar;
	float s1, s2, s3;
	float r = 0;
	t1 = 0;
	t2 = 0;
	t3 = 0;
	t4 = 0;
	s1 = 0;
	s2 = 0;
	s3 = 0;
	xbar = 0;
	ybar = 0;
	for( i = 0; i < length; i++) {
		t1 += x[i]*x[i];
		t2 += x[i];
		t3 += x[i]*y[i];
		t4 += y[i];
	}
	if(fabs(length*t1 - t2*t2) > 1e-10) {
		*b = (t1*t4-t2*t3)/(length*t1-t2*t2);
		*a = (length*t3 - t2 * t4)/(length*t1-t2*t2);
	}
	else{
		*a = 1;
		*b = 0;
		*error = 0;
		return;
	}
	xbar = t2/length;
	ybar = t4/length;
	
	for( i = 0; i < length; i++) {
		s1 += (x[i]-xbar)*(y[i]-ybar);
		s2 += (x[i]-xbar)*(x[i]-xbar);
		s3 += (y[i]-ybar)*(y[i]-ybar);
	}
	r = s1/(sqrt(s2)*sqrt(s3));
	*error = r * r;
}

float correction_yaw(float tmp_yaw, float t, float gz) {
	static float _History_yaw[HISTORY_YAW_SIZE];
	static float _Hisotry_t[HISTORY_YAW_SIZE];
	static int _History_init_count = HISTORY_YAW_SIZE-1;
	static int _History_error_count = 0;
	static float _Ez = 0;
	float a,b,error;
	float tmpt =0;
	int i = 0;

	if (t == GET_EZ){  // get correction ez
		return _Ez;
	}
	else{
		_Ez = 0;
		//gz = gz * 180.0 / M_PI * LSB;
		if( _History_init_count >= 0) {  
			_History_yaw[HISTORY_YAW_SIZE-1-_History_init_count] = tmp_yaw; // store tmp_yaw
			if (_History_init_count == (HISTORY_YAW_SIZE-1)){
				_Hisotry_t[HISTORY_YAW_SIZE-1-_History_init_count] = t; // store tmp_t
			}
			else{
				_Hisotry_t[HISTORY_YAW_SIZE-1-_History_init_count] = _Hisotry_t[HISTORY_YAW_SIZE-1-_History_init_count-1]+t; //new t
			}
			_History_init_count --;
			return _Ez;
		}		
		else {
			tmpt = _Hisotry_t[0]; // update yaw and t
			for( i = 0; i< HISTORY_YAW_SIZE-1; i++){
				_History_yaw[i] = _History_yaw[i+1];
				_Hisotry_t[i] = _Hisotry_t[i+1] - tmpt;
			}
			_History_yaw[HISTORY_YAW_SIZE-1] = tmp_yaw;
			_Hisotry_t[HISTORY_YAW_SIZE-1] = _Hisotry_t[HISTORY_YAW_SIZE-2] + t;
		
			least_square(_Hisotry_t, _History_yaw, &a, &b, &error); // get a,b,error; a is the gz speed
			if ( error > 0.98 && fabs(a) > 0.001 && fabs(a) < 0.01 ) {  //only shift can increase
				_History_error_count ++ ;
				
				if (_History_error_count > _HISTORY_ERROR_TIME)
				{
					_Ez = -a*M_PI/180.0; //a rotation
					if(fabs(a)>1e-10 )
					{
						Gz_offset += (int16_t) (gz / fabs(gz));
					}
					else
					{
						Gz_offset += 0;
					}
					_History_error_count = 0; //renew
					_History_init_count = HISTORY_YAW_SIZE-1;
				}
			}
			else { // have continue line
				_History_error_count = 0;
				_History_init_count = HISTORY_YAW_SIZE-1;
			}
			return _Ez;
		}
	}
}

void correct_drift()
{
	int32_t i;
	float sum2;
	float gzVal;
	float dift_gz_offset = 0;
	float max_gz = 0;
	float std_gz = 0;
  sum2 = 0;
  for ( i = 1; i < 400; ++i )
  {
		if( fabs(Gz_Buf_132E[i]) > max_gz) {
			 max_gz = fabs(Gz_Buf_132E[i]);
		}
    sum2 += Gz_Buf_132E[i];
  }	
	gzVal = sum2 / 399.0f;
	for ( i = 1; i < 400; ++i ) {
		std_gz += (gzVal - Gz_Buf_132E[i]) * (gzVal - Gz_Buf_132E[i]);
	}
	std_gz = sqrt(std_gz/399.0f);
  if ( gzVal > 0.1 * DSP2RAD || gzVal < -0.1 * DSP2RAD )
  {
		if( max_gz > 2*DSP2RAD ) {
			ram_234 = 0;
			return;
		}
  }
//	if ( std_gz 
	if ( ram_234++ > 2 )
	{
		ram_234 = 0;
		dift_gz_offset = sum_Gz_CE4/399.0f/DSP2RAD;
		Gz_offset +=  (int16_t)dift_gz_offset;	
	}
}

float correction_gz(float t, float gz)
{
	static float _Gz = 0;
	static float _Last_gz = 0;
	static long _Gz_Count = 0;
	static float _Duration_t = 0;
	static float _Mean_Gz = 0;
	static float _Gz_large = 0;
	static int16_t stable = 0;
	static long _Gz_Count_large = 0;
	static float _Duration_t_large = 0;
	static float _Mean_Gz_large = 0;
	static float _Threshold_large = 200*DSP2RAD;
	int16_t dift_gz_offset = 0;
	if ( offset_Cnt_210 )
	{
		sum_Gz_CE4 += gz;
		Gz_Buf_132E[offset_Cnt_210] = Gz_Last_CEC - gz;
		Gz_Last_CEC = gz;
		++offset_Cnt_210;
    if ( offset_Cnt_210 == 400 )
    {
      correct_drift();
      offset_Cnt_210 = 0;
    }		
	}
	else
	{
		Gz_Buf_132E[0] = 0;
		Gz_Last_CEC = gz;
		sum_Gz_CE4 =0;
		offset_Cnt_210++;
	}
	_Last_gz = gz;

}	

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate_no_m
*功　　能:	 更新AHRS 更新四元数, no magnetic
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
void IMU_AHRSupdate_no_m(float gx, float gy, float gz, float ax, float ay, float az) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez,halfT;
  float tempq0,tempq1,tempq2,tempq3;
//	float yaw_angles; //yaw_angle
  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  now = micros();  //读取时间
  if(now<lastUpdate){ //定时器溢出过了。
  halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
  }
  else	{
  halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
  lastUpdate = now;	//更新时间
  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。
  /*
  这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
  根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
  所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
  */
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  
  //现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
  ex = (ay*vz - az*vy) ;
  ey = (az*vx - ax*vz) ;
	//ez = 0*correction_yaw(0,GET_EZ,0);
  //ez = (ax*vy - ay*vx) ;
	//ez = 0;
	ez = correction_gz(halfT*2.0,gz); //gz : rad/s; halfT :time s
  /*
  axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
  axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
  那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
  向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
  这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
  */
if(ex != 0.0f && ey != 0.0f /*&& ez != 0.0f*/){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  //ezInt = ezInt + ez * Ki * halfT;

  // 用叉积误差来做PI修正陀螺零偏
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  //gz = gz + Kp*ez + ezInt;
	gz = gz + Kp * ez;

  }

  // 四元数微分方程
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;

//	yaw_angles = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/M_PI; //yaw degree
	//correction_yaw(yaw_angles,halfT*2.0,gz); //gz : rad/s; yaw_angle: degree, halfT :time
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
float mygetqval[9];	//用于存放传感器转换结果的数组
void IMU_getQ(float * q) {

  IMU_getValues(mygetqval);	 
  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
//IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
 //              mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
	IMU_AHRSupdate_no_m(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2]);
	
  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void AHRS_getYawPitchRoll(float * angles) {
  float q[4]; //　四元数
  volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
	q[0] = 1;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
  IMU_getQ(q); //更新全局四元数
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw 
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}

void _AHRS::Init(void)
{
	hw.addr = sensoraddr;
	mpu_init(0);
	mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);
	mpu_set_sample_rate(1000);
//	mpu_run_self_test(0,0);
	//	HMC5883L_SetUp();
	delay_ms(50);
	//	MPU6050_initialize();
	for(uint8_t i=0;i<10;i++)
	{//
		delay_ms(1);
		getMotion6(0,0,0,0,0,0);
	}
	InitGyro_Offset();

	//陀螺仪偏差
	_exInt = 0.0;
	_eyInt = 0.0;
	_ezInt = 0.0;

	_lastUpdate = micros();//更新时间
	_now = micros();

	_q0=1.0;
	_q1=0;
	_q2=0;
	_q3=0;
}

void _AHRS::Init_Tim(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
	/* TIM2 configuration*/ 
  /* Time Base configuration 基本配置 配置定时器的时基单元*/
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffffffff; //自动重装值         
  TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000-1;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  
	TIM_Cmd(TIM2, ENABLE);   
}

void _AHRS::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) 
{
	hw.addr = sensoraddr;
	if(_isRdy())
	{
		int16_t accel[3],gyro[3];
		MPU_GetAccel(0,accel,0);
		MPU_GetGyro(0,gyro,0);
		newValues(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);
		*ax = FIFO[0][10];
		*ay = FIFO[1][10];
		*az = FIFO[2][10];
		*gx = FIFO[3][10]-Gx_offset;
		*gy = FIFO[4][10]-Gy_offset;
		*gz = FIFO[5][10]-Gz_offset;
	} else {
		*ax = FIFO[0][10];//=MPU6050_FIFO[0][10];
		*ay = FIFO[1][10];//=MPU6050_FIFO[1][10];
		*az = FIFO[2][10];//=MPU6050_FIFO[2][10];
		*gx = FIFO[3][10]-Gx_offset;//=MPU6050_FIFO[3][10];
		*gy = FIFO[4][10]-Gy_offset;//=MPU6050_FIFO[4][10];
		*gz = FIFO[5][10]-Gz_offset;//=MPU6050_FIFO[5][10];
	}
}

uint8_t _AHRS::_isRdy()
{
	if(sensoraddr==0)
	{
		if(GPIO_ReadInputDataBit(IMU0_IT_PORT,IMU0_IT_Pinx)==Bit_RESET)
		{
			return 1;
		}else{
			return 0;
		}
	}
	return 0;
}

void _AHRS::newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO 操作
		FIFO[0][i-1]=FIFO[0][i];
		FIFO[1][i-1]=FIFO[1][i];
		FIFO[2][i-1]=FIFO[2][i];
		FIFO[3][i-1]=FIFO[3][i];
		FIFO[4][i-1]=FIFO[4][i];
		FIFO[5][i-1]=FIFO[5][i];
	}
		FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
		FIFO[1][9]=ay;
		FIFO[2][9]=az;
		FIFO[3][9]=gx;
		FIFO[4][9]=gy;
		FIFO[5][9]=gz;

	sum=0;
	for(i=0;i<10;i++){														//求当前数组的合，再取平均值
		 sum+=FIFO[0][i];
	}
	FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=FIFO[1][i];
	}
	FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=FIFO[2][i];
	}
	FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=FIFO[3][i];
	}
	FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=FIFO[4][i];
	}
	FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=FIFO[5][i];
	}
	FIFO[5][10]=sum/10;
}

void _AHRS::InitGyro_Offset()
{
	unsigned char i;
	int16_t temp[6];
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	int32_t	tempax=0,tempay=0,tempaz=0;
	_Gx_offset=0;
	_Gy_offset=0;
	_Gz_offset=0;
	for(i=0;i<50;i++){
  		delay_ms(1);
  		getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
	}
 	for(i=0;i<100;i++){
		delay_ms(1);
		getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
	}
	_Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
	_Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
	_Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];
}

void _AHRS::getYawPitchRoll(float * angles)
{
	float q[4]; //　四元数
  volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
	q[0] = 1;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
  getQ(q); //更新全局四元数
  
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}

void _AHRS::getQ(float * q)
{
	float mygetqval[9];	//用于存放传感器转换结果的数组
	getValues(mygetqval);	 
  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
	//IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
 //              mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
	update_no_m(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
	mygetqval[0], mygetqval[1], mygetqval[2]);
	
  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void _AHRS::getValues(float * val)
{
	int16_t accgyroval[6];
	int i;
	//读取加速度和陀螺仪的当前ADC
	getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	for(i = 0; i<6; i++) {
		if(i < 3) {
			val[i] = (float) accgyroval[i];
		}
		else {
			if ( accgyroval[i] <3 && accgyroval[i]> -3 ) {
				val[i] = 0;
			} else {
				val[i] = ((float) accgyroval[i]) / LSB; //转成度每秒
			}
	//这里已经将量程改成了 360度每秒  LSB 对应 1度每秒
		}
	}
}

#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
void _AHRS::update_no_m(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez,halfT;
  float tempq0,tempq1,tempq2,tempq3;
//	float yaw_angles; //yaw_angle
  // 先把这些用得到的值算好
  float q0q0 = _q0*_q0;
  float q0q1 = _q0*_q1;
  float q0q2 = _q0*_q2;
  float q0q3 = _q0*_q3;
  float q1q1 = _q1*_q1;
  float q1q2 = _q1*_q2;
  float q1q3 = _q1*_q3;
  float q2q2 = _q2*_q2;   
  float q2q3 = _q2*_q3;
  float q3q3 = _q3*_q3;          
  
  _now = micros();  //读取时间
  if(_now<_lastUpdate){ //定时器溢出过了。
  halfT =  ((float)(now + (0xffff- _lastUpdate)) / 2000000.0f);
  }
  else	{
  halfT =  ((float)(now - _lastUpdate) / 2000000.0f);
  }
  _lastUpdate = _now;	//更新时间
  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。
  /*
  这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
  根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
  所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
  */
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  
  //现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
  ex = (ay*vz - az*vy) ;		//To do
  ey = (az*vx - ax*vz) ;
	//ez = 0*correction_yaw(0,GET_EZ,0);
  //ez = (ax*vy - ay*vx) ;
	//ez = 0;
	ez = correction_gz(halfT*2.0,gz,ax,ay,az); //gz : rad/s; halfT :time s
  /*
  axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
  axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
  那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
  向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
  这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
  */
	if(ex != 0.0f && ey != 0.0f /*&& ez != 0.0f*/){
		_exInt = _exInt + ex * Ki * halfT;
		_eyInt = _eyInt + ey * Ki * halfT;	
		//ezInt = ezInt + ez * Ki * halfT;

		// 用叉积误差来做PI修正陀螺零偏
		gx = gx + Kp*ex + _exInt;
		gy = gy + Kp*ey + _eyInt;
		//gz = gz + Kp*ez + ezInt;
		gz = gz + Kp * ez;

	}

  // 四元数微分方程
  tempq0 = _q0 + (-_q1*gx - _q2*gy - _q3*gz)*halfT;
  tempq1 = _q1 + (_q0*gx + _q2*gz - _q3*gy)*halfT;
  tempq2 = _q2 + (_q0*gy - _q1*gz + _q3*gx)*halfT;
  tempq3 = _q3 + (_q0*gz + _q1*gy - _q2*gx)*halfT;  
  
  // 四元数规范化
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  _q0 = tempq0 * norm;
  _q1 = tempq1 * norm;
  _q2 = tempq2 * norm;
  _q3 = tempq3 * norm;

//	yaw_angles = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/M_PI; //yaw degree
	//correction_yaw(yaw_angles,halfT*2.0,gz); //gz : rad/s; yaw_angle: degree, halfT :time
}

float _AHRS::correction_gz(float t, float gz, float ax, float ay, float az)
{
	static float _Gz = 0;
	static float _Last_gz = 0;
	static long _Gz_Count = 0;
	static float _Duration_t = 0;
	static float _Mean_Gz = 0;
	static float _Gz_large = 0;
	static int16_t stable = 0;
	static long _Gz_Count_large = 0;
	static float _Duration_t_large = 0;
	static float _Mean_Gz_large = 0;
	static float _Threshold_large = 200*DSP2RAD;
	int16_t dift_gz_offset = 0;
	if ( _offset_Cnt_210 )
	{
		sum_Gz_CE4 += gz;
		_Gz_Buf_132E[offset_Cnt_210] = _Gz_Last_CEC - gz;
		_Ax_Buf_1662[offset_Cnt_210] = _Ax_Last_165C - ax;
		_Ay_Buf_1982[offset_Cnt_210] = _Ay_Last_165E - ay;
		_Az_Buf_1CA2[offset_Cnt_210] = _Az_Last_1660 - az;
		
		_Gz_Last_CEC = gz;
		++_offset_Cnt_210;
    if ( _offset_Cnt_210 == 400 )
    {
      correct_drift();
      _offset_Cnt_210 = 0;
    }		
	}
	else
	{
		_Gz_Buf_132E[0] = 0;
		_Gz_Last_CEC = gz;
		_sum_Gz_CE4 =0;
		_Ax_Buf_1662[0] = 0;
		_Ay_Buf_1982[0] = 0;
		_Az_Buf_1CA2[0] = 0;
		_Ax_Last_165C = ax;
		_Ay_Last_165E = ay;
		_Az_Last_1660 = az;
		_offset_Cnt_210 ++;
	}
	_Last_gz = gz;
}
