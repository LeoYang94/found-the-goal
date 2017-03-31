#include "Prepare_data.h"
#include "6050.h"
#include "Holder.h"
#include "USART3.h"
#include "OtherConfig.h"
#include "All_Config.h"
#include "Imu.h"
#include "clock.h"
#include "Chassis_Control.h"

#define KALMAN_Q        0.02
#define KALMAN_R        6.0000



static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);

uint8_t mpu6050_error_flag = 0;

float Gyro_File_Buf[3][GYRO_FILTER_NUM];


/*¸üÐÂË«Öá ½ÇËÙ¶È ½Ç¶È can±àÂëÆ÷ÐÅÏ¢*/
void Attitude_update(void)
{
	if ( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6025) 
	{
		Pitch_Hold_Info.angle_speed = sensor.gyro.radian.x / Gyro_Gr;//½Ç¶ÈÎ¢·ÖµÃµ½¼ÓËÙ¶È
		Pitch_Hold_Info.angle = angle.roll ;
		Pitch_Hold_Info.can_angle = K_CODE_2_ANGLE * (Pitch_Hold_Info.can_angle_raw -LaserAndPrep[TANK_SERIAL_NUMBER-1][3]) ;	
	}
	
	else if( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6623) 
	{
		Pitch_Hold_Info.angle_speed = sensor.gyro.radian.x / Gyro_Gr;
		Pitch_Hold_Info.angle = angle.roll  ;
		
		if( TANK_SERIAL_NUMBER == 1 )
		{
			if(Pitch_Hold_Info.can_angle_raw<0x104)
			  {Pitch_Hold_Info.can_angle_new= Pitch_Hold_Info.can_angle_raw +  0x1fff;}//pitchÖá¹ýÁãµã
    else
	    Pitch_Hold_Info.can_angle_new=Pitch_Hold_Info.can_angle_raw;
		}
		
		Pitch_Hold_Info.can_angle = K_CODE_2_ANGLE * (Pitch_Hold_Info.can_angle_raw -LaserAndPrep[TANK_SERIAL_NUMBER-1][3]) ;	
//		
		
	}
	
	
	yaw_Hold_Info.angle_speed =  sensor.gyro.radian.z / Gyro_Gr;
	if(  abs( yaw_Hold_Info.angle_speed ) < GYRO_GAP)
	{
		yaw_Hold_Info.angle_speed = 0;
	}
	
	yaw_Hold_Info.angle += ( yaw_Hold_Info.angle_speed * (K_ANGLESPEED_2_ANGLE));
	
	       
	
	if(yaw_Hold_Info.can_angle_raw < 3000)			  
    {
     yaw_Hold_Info.can_angle_new = yaw_Hold_Info.can_angle_raw + 8192;
    }	
//yawÖá¹ýÁãµã
	 else
		{
		 yaw_Hold_Info.can_angle_new=yaw_Hold_Info.can_angle_raw;
		}
	
	yaw_Hold_Info.can_angle = K_CODE_2_ANGLE * (yaw_Hold_Info.can_angle_new - LaserAndPrep[TANK_SERIAL_NUMBER-1][2]);
	
	
			
	Chassis_Control_Info.Chassis_angle = yaw_Hold_Info.can_angle;
	
	
	
}
	

//void Prepare_data(void)
//{
//	static int16_t gyro_filter_cnt = 0;
//	
//	uint8_t i ;
//	uint8_t Addr1;
//	
//	float sum_pitch , sum_yaw;
//	
//	MPU6050_Read();
//	
//	
//	//1. get angle speed
//	Pitch_Hold_Info.angle_speed_16 =  ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - sensor.gyro.quiet.x;// pitch angle speed
//	Pitch_Hold_Info.angle_speed_buff[gyro_filter_cnt] = Pitch_Hold_Info.angle_speed_16 ;
//	Pitch_Hold_Info.can_angle = K_CODE_2_ANGLE * (Pitch_Hold_Info.can_angle_raw -LaserAndPrep[TANK_SERIAL_NUMBER-1][3]) ;
//	
//	yaw_Hold_Info.angle_speed_16 =  - (((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - sensor.gyro.quiet.z);// pitch angle speed
//	yaw_Hold_Info.angle_speed_buff[gyro_filter_cnt] = -yaw_Hold_Info.angle_speed_16 ;
//	
//	
//	
//	
//	
//	//slide filter
//	sum_pitch = 0;
//	sum_yaw = 0;
//	
//	for( i = 0;i < GYRO_FILTER_NUM;i++ )
//	{
//		sum_pitch += Pitch_Hold_Info.angle_speed_buff[i];
//		sum_yaw += yaw_Hold_Info.angle_speed_buff[i];
//		
//		
//	}
//	Pitch_Hold_Info.angle_speed = sum_pitch / GYRO_FILTER_NUM;
//	yaw_Hold_Info.angle_speed = sum_yaw / GYRO_FILTER_NUM;
//	
//	if(  abs( Pitch_Hold_Info.angle_speed ) < GYRO_GAP)
//	{
//		Pitch_Hold_Info.angle_speed = 0;
//	}
//	
//	if(  abs( yaw_Hold_Info.angle_speed ) < GYRO_GAP)
//	{
//		yaw_Hold_Info.angle_speed = 0;
//	}
//	
//	
//	
//	gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
//	
//	//2. get angle
//	//2.1 PITCH ANGLE
//	
//	
//	Pitch_Hold_Info.can_angle = K_CODE_2_ANGLE * (Pitch_Hold_Info.can_angle_raw -LaserAndPrep[TANK_SERIAL_NUMBER-1][3]) ;
//	Pitch_Hold_Info.angle = Pitch_Hold_Info.can_angle;
//	
//	//2.2 YAW ANGLE
//	yaw_Hold_Info.angle += ( yaw_Hold_Info.angle_speed * K_ANGLESPEED_2_ANGLE  );
//	
//	if(  TANK_SERIAL_NUMBER == 3)
//	{
//		if(  yaw_Hold_Info.can_angle_raw < 2000)
//		{
//			yaw_Hold_Info.can_angle_raw = yaw_Hold_Info.can_angle_raw + 8192;
//		}
//	}
//	
//	
//	yaw_Hold_Info.can_angle = K_CODE_2_ANGLE * (yaw_Hold_Info.can_angle_raw - LaserAndPrep[TANK_SERIAL_NUMBER-1][2]);
//}


void PrepareForIMU(uint8_t flag)
{
	float sumx,sumy,sumz,sum_yaw;
		
	static uint8_t gyro_filter_cnt = 0;
		
	static uint8_t z_num = 0;
		
	int i =0;
		
	if(flag == 0) 
			return;
	else
		{
		
		MPU6050_Read();
		sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;//Ç°Èý¸öÎª¼ÓËÙ¶È¼Æ
		sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
		sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);
		
		sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9])- sensor.gyro.quiet.x;//ÍÓÂÝÒÇÈ¡ÏûÁãµãÆ«ÒÆ
		sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11])- sensor.gyro.quiet.y;
		sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13])- sensor.gyro.quiet.z;
		
		Gyro_File_Buf[0][gyro_filter_cnt] = sensor.gyro.origin.x ;
		Gyro_File_Buf[1][gyro_filter_cnt] = sensor.gyro.origin.y ;
		Gyro_File_Buf[2][gyro_filter_cnt] = sensor.gyro.origin.z ;
			
			sumx = 0;
			sumy = 0;
			sumz = 0;
		for(i=0;i<GYRO_FILTER_NUM;i++)//Ê®´ÎµÄºÍÏà¼Ó
		{
			sumx += Gyro_File_Buf[0][i];
			sumy += Gyro_File_Buf[1][i];
			sumz += Gyro_File_Buf[2][i];
		}

		
		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
		
		sensor.gyro.radian.x  = sumx / (float)GYRO_FILTER_NUM * Gyro_Gr;//»»Ëã³É»¡¶ÈÖµ
		sensor.gyro.radian.y  = sumy / (float)GYRO_FILTER_NUM * Gyro_Gr;
		sensor.gyro.radian.z  = sumz / (float)GYRO_FILTER_NUM * Gyro_Gr;
		
		
		sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  // ACC XÖá¿¨¶ûÂüÂË²¨
		sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  // ACC YÖá¿¨¶ûÂüÂË²¨
		sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  // ACC ZÖá¿¨¶ûÂüÂË²
	
		

		
	}
}


static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=ÔëÉù
   kg=p_mid/(p_mid+R); //kgÎªkalman filter£¬RÎªÔëÉù
   x_now=x_mid+kg*(ResrcData-x_mid);//¹À¼Æ³öµÄ×îÓÅÖµ
                
   p_now=(1-kg)*p_mid;//×îÓÅÖµ¶ÔÓ¦µÄcovariance       
   p_last = p_now; //¸üÐÂcovarianceÖµ
   x_last = x_now; //¸üÐÂÏµÍ³×´Ì¬Öµ
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=ÔëÉù
   kg=p_mid/(p_mid+R); //kgÎªkalman filter£¬RÎªÔëÉù
   x_now=x_mid+kg*(ResrcData-x_mid);//¹À¼Æ³öµÄ×îÓÅÖµ
                
   p_now=(1-kg)*p_mid;//×îÓÅÖµ¶ÔÓ¦µÄcovariance       
   p_last = p_now; //¸üÐÂcovarianceÖµ
   x_last = x_now; //¸üÐÂÏµÍ³×´Ì¬Öµ
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=ÔëÉù
   kg=p_mid/(p_mid+R); //kgÎªkalman filter£¬RÎªÔëÉù
   x_now=x_mid+kg*(ResrcData-x_mid);//¹À¼Æ³öµÄ×îÓÅÖµ
                
   p_now=(1-kg)*p_mid;//×îÓÅÖµ¶ÔÓ¦µÄcovariance       
   p_last = p_now; //¸üÐÂcovarianceÖµ
   x_last = x_now; //¸üÐÂÏµÍ³×´Ì¬Öµ
   return x_now;                
}
