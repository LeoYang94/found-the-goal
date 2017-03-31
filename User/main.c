#include "stm32f4xx.h"
#include "sys.h" 
#include "delay.h"
#include "led.h" 
#include "USART3.h"
#include "CAN2.h"
#include "I2C.h"
#include "6050.h"
#include "clock.h"
#include "Wheel_Speed.h"
#include "Shoot.h"
#include "Remote.h"
#include "Holder.h"
#include "Load_motor.h"
#include "Motor_out.h"
#include "PowerLimit.h"
#include "referee.h"
#include "BKP_DATA.h"
#include "MyWDG.h"
#include "OtherConfig.h"
#include "All_Config.h"
#include "imu.h"
#include "Chassis_Control.h"
#include "Hit.h"
#include "I2C.h"
//测试BKP
uint32_t ORI[3]={1000,2000,3000};
uint32_t BKP_DATA[3]; 
//***************************************


/*
2016.7.11

实地修改

连续按住右键1.5s，无线开启摩擦轮

继续连续按住右键1.5s，关闭摩擦轮
*/

/*
2016.8.1
姿态结算
*/

/*

2016.8.6

修改定时器

兼容弹仓定时器


*/

/*
2016.8.7

优化底盘电机输出,增速

*/

/*
2016.8.10

整合 妙算步兵车和普通步兵车
妙算步兵车 1号

*/


/*
2016.8.16
优化电机输出
平滑遥控器给定


//注释掉所有关于射击和拨单电机配置，MAIN和CLOCK中

//注释掉所有关于底盘的东西
//CLOCK也关上，现在相当于只用can

*/
/*
20177.3.30 加入云台自动识别装甲程序
核心
*/
uint32_t p1 , p2 , p3;

int16_t speed_out = 800;
extern float send_anto_data[4];

void my_delay(int32_t num)
{
	while(num >0)
	{
		num--;
	}
}
int main(void)
{     

	float speed_main[4] = {1200,1200,1200,1200};
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;
	
	RC_Ctl.rc.ChangeFlag = 0;
	IsComputerControl = 0;//手柄模式	
	Holder_update_para();//云台参数初始化
	delay_init(168);
	if( LaserAndPrep[TANK_SERIAL_NUMBER-1][10] == 1 )//支持妙算
	{
		Hit_flag_init();
		CalculateKey();
	}	
	
	//Shoot_PWM_Configuration();
	//Shoot_Fire(1000);
	LED_Init();	
	Referee_Configuration();
	//Load_Motor_PWM_Configuration();//拨弹电机
	//Shoot_Info_Init();//射击参数初始化
	CAN2_Configuration();
	Holder_Motor_output(0);//输出为0，降低干扰
	delay_ms(1000);

	//Encoder_Configuration();//拨弹电机编码器
	
	//Bullet_Cap_Conf();//弹仓舵机初始化
	//Bullet_Cap_Open();
	Remote_Config();//远程遥控初始化	
	I2C_INIT();
	InitMPU6050();//6050初始化
	Gyro_OFFEST();
	Sys_Configuration();//1ms 无中断
	delay_ms(1000);
	Laser_Init();//激光初始化
	Clock_Int_Init();//1ms
	
	
	while(1)
	{
		PrepareForIMU(1);
		IMUupdate(sensor.gyro.radian.x,
							sensor.gyro.radian.y,
						  sensor.gyro.radian.z,
							sensor.acc.averag.x,
							sensor.acc.averag.y,
							sensor.acc.averag.z);
							
		Attitude_update();
		/***********调试************/
    //send_odm_msg(send_anto_data);
		#if 1
		send_anto_data[0] = Pitch_Hold_Info.angle_target;
			
		send_anto_data[1] = Pitch_Hold_Info.angle;
		
		send_anto_data[2] = yaw_Hold_Info.angle_target;
		
		send_anto_data[3] = yaw_Hold_Info.angle;
		
		
		#endif
		//Debug_uart3_send(Send_data);
		
	}
	
	
}




