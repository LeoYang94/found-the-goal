#include "Chassis_Control.h"
#include "Prepare_data.h"
#include "Holder.h"
#include "Wheel_Speed.h"
#include "Remote.h"
#include "USART3.h"
#include "OtherConfig.h"
#include "All_Config.h"
#include "my_func.h"


#define SPEED_MAX 1500 //只修改了一处，在84行
#define SPEED_K 0.025f //


#define ENABLE 1
#define DISABLE 0


int16_t	chassis_f0 = 0;
int16_t chassis_r0 = 0;
int16_t chassis_y0 = 0; 



/*
		测试突然加速处理
*/
//-------------------
int16_t last_chassis_speed[3],speed_save[3];
//-------------------
int16_t	f0 = 0;
int16_t r0 = 0;
int16_t y0 = 0; 

struct Chassis_Info Chassis_Control_Info;
//2600
struct PID_PARA Chassis_para = 
{
	1000,0,0,
	0,0,0
};

float AmplitudeCheck(float Angle , float limit)
{
	if(Angle>limit) 
		return limit;
	else if(Angle<-limit)
		return -limit;
	else
		return Angle;
		
}

void Chessis_Info_Init(void)
{
	Chassis_Control_Info.Chassis_angle_Target = 0;
	Chassis_Control_Info.Anaconda_flag = 0;
}


int8_t Chassis_Control(void)
{
	
	
	float delta;
	
	float Scale[5] = { 0 , 5 , 10 , 15 , 20  };
	
	float k[5] = { 1.0f , 0.8f , 0.7f , 0.6f , 0.5f };
	//1.angle
	
	

	
	//2.pid
	delta = 0 - Chassis_Control_Info.Chassis_angle ;
	//delta = 0 - 0 ;
	delta = Amplitude_Limiting(ENABLE , delta ,30 , -30 );
	
	delta = Subsection_PID( ENABLE , delta , 5 , k , Scale );
	
	Chassis_Control_Info.Chassis_out = Chassis_para.shell_P * 0.1f * delta ;
	

	
	return 1;
	
}


float	direction[4];//电机1，电机2，电机3，电机4

int8_t Chassis_Remote_Dispack(uint8_t flag)
{
	int16_t x=100.0f,y=20.0f;//发送数据电流值范围 -5000~+5000, 10000/1320=8	x是速度
	
	uint8_t i;	
	

	
	if (IsComputerControl == 1)//键鼠控制 
	{
		;
	}

	else if( IsComputerControl == 0 )  //摇杆控制
	{
		chassis_f0 = RC_Ctl.rc.ch1-1024 ;
		chassis_r0 = RC_Ctl.rc.ch0-1024;
	}

	
	
	chassis_y0 = Chassis_Control_Info.Chassis_out;
	
	
	f0 = chassis_f0;
	r0 = chassis_r0 * 1.2f;
	y0 = chassis_y0;
	
	//y0 = 0;//**********************************************我被改了********************************************************************8



	//remote_data_deal(1);//防止速度变化过大导致的功率突然超
	
	direction[0]=100*(-f0+r0)+y*y0;
	direction[1]=100*(f0+r0)+y*y0;
	direction[2]=100*(f0-r0)+y*y0;
	direction[3]=100*(-f0-r0)+y*y0;
	
	if( flag != 0 )
	{
		for(i=0;i<4;i++)//限幅输出
		{
			Four_Wheel_Info.Target_speed[i] = AmplitudeCheck( Four_Wheel_Info.speed_K * direction[i] , Four_Wheel_Info.speed_limit);	
			Four_Wheel_Info.speed_diff_cnt = 0;
			Four_Wheel_Info.Target_speed_Old[i] = Four_Wheel_Info.Target_speed_New[i];//保存旧值，更新新值
			Four_Wheel_Info.Target_speed_New[i] = AmplitudeCheck( Four_Wheel_Info.speed_K * direction[i] , Four_Wheel_Info.speed_limit);	
		}	
	}		   
	else
	{
		for(i=0;i<4;i++)//限幅输出
		{
			Four_Wheel_Info.Target_speed[i] = 0;	
		}	
	}	
	
	
	return 1;
	
	
}


void Anaconda(uint8_t flag)
{
	uint8_t enable = 0;
	
	if( flag == 0 )
	{
		return;
	}		
	
	
	/*判断使能条件*/
	
	Lets_Rock(1);
	
}

void Lets_Rock(uint8_t flag)
{
	static uint32_t cnt = 0;
	static int16_t time_ms = 400;
	
	static float angle = 40.0f;
	
	static float k ;
		
	k = angle / time_ms;
	
	if( flag == 0 )
	{
		return;
	}		
	
	
	
	cnt ++;
	
	if( cnt <= time_ms )
	{
		Chassis_Control_Info.Chassis_angle_Target = -( cnt * k );
	}
	
	else if( cnt <= (2 * time_ms) )
	{
		Chassis_Control_Info.Chassis_angle_Target = -( (2 * time_ms - cnt) * k );
	}
	
		
	else if( cnt <= (3 * time_ms))
	{
		Chassis_Control_Info.Chassis_angle_Target = ( (cnt - ( 2 * time_ms )) * k );
	}
	
	else if( cnt <= (4 * time_ms))
	{
		Chassis_Control_Info.Chassis_angle_Target = ( (4 * time_ms - cnt) * k );
	}
	else
	{
		cnt = 0;
		Chassis_Control_Info.Chassis_angle_Target = 0;
	}
	
}



void remote_data_deal(uint8_t flag)
{
	uint8_t i;
	int16_t current_chassis_speed[3],div_four[3];
	if(flag)
	{
			current_chassis_speed[1] = f0;
			current_chassis_speed[0] = r0;
			current_chassis_speed[2] = y0;

			for(i=0;i<3;i++)
			{
				div_four[i] = current_chassis_speed[i] - last_chassis_speed[i];

				if(abs(div_four[i] <= 50))
				{
					speed_save[i] = current_chassis_speed[i];
					
				}else if(div_four[i] > 0)
				{
					speed_save[i] = last_chassis_speed[i]+20;
					
				}else if(div_four[i] < 0)
				{
					speed_save[i] = last_chassis_speed[i]-20;
				}
				
			}
			last_chassis_speed[0] = speed_save[0];
			last_chassis_speed[1] = speed_save[1];
			last_chassis_speed[2] = speed_save[2];
			
			r0 = speed_save[0];
			f0 = speed_save[1];
			y0 = speed_save[2];
			
	}
}
