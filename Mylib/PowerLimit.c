#include "PowerLimit.h"
#include "referee.h"
#include "Wheel_Speed.h"
#include "Remote.h"
#include "Holder.h"
struct S_POWER_INFO S_Power_info;

void Power_init(void)
{
	
	S_Power_info.Over_Joule_flag = 0;//过载标志位
	
	S_Power_info.Joule_limit_min = 10;//10J
	S_Power_info.Joule_limit_max = 60;//60J
	S_Power_info.Joule_left = S_Power_info.Joule_limit_max;
	S_Power_info.Joule_limit_recover = 30;
	
	S_Power_info.Power_limit = 80;//80W
	
}

void Power_info_update(void)
{
	//1.读取当前功率
	S_Power_info.Power_real = Judgment_01_data.power_W;
	
	//2.
	//S_Power_info.Joule_left += (( S_Power_info.Power_limit - S_Power_info.Power_real )*0.002f);
	S_Power_info.Joule_left += ((int)((( S_Power_info.Power_limit - S_Power_info.Power_real )*4000.0f)+0.5)/1000000.0f);
	
	S_Power_info.Joule_left = Range( 0 , 60 , S_Power_info.Joule_left );
	
	if( S_Power_info.Joule_left == 0 )//过载
	{
		S_Power_info.Over_Joule_flag = 1;
	}
	
	if( S_Power_info.Over_Joule_flag  == 1 )//若已经过载，等待恢复
	{
		if( S_Power_info.Joule_left >= S_Power_info.Joule_limit_recover )
		{
			S_Power_info.Over_Joule_flag = 0;
		}
	}
	
	
}

float Range(float low , float high , float var)
{
	if( var <= low )
		return low;
	
	if( var >= high )
		return high;
	
	return var;
}

void Update_PowerInformation(uint8_t flag)
{
	if(flag == 1)
	{
		
		if( (RC_Ctl.rc.s1 == 1) && (S_Power_info.Over_Joule_flag == 0) )
		{
			Four_Wheel_Info.speed_limit = 1600;
			Four_Wheel_Info.out_limit = 15000;
			Four_Wheel_Info.speed_K = 0.025f;
			Yaw_para.shell_P = 250;
			Yaw_para.shell_D = 50;
			Yaw_para.core_P = 550;
		}
		else if(RC_Ctl.rc.s1 == 3)
		{
			Four_Wheel_Info.speed_limit = 1800;
			Four_Wheel_Info.out_limit = 20000;
			Four_Wheel_Info.speed_K = 0.038f;
			Yaw_para.shell_P = 300;
			Yaw_para.shell_D = 70;
			Yaw_para.core_P = 650;
		}
	}
}
