#include "Hit.h"
#include "sys.h"
#include "Remote.h"
#include "Chassis_Control.h"
#include "Prepare_data.h"
#include "Holder.h"
#include "Hit_calculation.h"
#include "Motor_out.h"
#include "Wheel_Speed.h"
#include "Load_motor.h"
#include "clock.h"

#define ENABLE 1
#define DISABLE 0


struct RC_Ctl_key RC_Key;

uint8_t HitMode=0,TestSDSU=0,FetchMode=0,FetchFinish=0,CanChangeFlag=0,CanFetchFlag=0;
uint32_t LastPoint;
uint8_t Fetch_cnt=0,CanShootFlag=0;

uint8_t Hit_rev;//妙算接收缓冲区


static void SelectMode(uint8_t mode);
static void VisonControl(float target,uint8_t flag);
static void HitSDSU(uint8_t point,uint8_t mode);

static void HitSDSU_Pro(uint8_t flag);
static void HitSDSU_Key_scan(uint8_t flag);


float TargetTable[9][2]=
{
{9.18,-9.05396},//1号
{0,-9.05396},
{-8.5,-9.05396},
{9.18,-4.8848},
{0,-4.8848},
{-8.2,-4.8848},
{9.3075,-0.70244},
{0,-0.70244},
{-8.67,-0.70244}
};




void CalculateKey(void)
{
	RC_Key.key_A  = 0x0004;
	RC_Key.key_W  = 0x0001;
	RC_Key.key_S  = 0x0002;
	RC_Key.key_Q  = 0x0040;//KEYI
	RC_Key.key_E  = 0x0080;//KEYI
	RC_Key.key_R  = 0x0100;
	RC_Key.key_D  = 0x0008;
	RC_Key.key_F  = 0x0200;
	RC_Key.key_G  = 0x0400;
	RC_Key.key_Z  = 0x0800;
	RC_Key.key_X  = 0x1000;
	RC_Key.key_C  = 0x2000;
	RC_Key.key_V  = 0x4000;
	RC_Key.key_B  = 0x8000;
	
	RC_Key.key_Shift  = 0x0010;//KEYI
	RC_Key.key_Ctrl  = 0x0020;//KEYI
	
}


void Hit_flag_init(void)
{
	HitMode =0;//九宫格模式
	FetchMode = 0;//采集模式
}


void Hit_Task(uint8_t flag)
{
	if( flag == 0 )
	{
		return;
	}
	HitSDSU_Key_scan(1);
	
	HitSDSU_Pro(1);
}


static void HitSDSU_Key_scan(uint8_t flag)
{
	if( flag == 0 )
	{
		return;
	}
	
	//采集模式按键处理
		if(CanChangeFlag)
		{
			if(RC_Ctl.key.v == RC_Key.key_E)
			{
				CanFetchFlag = 1;
				CanChangeFlag = 0;
			}
		}
		
		if((FetchMode == 1)&&(RC_Ctl.key.v == 0))
		{
			CanChangeFlag = 1;
		}
//********************************************		
		
		//采集模式 Q开启 Ctrl+Q关闭
	if(RC_Ctl.key.v == RC_Key.key_Q)
	{
		FetchMode = 1;
		HitMode = 0;
		TestSDSU = 0;
	}
	if((RC_Ctl.key.v & RC_Key.key_Ctrl)&&(RC_Ctl.key.v & RC_Key.key_Q))
	{
		FetchMode = 0;
		HitMode = 0;
		TestSDSU = 0;
		Fetch_cnt =0;
	}
		
}

static void HitSDSU_Pro(uint8_t flag)
{
	static uint32_t iiiwww = 0;
	static uint32_t jjjjwww = 0;
	
	if( flag == 0 )
	{
		return;
	}
	
	if(!FetchMode)
	{
		if(!HitMode)
		{	
			//正常运行模式
			PAout(8) = 1;		
			
			Holder_Pitch_Control(ENABLE  , &Pitch_para,Pitch_Hold_Info.angle_target, Pitch_Hold_Info.angle);//pitch
			Holder_Yaw_Control(ENABLE, &Yaw_para ,  yaw_Hold_Info.angle_target , yaw_Hold_Info.angle);//YAW
			
			Holder_Motor_output(HOLDER_OUT);
			
			Chassis_Control();
			Chassis_Remote_Dispack(1);
			
			Wheel_Speed_control(WHEEL_OUT);	
		}
		else
		{			
				
				PAout(8) = 0;//关闭激光笔
				
				HitSDSU(Hit_rev-0x30,0);		
			
				#if 0
				iiiwww++;
				if(iiiwww > 500)
				{
					jjjjwww++;
					iiiwww = 1;
					jjjjwww = jjjjwww%10;
					if(jjjjwww == 0)
					{
						jjjjwww = 1;
					}
				}
				HitSDSU(jjjjwww,0);		
				#endif							
			}
	}
	else
	{
		//采集模式
		
		PAout(8) = 1;					
		
		Holder_Pitch_Control(ENABLE  , &Pitch_para,Pitch_Hold_Info.angle_target, Pitch_Hold_Info.angle);//pitch
		Holder_Yaw_Control(ENABLE, &Yaw_para ,  yaw_Hold_Info.angle_target , yaw_Hold_Info.angle);//YAW
			
		Holder_Motor_output(1);
		
		Wheel_Speed_control(0);
		
		SelectMode(3);				
	}
}

static void HitSDSU(uint8_t point,uint8_t mode)
{
	float pitch_temp;
	
	uint32_t TargetPoint = point;

	Holder_Pitch_Control(ENABLE ,&Pitch_para ,TargetTable[TargetPoint-1][1], Pitch_Hold_Info.angle);//pitch
	Holder_Yaw_Control(ENABLE, &Yaw_para ,  TargetTable[TargetPoint-1][0] , yaw_Hold_Info.can_angle);//YAW
			
	
	Holder_Motor_output(1);
	
  if(LastPoint == TargetPoint)
  {

  }else
  {
    Load_Motor_position_plus(105);
  }
  
  LastPoint = TargetPoint;
}



static void SelectMode(uint8_t mode)
{
	float Pix[9][2];
	float ReturnRes[2];
	static uint8_t j=1;
//	float dev_Yaw,dev_Pitch;
	switch(mode){
	case 1:{break;}
	case 2:{
				Pix[0][0] = yaw_Hold_Info.can_angle;
				Pix[0][1] = Pitch_Hold_Info.angle;

				
				Pix[8][0] = yaw_Hold_Info.can_angle;
				Pix[8][1] = Pitch_Hold_Info.angle;
				
				TwoPointCal(abs(Pix[0][0]),abs(Pix[8][0]),ReturnRes);
				TargetTable[0][0] = Pix[0][0];
				TargetTable[1][0] = TargetTable[0][0]+ReturnRes[0];
				TargetTable[2][0] = TargetTable[1][0]+ReturnRes[1];
				TargetTable[0][1] = Pix[0][1];
				TargetTable[1][1] = Pix[0][1];
				TargetTable[2][1] = Pix[0][1];
				
				TwoPointCal(abs(Pix[0][1]),abs(Pix[8][1]),ReturnRes);
								
	break;}
	case 3:{
				if(CanFetchFlag)
				{	
					if(j==1)
					{
						TargetTable[0][0] = yaw_Hold_Info.can_angle;
						TargetTable[0][1] = Pitch_Hold_Info.angle;					
					}else if(j==2)
					{	
						TargetTable[4][0] = yaw_Hold_Info.can_angle;
						TargetTable[4][1] = Pitch_Hold_Info.angle;
					}else
					{
						TargetTable[8][0] = yaw_Hold_Info.can_angle;
						TargetTable[8][1] = Pitch_Hold_Info.angle;
						FetchFinish = 1;
					}
					j++;
					CanFetchFlag = 0;
				}
				if(FetchFinish)
				{			
					TargetTable[1][0] = TargetTable[4][0];
					TargetTable[1][1] = TargetTable[0][1];
					
					TargetTable[2][0] = TargetTable[8][0];
					TargetTable[2][1] = TargetTable[0][1];
					
					TargetTable[3][0] = TargetTable[0][0];
					TargetTable[3][1] = TargetTable[4][1];
					
					TargetTable[5][0] = TargetTable[8][0];
					TargetTable[5][1] = TargetTable[4][1];
					
					TargetTable[6][0] = TargetTable[0][0];
					TargetTable[6][1] = TargetTable[8][1];
					
					TargetTable[7][0] = TargetTable[4][0];
					TargetTable[7][1] = TargetTable[8][1];
					
					FetchMode = 0;
					HitMode = 1;
					TestSDSU = 0;
					FetchFinish =0;
					j=1;
					return;
					}
//			if(j>3)
//				{j=1;}	
	break;}
	case 9:{
					if(CanFetchFlag)
					{
						TargetTable[Fetch_cnt][0] = Chassis_Control_Info.Chassis_angle;
						TargetTable[Fetch_cnt][1] = Pitch_Hold_Info.angle;
						Fetch_cnt++;
						CanFetchFlag = 0;
					}
			if(Fetch_cnt>8)
				{	
					Fetch_cnt=0;
					FetchMode = 0;
					HitMode = 1;
					TestSDSU = 0;
				}	
					break;
				}
	}
}

static void VisonControl(float target,uint8_t flag)
{
		float core_delta , core_p_part;
		float shell_delta , shell_p_part , shell_d_part;
	
		//1. shell control
		shell_delta = target - Chassis_Control_Info.Chassis_angle;
		
		shell_p_part = shell_delta * 300;
		
		//shell_d_part = yaw_Hold_Info.angle_speed * Yaw_para.shell_D * 0.01f;//D SHELL
		
		yaw_Hold_Info.shell_out = shell_p_part;
		
		
		//2. core control
		
		core_delta = yaw_Hold_Info.shell_out - yaw_Hold_Info.angle_speed;
		
		core_p_part = core_delta * 0.01f * 150;
		
		if( core_p_part >32000){
			core_p_part = 32000;
		}
		else if(core_p_part < -32000){
			core_p_part = -32000;
		}

	
		if( flag == 1 )
		{
			yaw_Hold_Info.out = core_p_part;
		}
		else
		{
			yaw_Hold_Info.out = 0;
		}
}


