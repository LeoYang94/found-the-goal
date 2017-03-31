#include "Holder.h"
#include "Motor_out.h"
#include "USART3.h"
#include "OtherConfig.h"
#include "clock.h"
#include "Chassis_Control.h"
#include "Prepare_data.h"
//2017.3.23

#define ENABLE 1
#define DISABLE 0

struct Hold_Info Pitch_Hold_Info;
struct Hold_Info yaw_Hold_Info;	

uint8_t ComeToZero_Flag = 0;


struct PID_PARA Pitch_para;

struct PID_PARA Pitch_para_6025 = 
{
	750, 0 , 0,
	290 , 0 , 0
};


struct PID_PARA Pitch_para_6623 = 
{
	250, 0 , 40,
	120, 0 , 0
}	;



struct PID_PARA Pitch_para_init = 
{
	185, 50 , 0,
	60 , 0 , 0
}	;




struct PID_PARA Yaw_para = 
{
	400,0,0,
	80,0,0
};


void Holder_update_para(void)
{
	if ( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6025)
	{
		Pitch_para.shell_P = Pitch_para_6025.shell_P;
		Pitch_para.shell_I = Pitch_para_6025.shell_I;
		Pitch_para.shell_D = Pitch_para_6025.shell_D;
		Pitch_para.core_P = Pitch_para_6025.core_P;
		Pitch_para.core_I = Pitch_para_6025.core_I;
		Pitch_para.core_D = Pitch_para_6025.core_D;
		
		
	}
	else if ( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6623)
	{
		Pitch_para.shell_P = Pitch_para_6623.shell_P;
		Pitch_para.shell_I = Pitch_para_6623.shell_I;
		Pitch_para.shell_D = Pitch_para_6623.shell_D;
		Pitch_para.core_P = Pitch_para_6623.core_P;
		Pitch_para.core_I = Pitch_para_6623.core_I;
		Pitch_para.core_D = Pitch_para_6623.core_D;
	}
}

float	shell_i_part;
float shell_i_div = 2;
int16_t pitch_compensate = -100;
int8_t Holder_Pitch_Control(uint8_t flag , struct PID_PARA * p , float target , float feedback_angle)
{
	
	float core_delta , core_p_part;
	float shell_delta , shell_p_part , shell_d_part;
	
	
	//1. shell control
	shell_delta = target- feedback_angle;//0 - Pitch_Hold_Info.angle;
	
	shell_p_part = shell_delta * (*p).shell_P;
	
	shell_i_part += shell_delta/shell_i_div;
	shell_i_part  = AmplitudeCheck(shell_i_part , SHELL_I_PART_MAX);
		
	shell_d_part = Pitch_Hold_Info.angle_speed * (*p).shell_D * 0.01f;//D SHELL
	
	Pitch_Hold_Info.shell_out = shell_p_part + shell_i_part * (*p).shell_I * 0.01f + shell_d_part;
	
	AmplitudeCheck(Pitch_Hold_Info.shell_out , PITCH_HOLD_SHELL_OUT_MAX); 
	
	//2. core control
	
	core_delta = Pitch_Hold_Info.shell_out - Pitch_Hold_Info.angle_speed/1.0;
	
	core_p_part = core_delta * 0.01f * (*p).core_P;
	
	if( core_p_part >32000)
	{
		core_p_part = 32000;
	}
	else if(core_p_part < -32000)
	{
		core_p_part = -32000;
	}

	
	if( flag == 1 )
	{
		if( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6623 )
		{
			pitch_compensate = -100;
			pitch_compensate += sub_pitch();
			Pitch_Hold_Info.out = core_p_part + pitch_compensate;
		}
		else
		{
			Pitch_Hold_Info.out = core_p_part;
		}
	}
	else
	{
		Pitch_Hold_Info.out = 0;
	}
	
	return 1;
	
}

int8_t Holder_Yaw_Control(uint8_t flag , struct PID_PARA * p , float target ,float feedback_angle)
{
	
	float core_delta , core_p_part;
	float shell_delta , shell_p_part , shell_d_part;
	
	
	//1. shell control，绝对式PID
	shell_delta = target - feedback_angle;
	
	shell_p_part = shell_delta * (*p).shell_P;
	
	shell_d_part = yaw_Hold_Info.angle_speed * (*p).shell_D * 0.01f;//D SHELL
	
	yaw_Hold_Info.shell_out = shell_p_part + shell_d_part;
	
	
	//2. core control，绝对式PID
	
	core_delta = yaw_Hold_Info.shell_out - yaw_Hold_Info.angle_speed;
	
	core_p_part = core_delta * 0.01f * (*p).core_P;
	
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

	return 1;
	
}


float sub_pitch()
{
	int flag;
	float out;
	
	flag = -Pitch_Hold_Info.angle / abs(Pitch_Hold_Info.angle);
	
	if(abs(Pitch_Hold_Info.angle) < 2)
		out = 0;
	
	else if(abs(Pitch_Hold_Info.angle) < 4)
		out = 86*flag;
	
	else if(abs(Pitch_Hold_Info.angle) < 6)
		out = 130*flag;
	
	else if(abs(Pitch_Hold_Info.angle) < 8)
		out = 173*flag;
	
	else if(abs(Pitch_Hold_Info.angle) < 10)
		out = 216*flag;
	
	else
		out = 260*flag;
	
	return out;

}





void ComeToZero(uint8_t flag )
{
	if( flag == 0 )
	{
		Holder_Motor_output(0);//输出为0，降低干扰
		return;
	}
	
	Attitude_update();
	Holder_Pitch_Control(ENABLE , &Pitch_para_init , 0 , Pitch_Hold_Info.can_angle);//pitch
	Holder_Yaw_Control(ENABLE , &Yaw_para , 0 , yaw_Hold_Info.can_angle);//YAW
	Holder_Motor_output(1);//falg=1

}

void Holder_motor_calibration(void)
{
	uint8_t TransmitMailbox;//发送信箱号
	CanTxMsg TxMessage;
	
	
	TxMessage.StdId = 0x1ff;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
	TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
	TxMessage.DLC=8;							 //数据长度为2字节

	
	
	TxMessage.Data[0]=0;
  TxMessage.Data[1]=0 ; 
	

	TxMessage.Data[2]=0;
	TxMessage.Data[3]=0;
	
	TxMessage.Data[4]=0;
	TxMessage.Data[5]=0;
	TxMessage.Data[6]=0x04;
	TxMessage.Data[7]=0;

	TransmitMailbox = CAN_Transmit(CAN2,&TxMessage);
	
	while(CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)
	{
		;
	}
}


