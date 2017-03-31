#include "Wheel_Speed.h"
#include "Holder.h"
#include "OtherConfig.h"


#define ENABLE 1
#define DISABLE 0
#define NEW 1
#define OLD 0



//为了适应低功耗
struct PID_PARA Wheel_para ;



struct PID_PARA Wheel_para_old = 
{
	50,0.05f,0,//0.2f
	0,0,0
};



//新电调比旧电调输出小，参数相应增大
struct PID_PARA Wheel_para_new = 
{
	65,0.05f,0,//0.2f
	0,0,0
};




void Wheel_para_update(void)
{
	if ( LaserAndPrep[TANK_SERIAL_NUMBER-1][11] == NEW)
	{
		Wheel_para.shell_P = Wheel_para_new.shell_P;
		Wheel_para.shell_I = Wheel_para_new.shell_I;
		Wheel_para.shell_D = Wheel_para_new.shell_D;
		
		Wheel_para.core_P = Wheel_para_new.core_P;
		Wheel_para.core_I = Wheel_para_new.core_I;
		Wheel_para.core_D = Wheel_para_new.core_D;
	}
	else if ( LaserAndPrep[TANK_SERIAL_NUMBER-1][11] == OLD)
	{
		Wheel_para.shell_P = Wheel_para_old.shell_P;
		Wheel_para.shell_I = Wheel_para_old.shell_I;
		Wheel_para.shell_D = Wheel_para_old.shell_D;
		
		Wheel_para.core_P = Wheel_para_old.core_P;
		Wheel_para.core_I = Wheel_para_old.core_I;
		Wheel_para.core_D = Wheel_para_old.core_D;
	}
}


/*

实际最大速度1000左右  不确定

实际给定速度最大 2000左右



遥控实际给定最大1500

*/


struct Wheel_Info Four_Wheel_Info;
	
void Wheel_Info_Init(void)
{
	if ( LaserAndPrep[TANK_SERIAL_NUMBER-1][11] == NEW)
	{
		Four_Wheel_Info.out_limit = 15000;
		Four_Wheel_Info.speed_limit = 1900;
		Four_Wheel_Info.speed_K = 0.025f;
	}
	else
	{
		Four_Wheel_Info.out_limit = 15000;
		Four_Wheel_Info.speed_limit = 1900;
		Four_Wheel_Info.speed_K = 0.025f;
	}

}



static float Wheel_dead_bias(uint8_t flag , float input , float bias)//增加电机偏置
{
	if( flag == 0 )
	{
		return input;
	}
	
	if( input > 0 )
	{
		input += bias;
	}
	else if( input < 0 )
	{
		input -= bias;
	}
	
	return input;
	
}
	

static float Multy_section_PID(uint8_t flag , float input)
{
	if(flag == 0 )
	{
		return input;
	}
	
	if( abs(input) < 200 )
	{
		input = input;
	}
	
//	else if( abs(input) < 300 )
//	{
//		input = input * 0.95f;
//	}
//	
	else if( abs(input) < 400 )
	{
		input = input * 0.9f;
	}
//	
//	else if( abs(input) < 500 )
//	{
//		input = input * 0.85f;
//	}

	else if( abs(input) < 600 )
	{
		input = input * 0.8f;
	}
		else if( abs(input) < 700 )
	{
		input = input * 0.75f;
	}
	
	else if( abs(input) < 800 )
	{
		input = input * 0.7f;
	}
	else
	{
		input = input * 0.65f;
	}
	return input;
}



uint8_t Wheel_Speed_control(uint8_t flag)
{
	float delta , delta_b ;
	
	float p_part,i_part , out;
	
	static int8_t num = 0;
	
	int8_t i,wheel_cnt;
	
	float sum[4];
	
	float speed_temp;
	/***********NUM1 计算四个轮子的电机输出*************/
	for( wheel_cnt = 0;wheel_cnt<4;wheel_cnt++ )
	{
		
		//计算target
		Four_Wheel_Info.speed_diff_cnt ++;
		
		speed_temp = Four_Wheel_Info.Target_speed_Old[wheel_cnt] + \
		 Four_Wheel_Info.speed_diff_cnt * \
		(( Four_Wheel_Info.Target_speed_New[wheel_cnt]- Four_Wheel_Info.Target_speed_Old[wheel_cnt] ) / 7.0f) ;//??
		

		if( abs(speed_temp) > abs(Four_Wheel_Info.Target_speed_New[wheel_cnt]) )//abs:绝对值函数
		{
			speed_temp = Four_Wheel_Info.Target_speed_New[wheel_cnt];
		}
		
		if( Four_Wheel_Info.speed_diff_cnt >= 7 )
		{
			Four_Wheel_Info.speed_diff_cnt = 0;
		}
		
		Four_Wheel_Info.Target_speed[wheel_cnt] = speed_temp;
		
		
		
		//根据target和feedback 计算
		Four_Wheel_Info.speed[wheel_cnt] = Four_Wheel_Info.speed_raw[wheel_cnt] / 5.0f;//feedback
		
			/*(2)计算PID*/
		delta = Four_Wheel_Info.Target_speed[wheel_cnt] - Four_Wheel_Info.speed[wheel_cnt];
		
		 
		delta_b = Multy_section_PID( ENABLE ,   delta);
		
		#if 1
		if( (wheel_cnt == 2)||(wheel_cnt==3) )
		{
			p_part = ( Wheel_para.shell_P ) * 0.1f * delta_b * 1.1f;//后轮比前轮承重大，参数适当放大
		}
		else
		{
			p_part = ( Wheel_para.shell_P ) * 0.1f * delta_b;//P PART 缩小10倍
		}
		
		
		#else
		p_part = ( Wheel_para.shell_P ) * 0.1f * delta_b;//P PART 缩小10倍
		#endif
		
		/*****************积分操作*************/
		Four_Wheel_Info.interval += delta;
		
		if( Four_Wheel_Info.interval > 6000 )
		{
			Four_Wheel_Info.interval = 6000;
		}
		else if( Four_Wheel_Info.interval < -6000)
		{
			Four_Wheel_Info.interval = -6000;
		}
		
		
		i_part = Four_Wheel_Info.interval * 0.1f * Wheel_para.shell_I;
		
		out = p_part + i_part ;
		
		Four_Wheel_Info.out[wheel_cnt] = Wheel_dead_bias(DISABLE , out , 200);//增加电机偏置
		
			/*(3)输出限幅*/
		if( Four_Wheel_Info.out[wheel_cnt]  > Four_Wheel_Info.out_limit )
		{
			Four_Wheel_Info.out[wheel_cnt]  = Four_Wheel_Info.out_limit;
		}

		else if( Four_Wheel_Info.out[wheel_cnt]  < -Four_Wheel_Info.out_limit )
		{
			Four_Wheel_Info.out[wheel_cnt]  = -Four_Wheel_Info.out_limit;
		}
	}
	
	num = (num + 1 )%SPEED_FILTER_NUM;//滑动
	
	
	/***********NUM2 输出计算值到电机*************/

	//1.消除开机时的干扰
	if( Four_Wheel_Info.start_flag < 20 )
	{
		Four_Wheel_Info.start_flag ++;
		
		return 0;
	}
	
	else
	{
		//输出到电机
		if( flag == DISABLE_MOTOR_OUT)
		{
			Wheel_out( NO_OUT , Four_Wheel_Info.out);
		}
		
		else
		{
			Wheel_out( ALL_OUT , Four_Wheel_Info.out);
		}

		return 1;
	}

}


/*
 * 函数名：Wheel_out
 * 描述  ：控制四个底盘电机的输出
 * 输入  ：1. out_mode
		可选参数：NO_OUT  : 无电机输出模式
							ALL_OUT  :所有电机输出模式
							SIGNAL_OUT	:单个电机输出模式

 * 输出  : 无

 */	 
void Wheel_out(uint8_t out_mode , float * speed_list )
{
	int16_t i16_speed_list[4];
	int8_t i;
	uint8_t TransmitMailbox;
	
	int16_t t;
	
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x200;	//？1ff和200的区别在哪里
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
  TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
  TxMessage.DLC=8;							 //数据长度为2字节
	

	
	if( out_mode == ALL_OUT )//所有电机输出模式
	{
		for( i=0;i<4;i++ )
		{
			i16_speed_list[i] = (int16_t)(*(speed_list+i));
			
			TxMessage.Data[i*2+0] = (uint8_t)(i16_speed_list[i]>>8);
			
			TxMessage.Data[i*2+1] = (uint8_t)(i16_speed_list[i]);
		}
	}
	
	else
	{
		TxMessage.Data[0]=0;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
	}
	

	TransmitMailbox=CAN_Transmit(CAN2,&TxMessage);
	t=0;
	while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t<255))//这几个数字的含义是什么
	{
		t++;
	}

}

