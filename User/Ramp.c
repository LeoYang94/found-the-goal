#include "ramp.h"
#include "Remote.h"
#include "Chassis_Control.h"

RampTime RampTFB ;
RampTime RampTLF ;


float RampCal(RampTime *RampT)
{
	if(RampT->isSameKey ==1 )
	{
		RampT->now = (*RampT).clock_cnt;	
	}
	else {
		RampT->lasttime = (*RampT).clock_cnt;
		RampT->now = (*RampT).clock_cnt;	
	}
	RampT->count = RampT->now  - RampT->lasttime;
	RampT->out = (float)RampT->count /500.0f;

	if(RampT->out > 1) RampT->out = 1;
	
	return RampT->out;
	
#if 0
	if(RampT->isSameKey ==1 )
	{
		RampT->now = Get_Time_Micros();	
	}
	else {
		RampT->lasttime = Get_Time_Micros();
		RampT->now = Get_Time_Micros();	
	}
	RampT->count = RampT->now  - RampT->lasttime;
	RampT->out = (float)RampT->count /500000.0f;

	if(RampT->out > 1) RampT->out = 1;
	return RampT->out;
	
	#endif
}


void ComputerControl(void){
	static int32_t forward_back_speed = 0;
	static int32_t left_right_speed = 0;
		if(RC_Ctl.key.v & 0x10)
		{
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		if(RC_Ctl.key.v & 0x01)  
		{
			if(RampTFB.lastKey & 0x01){
				RampTFB.isSameKey =1;
			}
			else {
				RampTFB.isSameKey =0;
			}
			forward_back_speed = forward_back_speed*RampCal(&RampTFB);
		}
		else if(RC_Ctl.key.v & 0x02) 
		{
			if(RampTFB.lastKey & 0x02){
				RampTFB.isSameKey =1;
			}
			else {
				RampTFB.isSameKey =0;
			}
			forward_back_speed = -forward_back_speed*RampCal(&RampTFB);
		}
		else
		{
			forward_back_speed=0;
			RampTFB.isSameKey =0;
		}
		
		if(RC_Ctl.key.v & 0x04)  
		{
			if(RampTLF.lastKey & 0x04){
				RampTLF.isSameKey =1;
			}
			else {
				RampTLF.isSameKey =0;
			}
			left_right_speed = left_right_speed*RampCal(&RampTLF);
		}
		else if(RC_Ctl.key.v & 0x08) 
		{
			if(RampTLF.lastKey & 0x08){
				RampTLF.isSameKey =1;
			}
			else {
				RampTLF.isSameKey =0;
			}
			left_right_speed = -left_right_speed*RampCal(&RampTLF);
		}
		else
		{
			left_right_speed=0;
			RampTLF.isSameKey =0;
		}
		RampTFB.lastKey =RC_Ctl.key.v;
		RampTLF.lastKey =RC_Ctl.key.v;
	
		chassis_f0 = forward_back_speed;
		chassis_r0 = -left_right_speed;
}



