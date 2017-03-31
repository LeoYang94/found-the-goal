#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__


#include "stm32f4xx.h"
#include <stdio.h>

struct Chassis_Info
{
	uint8_t Anaconda_flag;
	
	int16_t Chassis_angle_raw;
	
	
	float Chassis_angle_Target;
	float Chassis_angle;
	
	float Chassis_out;

	
};

extern struct Chassis_Info Chassis_Control_Info;
extern struct PID_PARA Chassis_para;

extern int16_t	chassis_f0;
extern int16_t chassis_r0 ;
extern int16_t chassis_y0 ; 

int8_t Chassis_Control(void);
int8_t Chassis_Remote_Dispack(uint8_t flag);
float AmplitudeCheck(float Angle , float limit);

void remote_data_deal(uint8_t flag);

void Chessis_Info_Init(void);

void Lets_Rock(uint8_t flag);

void Anaconda(uint8_t flag);
#endif



