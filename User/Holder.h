
//2016.4.17
#ifndef __HOLDER_H_
#define __HOLDER_H_
#include<stdint.h>
#include "stm32f4xx.h" 

#define GYRO_FILTER_NUM 10

#define MOTOR_6025 0
#define MOTOR_6623 1

#define SHELL_I_PART_MAX 150
#define PITCH_HOLD_SHELL_OUT_MAX 10000




struct Hold_Info
{
	
	uint8_t angle_flag ;
	
	uint8_t init_ok;
	
	int16_t angle_speed_16;
	int16_t angle_speed_buff[GYRO_FILTER_NUM];
		
	int16_t can_angle_raw;
	int16_t can_angle_new;
	float can_angle;
	
	float angle;
	float angle_temp;
	float angle_old;
	
	float angle_delta_interal;
	
	float angle_speed;
	float angle_speed_delta_interal;
	
	float angle_target;
	float angle_target_old;
	float receive_speed;
	int receive_speed_raw;
	
	float shell_out;
	float out;
	
};
extern struct Hold_Info Pitch_Hold_Info;
extern struct Hold_Info yaw_Hold_Info;	


struct PID_PARA
{
	uint16_t shell_P;
	float shell_I;
	float shell_D;
	
	uint16_t core_P;
	float core_I;
	float core_D;
};
	
extern struct PID_PARA Pitch_para;
extern struct PID_PARA Pitch_para_init;
extern struct PID_PARA Yaw_para;
extern uint8_t ComeToZero_Flag ;



int8_t Holder_Pitch_Control(uint8_t flag , struct PID_PARA * p , float target , float feedback_angle);

int8_t Holder_Yaw_Control(uint8_t flag , struct PID_PARA * p , float target ,float feedback_angle);

float sub_pitch();

void ComeToZero(uint8_t flag);

void Holder_update_para(void);

void Holder_motor_calibration(void);
#endif

