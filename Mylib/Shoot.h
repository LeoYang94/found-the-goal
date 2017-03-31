#ifndef __SHOOT_H_
#define __SHOOT_H_
#include<stdint.h>
#include "stm32f4xx.h" 

struct SHOOT_INFO
{
	uint8_t shoot_flag;
	
	int32_t shoot_cnt;
	
	uint32_t pulse_cnt;
	float pulse_target;
	uint32_t pulse_inteval;
	float shell_out;
	
	uint8_t load_command;
	
	uint16_t speed_filter_cnt;
	
	uint16_t Call_cnt;
	
	short speed;
	float Load_Motor_Speed;
	float speed_buff[5];
	
	float Load_Motor_Speed_Target;
	
	float Load_Motor_Out;
	
	uint8_t Load_motor_error;
	uint8_t Load_motor_out_flag;
	
};

extern struct SHOOT_INFO Shoot_Info;




void Shoot_PWM_Configuration(void);
	


void Shoot_Fire(uint16_t speed);


void Detect_shoot(void);

void Shoot_Info_Init(void);

void Shoot_Keep(void);
void Detect_Cap(void);

#endif

