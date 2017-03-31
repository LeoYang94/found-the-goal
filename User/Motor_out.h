#ifndef __MOTOR_OUT_H_
#define __MOTOR_OUT_H_
#include<stdint.h>
#include "stm32f4xx.h" 


#define PITCH_OUT_MAX 4990
#define YAW_OUT_MAX 4100

int8_t Holder_Motor_output(int8_t flag );
void can2_send(int16_t speed);

#endif



