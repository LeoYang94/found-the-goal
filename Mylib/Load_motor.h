#ifndef __LOAD_MOTOR_H_
#define __LOAD_MOTOR_H_
#include<stdint.h>
#include "stm32f4xx.h" 

#define LOAD_MOTOR_DETECT_OR_NOT 0

#define LOAD_MOTOR_SPEED_MIN 0.1f
#define LOAD_MOTOR_OVER_CNT  10

void Load_Motor_PWM_Configuration(void);

void Encoder_Configuration(void);

void Load_motor_control(uint8_t flag , uint8_t type);

void Bullet_Cap_Conf(void);

void Bullet_Cap_Close(void);
void Bullet_Cap_Open(void);

void Load_motor_out(uint32_t a);

void Load_Motor_position_plus(uint16_t adds);

void Load_Motor_Position_plus(uint32_t adds , uint8_t delay_s);

void Load_Motor_Fault_detet(void);
#endif

