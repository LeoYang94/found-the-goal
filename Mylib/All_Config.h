#ifndef __ALL_CONFIG_H_
#define __ALL_CONFIG_H_

#include "stm32f4xx.h" 


#define YAW_SENSITY 0.050f		//鼠标yaw轴灵敏度
#define PITCH_SENSITY 0.03f	//鼠标pitch轴灵敏度

#define LEFT_LIMINT_ANGLE 20.0f //云台左侧角度限制
#define RIGHT_LIMINT_ANGLE -20.0f//云台右侧角度限制

#define PITCH_NEGTIVE_LIMIT_ANGLE -12.0f		//云台上方角度限制
#define PITCH_POSITIVE_LIMIT_ANGLE 12.0f		//云台下方角度限制

#define YAW_NEGTIVE_LIMIT_ANGLE -100.0f		//云台上方角度限制
#define YAW_POSITIVE_LIMIT_ANGLE 100.0f		//云台下方角度限制


#define LEFT_CHESSIS_LIMIT 25.0f //底盘左边限制
#define RIGHT_CHESSIS_LIMIT -25.0f//底盘右边限制

#define pitch_limit_under  12.0f
#define pitch_limit_on    -10.0f
#define yaw_limit_left 50.0f
#define yaw_limit_right -50.0f
//#define YAW_INITIAL_VAL  6896.0f //YAW 轴零点编码器值
//#define PITCH_INITIAL_VAL  1500.0f //PITCH 轴零点编码器值
#define K_CODE_2_ANGLE  0.04132f  //编码器值换算到角度系数

#define K_ANGLESPEED_2_ANGLE 0.00003272f //陀螺仪积分获得角度系数




#endif

