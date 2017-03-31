#ifndef __ALL_CONFIG_H_
#define __ALL_CONFIG_H_

#include "stm32f4xx.h" 


#define YAW_SENSITY 0.050f		//���yaw��������
#define PITCH_SENSITY 0.03f	//���pitch��������

#define LEFT_LIMINT_ANGLE 20.0f //��̨���Ƕ�����
#define RIGHT_LIMINT_ANGLE -20.0f//��̨�Ҳ�Ƕ�����

#define PITCH_NEGTIVE_LIMIT_ANGLE -12.0f		//��̨�Ϸ��Ƕ�����
#define PITCH_POSITIVE_LIMIT_ANGLE 12.0f		//��̨�·��Ƕ�����

#define YAW_NEGTIVE_LIMIT_ANGLE -100.0f		//��̨�Ϸ��Ƕ�����
#define YAW_POSITIVE_LIMIT_ANGLE 100.0f		//��̨�·��Ƕ�����


#define LEFT_CHESSIS_LIMIT 25.0f //�����������
#define RIGHT_CHESSIS_LIMIT -25.0f//�����ұ�����

#define pitch_limit_under  12.0f
#define pitch_limit_on    -10.0f
#define yaw_limit_left 50.0f
#define yaw_limit_right -50.0f
//#define YAW_INITIAL_VAL  6896.0f //YAW ����������ֵ
//#define PITCH_INITIAL_VAL  1500.0f //PITCH ����������ֵ
#define K_CODE_2_ANGLE  0.04132f  //������ֵ���㵽�Ƕ�ϵ��

#define K_ANGLESPEED_2_ANGLE 0.00003272f //�����ǻ��ֻ�ýǶ�ϵ��




#endif

