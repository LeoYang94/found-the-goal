#include "stm32f4xx.h"
#include "sys.h" 
#include "delay.h"
#include "led.h" 
#include "USART3.h"
#include "CAN2.h"
#include "I2C.h"
#include "6050.h"
#include "clock.h"
#include "Wheel_Speed.h"
#include "Shoot.h"
#include "Remote.h"
#include "Holder.h"
#include "Load_motor.h"
#include "Motor_out.h"
#include "PowerLimit.h"
#include "referee.h"
#include "BKP_DATA.h"
#include "MyWDG.h"
#include "OtherConfig.h"
#include "All_Config.h"
#include "imu.h"
#include "Chassis_Control.h"
#include "Hit.h"
#include "I2C.h"
//����BKP
uint32_t ORI[3]={1000,2000,3000};
uint32_t BKP_DATA[3]; 
//***************************************


/*
2016.7.11

ʵ���޸�

������ס�Ҽ�1.5s�����߿���Ħ����

����������ס�Ҽ�1.5s���ر�Ħ����
*/

/*
2016.8.1
��̬����
*/

/*

2016.8.6

�޸Ķ�ʱ��

���ݵ��ֶ�ʱ��


*/

/*
2016.8.7

�Ż����̵�����,����

*/

/*
2016.8.10

���� ���㲽��������ͨ������
���㲽���� 1��

*/


/*
2016.8.16
�Ż�������
ƽ��ң��������


//ע�͵����й�������Ͳ���������ã�MAIN��CLOCK��

//ע�͵����й��ڵ��̵Ķ���
//CLOCKҲ���ϣ������൱��ֻ��can

*/
/*
20177.3.30 ������̨�Զ�ʶ��װ�׳���
����
*/
uint32_t p1 , p2 , p3;

int16_t speed_out = 800;
extern float send_anto_data[4];

void my_delay(int32_t num)
{
	while(num >0)
	{
		num--;
	}
}
int main(void)
{     

	float speed_main[4] = {1200,1200,1200,1200};
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;
	
	RC_Ctl.rc.ChangeFlag = 0;
	IsComputerControl = 0;//�ֱ�ģʽ	
	Holder_update_para();//��̨������ʼ��
	delay_init(168);
	if( LaserAndPrep[TANK_SERIAL_NUMBER-1][10] == 1 )//֧������
	{
		Hit_flag_init();
		CalculateKey();
	}	
	
	//Shoot_PWM_Configuration();
	//Shoot_Fire(1000);
	LED_Init();	
	Referee_Configuration();
	//Load_Motor_PWM_Configuration();//�������
	//Shoot_Info_Init();//���������ʼ��
	CAN2_Configuration();
	Holder_Motor_output(0);//���Ϊ0�����͸���
	delay_ms(1000);

	//Encoder_Configuration();//�������������
	
	//Bullet_Cap_Conf();//���ֶ����ʼ��
	//Bullet_Cap_Open();
	Remote_Config();//Զ��ң�س�ʼ��	
	I2C_INIT();
	InitMPU6050();//6050��ʼ��
	Gyro_OFFEST();
	Sys_Configuration();//1ms ���ж�
	delay_ms(1000);
	Laser_Init();//�����ʼ��
	Clock_Int_Init();//1ms
	
	
	while(1)
	{
		PrepareForIMU(1);
		IMUupdate(sensor.gyro.radian.x,
							sensor.gyro.radian.y,
						  sensor.gyro.radian.z,
							sensor.acc.averag.x,
							sensor.acc.averag.y,
							sensor.acc.averag.z);
							
		Attitude_update();
		/***********����************/
    //send_odm_msg(send_anto_data);
		#if 1
		send_anto_data[0] = Pitch_Hold_Info.angle_target;
			
		send_anto_data[1] = Pitch_Hold_Info.angle;
		
		send_anto_data[2] = yaw_Hold_Info.angle_target;
		
		send_anto_data[3] = yaw_Hold_Info.angle;
		
		
		#endif
		//Debug_uart3_send(Send_data);
		
	}
	
	
}




