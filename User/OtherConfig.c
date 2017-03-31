#include "OtherConfig.h"

/*
	弹舱舵机参数
	
	云台Pitch Yaw 补偿值
	
	格式：
	
	开门--关门--Yaw--Pitch--拨弹电机类型(0:官方 1：自制)--驱动板（0：官方  1：自制）--Pitch处理--射速--是否更换6623--是否支持妙算--是否新电调
	0			1			2			3			4																5													6，7				8			9								10								11			
*/

const uint16_t LaserAndPrep[5][12]=
{

{11,21,9390,7831,1,1,666,666,1370,1,1,0},//?? ????

{22,15,5465,7733,0,0,666,666,1530,0,0,1},					//p

{10,20,3740,4315,0,0,666,666,1370,1,0,0},

{9,19,2822,6784,0,0,666,666,1360,1,0,0},

{25,15,5015,965,0,0,666,666,1360,1,0,1}

};


/*
	激光笔引脚初始化 PA9  
	开机一直开启
*/
void Laser_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //??
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	PAout(8) = 1;
	
}














