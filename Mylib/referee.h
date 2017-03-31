#ifndef __REFEREE_H__
#define __REFEREE_H__

#include "stm32f4xx.h" 
#include <stdio.h>


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define BSP_USART3_DMA_RX_BUF_LEN	20u
#define REFEREE_REV_DMA_CNT (DMA1_Stream1->MA_CNDTR)//DMA?????
#define k_yaw  0.003625f  //ÏµÊýb


typedef struct
{
	uint16_t left_time_M;
	uint16_t left_time_S;
	uint16_t left_HP;
	float voltage_V;
	float current_A;
	float power_W;
	uint8_t length;
	uint8_t GameBegin;
}_JUDGMENT_01_DATA;

typedef struct
{
	uint8_t weakid;
	uint8_t weak;
	uint16_t ValueChange;
}_JUDGMENT_02_DATA;

typedef struct
{
	float small_bullet_speed;
	float small_bulet_frequency;
	float big_bullet_speed;
	float big_bulet_frequency;
	uint8_t length;
}_JUDGMENT_03_DATA;

void Referee_Configuration(void);
void send_odm_msg(float * data);
void send_check(uint16_t data);
void Send_CheckData(void);

void auto_shoot_mode(void);
void camera_receive(void);

extern _JUDGMENT_01_DATA Judgment_01_data;
extern _JUDGMENT_02_DATA Judgment_02_data;
extern _JUDGMENT_03_DATA Judgment_03_data;
#endif


