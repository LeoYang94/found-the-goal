#ifndef __USART3_H__
#define __USART3_H__


#include "stm32f4xx.h"
#include <stdio.h>
#include "Holder.h"

#define PARA_REV_NUM 100      //?????
#define PRAR_REV_DMA_CNT (DMA1_Stream1->MA_CNDTR)//DMA?????

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
#define abs(x) ((x)>0? (x):(-(x)))
//judgment_data
#define DATALENGTH_01 46
#define DATALENGTH_02 11
#define DATALENGTH_03 24
#define BUFFERSIZE 100

typedef struct
{
	uint8_t  rev_state;
	uint16_t data_num_now;
	uint16_t data_num_last;
	uint16_t rev_overtime_cnt;
	
}_PARA_REV;

typedef struct
{
	uint8_t left_time;
	uint8_t left_HP;
	float voltage_V;
	float current_A;
	float power_W;
}_JUDGMENT_01_DATA;

typedef struct
{
	float small_bullet_speed;
	float small_bulet_frequency;
	float big_bullet_speed;
	float big_bulet_frequency;
}_JUDGMENT_03_DATA;

extern _PARA_REV Para_s_rev;
extern uint8_t para_rev[PARA_REV_NUM];

extern float Send_data[4];

/*********????*******/

void USART3_Configuration(void);
void uart3_send_multi_data(float *data);
void USART3_Configuration(void);



uint8_t Detect_para_rev_dma(uint16_t cnt , struct PID_PARA *PARA_INDEX);//????PID??

uint8_t Updata_PID(uint8_t *judgment_data);
uint8_t get_judgment_data(uint8_t flag);
void GetDataFrame(uint8_t dataFrameID);
//CRC

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
#endif
