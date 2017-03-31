#ifndef __CAN1_H__
#define __CAN1_H__
#include "stm32f4xx.h"

void CAN2_Configuration(void);
void CAN2_Send(uint32_t ID,u8 *Data);



extern CanRxMsg RxMessage_205;
extern CanRxMsg RxMessage_206;
extern CanRxMsg RxMessage_201;
extern CanRxMsg RxMessage_202;
extern CanRxMsg RxMessage_203;
extern CanRxMsg RxMessage_204;


extern int Turn;


void Motro_test_out(uint16_t id  , float * speed_list );

#endif 

