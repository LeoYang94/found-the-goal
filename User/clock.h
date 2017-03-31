#ifndef _CLOCK_H
#define _CLOCK_H
#include "sys.h"

#define STATE_INIT 1
#define STATE_RUN 2
#define STATE_STOP 0


/*ต๗สิสนำร*/
#define COME_TO_ZERO 1

#define HOLDER_OUT 1

#define WHEEL_OUT 1



void Clock_Int_Init(void);

void Sys_Configuration(void);


uint32_t Get_Time_Micros(void);

void ControlTask(uint8_t flag);

extern int32_t clock_cnt;
extern uint8_t Run_state ;
#endif
