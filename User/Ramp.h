#ifndef __RAMP_H_
#define __RAMP_H_

#include<stdint.h>
#include "stm32f4xx.h" 


typedef struct RampTime{
		uint32_t lasttime;
		uint32_t now;
		uint32_t count;
		float out;
		int lastKey;
		int isSameKey;
		
		int32_t clock_cnt;
}RampTime;

extern RampTime RampTFB;
extern RampTime RampTLF;
float RampCal(RampTime *RampT);

#define NORMAL_FORWARD_BACK_SPEED 			700
#define NORMAL_LEFT_RIGHT_SPEED   			600
#define HIGH_FORWARD_BACK_SPEED 			660
#define HIGH_LEFT_RIGHT_SPEED   			800

void ComputerControl(void);
void TIM9_Configuration(void);
	

#endif
