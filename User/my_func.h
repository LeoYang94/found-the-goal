#ifndef __MY_FUNC_H__
#define __MY_FUNC_H__
#include "stm32f4xx.h"
#include <stdio.h>

float Amplitude_Limiting(uint8_t flag , float input , float High , float Low);


float Subsection_PID(uint8_t flag , float raw , uint8_t num , float * K , float * Scale);


#endif

