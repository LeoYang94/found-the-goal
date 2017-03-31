
//2016.4.17
#ifndef __REMOTE_H_
#define __REMOTE_H_
#include<stdint.h>
#include "stm32f4xx.h" 




typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
		uint8_t ChangeFlag;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t;

extern RC_Ctl_t RC_Ctl;


extern unsigned char sbus_rx_buffer[18];

extern uint8_t IsComputerControl;

void Remote_Config(void);



uint8_t In_range( float low , float high , float var );


#endif

