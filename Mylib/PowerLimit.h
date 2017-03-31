#ifndef __POWERLIMIT_H__
#define __POWERLIMIT_H__
#include "sys.h"


struct S_POWER_INFO
{
	uint8_t Over_Joule_flag;
	
	float Joule_limit_max;
	float Joule_limit_min;
	float Joule_limit_recover;
	double Joule_left;
	
	float Power_real;
	float Power_limit;
	
};

extern struct S_POWER_INFO S_Power_info;


float Range(float low , float high , float var);

void Power_init(void);
void Power_info_update(void);

void Update_PowerInformation(uint8_t flag);

#endif
