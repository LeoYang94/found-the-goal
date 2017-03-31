#include "my_func.h"
#include "USART3.h"


//ÏÞ·ùÊä³ö
float Amplitude_Limiting(uint8_t flag , float input , float High , float Low)
{
	float result;
	
	if( flag == 0 )
	{
		return input;
	}
	
	
	if( input > High)
		result = High;
	else if( input < Low )
		result = Low;
	else
		result = input;
	return result;
}
	

//·Ö¶ÎP
float Subsection_PID(uint8_t flag , float raw , uint8_t num , float * K , float * Scale)
{
	uint8_t cnt = 0;
	
	if( flag == 0 )
	{
		return raw;
	}
	
	for( cnt = 0;cnt<num-1;cnt++ )
	{
		if( ( abs(raw) < (*(Scale + cnt + 1)) )&&( abs(raw) >= (*(Scale + cnt)) ) )
		{
			break;
		}
	}
	
	
	return ( (*(K+cnt)) * raw );
}