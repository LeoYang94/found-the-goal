#ifndef _BKP_DATA_H
#define _BKP_DATA_H

#include "stm32f4xx.h"
#include "sys.h"
#include "stm32f4xx_rtc.h"


extern uint32_t BKP_ADR[20];
extern uint8_t Recovery_flag;

void BKP_INIT(void);//初始化RTC和PWR时钟
void WriteDataToBKP(uint32_t RTC_BKP_DR, uint32_t Data);//向备份区写数据
uint32_t ReadDataFromBKP(uint32_t RTC_BKP_DR);//从备份区读数据
void ForceToResetBKP(void);//清除备份区

void Recovery_6050(uint8_t mode);

#endif