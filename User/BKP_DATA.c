#include "BKP_DATA.h"
#include "6050.h"
uint32_t BKP_ADR[20]={RTC_BKP_DR0,
											RTC_BKP_DR1,
											RTC_BKP_DR2,
											RTC_BKP_DR3,
											RTC_BKP_DR4,
											RTC_BKP_DR5,
											RTC_BKP_DR6,
											RTC_BKP_DR7,
											RTC_BKP_DR8,
											RTC_BKP_DR9,
											RTC_BKP_DR10,
											RTC_BKP_DR11,
											RTC_BKP_DR12,
											RTC_BKP_DR13,
											RTC_BKP_DR14,
											RTC_BKP_DR15,
											RTC_BKP_DR16,
											RTC_BKP_DR17,
											RTC_BKP_DR18,
											RTC_BKP_DR19
};

uint8_t Recovery_flag=0;
//初始化时钟
void BKP_INIT(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
}
//写数据
void WriteDataToBKP(uint32_t RTC_BKP_DR, uint32_t Data)
{
	//关闭写保护
	PWR_BackupAccessCmd(ENABLE);
	RTC_WriteProtectionCmd(DISABLE);
	
	RTC_WriteBackupRegister(RTC_BKP_DR,Data);
	//开启写保护
	RTC_WriteProtectionCmd(ENABLE);	
	PWR_BackupAccessCmd(DISABLE);	
}

//读数据

uint32_t ReadDataFromBKP(uint32_t RTC_BKP_DR)
{
	return RTC_ReadBackupRegister(RTC_BKP_DR);
}

//清空备份区
void ForceToResetBKP(void)
{

	PWR_BackupAccessCmd(ENABLE);
	RTC_WriteProtectionCmd(DISABLE);
	
	#if 1
	RCC->BDCR|=1<<16; 
	RCC->BDCR&=~(1<<16);
	#else
	RCC_BackupResetCmd(ENABLE);//复位整个备份域
	RCC_BackupResetCmd(DISABLE);
	#endif
	
	RTC_WriteProtectionCmd(ENABLE);	
	PWR_BackupAccessCmd(DISABLE);	
}

//mode:0->读取后不清除备份区
//     1->读取后清除备份区
void Recovery_6050(uint8_t mode)
{
		sensor.gyro.quiet.x = (uint16_t)ReadDataFromBKP(BKP_ADR[0]);
		sensor.gyro.quiet.y = (uint16_t)ReadDataFromBKP(BKP_ADR[1]);
		sensor.gyro.quiet.z = (uint16_t)ReadDataFromBKP(BKP_ADR[2]);
	if(mode == 0)
	{
	}
	else
	{
		ForceToResetBKP();
	}
	if(sensor.gyro.quiet.x == 0 && sensor.gyro.quiet.y == 0 && sensor.gyro.quiet.z == 0)
	{
		Recovery_flag = 0;//表示开机读取
	}else
	{
		Recovery_flag = 1;
	}
}