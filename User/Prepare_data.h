#ifndef __PREPARE_DATA_H_
#define __PREPARE_DATA_H_
#include<stdint.h>
#include "stm32f4xx.h" 


#define abs(x) ((x)>0? (x):(-(x)))

#define GYRO_GAP 30


void Prepare_data(void);
void PrepareForIMU(uint8_t flag);
/*更新双轴 角速度 角度 can编码器信息*/
void Attitude_update(void);

extern uint8_t mpu6050_error_flag;

#endif


