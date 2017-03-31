#ifndef __PREPARE_DATA_H_
#define __PREPARE_DATA_H_
#include<stdint.h>
#include "stm32f4xx.h" 


#define abs(x) ((x)>0? (x):(-(x)))

#define GYRO_GAP 30


void Prepare_data(void);
void PrepareForIMU(uint8_t flag);
/*����˫�� ���ٶ� �Ƕ� can��������Ϣ*/
void Attitude_update(void);

extern uint8_t mpu6050_error_flag;

#endif


