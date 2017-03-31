#include "Motor_out.h"
#include "Holder.h"
//2016.4.17


void can2_send(int16_t speed)
{
	uint8_t TransmitMailbox;//发送信箱号
	CanTxMsg TxMessage;
	
	
	TxMessage.StdId = 0x1ff;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
	TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
	TxMessage.DLC=8;							 //数据长度为2字节
	
	TxMessage.Data[0]=speed >> 8;
  TxMessage.Data[1]=speed ; 
	

	TxMessage.Data[2]=speed >> 8;
	TxMessage.Data[3]=speed;
	
	TxMessage.Data[4]=0;
	TxMessage.Data[5]=0;
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;

	TransmitMailbox = CAN_Transmit(CAN2,&TxMessage);
	
	while(CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)
	{
		;
	}


}

int8_t Holder_Motor_output(int8_t flag )//云台电机CAN发送数据
{

	uint8_t c1,c2,c3,c4;
	
	int16_t yaw_out_s16;
  int16_t pitch_out_s16;
	CanTxMsg TxMessage;
	int16_t t;
	
  uint8_t TransmitMailbox;//发送信箱号
	
	TxMessage.StdId = 0x1ff;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
	TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
	TxMessage.DLC=8;							 //数据长度为2字节

	//转换到int16类型，方便后续CAN发送

	pitch_out_s16 = (int16_t)Pitch_Hold_Info.out;//PITCH电机速度环输出
	yaw_out_s16 = (int16_t)yaw_Hold_Info.out;//YAW电机速度环输出
	
	
	//Pitch轴限幅
	if( pitch_out_s16 < -PITCH_OUT_MAX)
	{
		pitch_out_s16 = -PITCH_OUT_MAX;
	}
	else if( pitch_out_s16 > PITCH_OUT_MAX )
	{
		pitch_out_s16 = PITCH_OUT_MAX;
	}
	
		//YAW幅
	if( yaw_out_s16 < -PITCH_OUT_MAX)
	{
		yaw_out_s16 = -PITCH_OUT_MAX;
	}
	else if( yaw_out_s16 > PITCH_OUT_MAX )
	{
		yaw_out_s16 = PITCH_OUT_MAX;
	}
	
	
	//yaw_out_s16 = 0;//**********************change********************************************
	//pitch_out_s16 = 0;
	
	
	if( flag == 1 )
	{
		c1 = (uint8_t)( pitch_out_s16 >> 8);
	
		c2	 = (uint8_t)pitch_out_s16;
		
		c3 = (uint8_t)( yaw_out_s16>> 8);
	
		c4	 = (uint8_t)yaw_out_s16;	
	}
	else
	{
		c1= 0 ;
		c2 = 0;
		c3 = 0;
		c4 = 0;
	}
	
	
  TxMessage.Data[0]=c3;//yaw
  TxMessage.Data[1]=c4; 
	

	TxMessage.Data[2]=c1;//pitch
	TxMessage.Data[3]=c2;
	
	TxMessage.Data[4]=0;
	TxMessage.Data[5]=0;
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;

	
	TransmitMailbox = CAN_Transmit(CAN2,&TxMessage);
	t=0;
	while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t<500))
	{
		t++;
		
	}

	return 1;

}
