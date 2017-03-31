#include "referee.h"
#include "math.h"
//#include "Define.h"
/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
#include "string.h"
#include "Holder.h"
#include "Remote.h"
#include "All_Config.h"
#include "auto_shoot.h"
uint8_t met_data[BSP_USART3_DMA_RX_BUF_LEN];

_JUDGMENT_01_DATA Judgment_01_data; 
_JUDGMENT_02_DATA Judgment_02_data;
_JUDGMENT_03_DATA Judgment_03_data;

uint16_t re_data[9];
uint8_t check_flag=0;

//œ‡ª˙ƒ£ Ω≤Œ ˝  	`
float test_count1=0;
float test_count2=0;
float yaw_high=0.32;//yaw◊Ó∏ﬂÀŸ∂»
float pitch_high=0.03;//pitch◊Ó∏ﬂÀŸ∂»
float yaw_b=-0.0425;//∫Ø ˝÷–bµƒ÷µ
float pitch_dev=5,yaw_dev=6;
int camera_flag=1;

extern struct Hold_Info Pitch_Hold_Info;
extern struct Hold_Info yaw_Hold_Info;
//WAU:where are u?
float temp_V,temp_A,temp_W;
#define TX_LEN 100
uint8_t Tx_Buf[TX_LEN];
uint8_t Flag_Uart_Busy=0;

float Send_data[4];
float send_anto_data[4];

void Referee_Configuration(void)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  NVIC_InitStructure;
		DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		gpio.GPIO_Mode  = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd  = GPIO_PuPd_UP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_Pin   = GPIO_Pin_10;
		GPIO_Init(GPIOB, &gpio);
						
		gpio.GPIO_Pin   = GPIO_Pin_11;
		GPIO_Init(GPIOB, &gpio);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);        
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
		
		usart3.USART_BaudRate = 115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    
		USART_Cmd(USART3,ENABLE);
				
//		memset(met_data, 0xff, sizeof(met_data));
			
		DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)met_data;//(uint32_t)EnemyData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = BSP_USART3_DMA_RX_BUF_LEN;//100;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&DMA_InitStructure);
    
    DMA_DeInit(DMA1_Stream3);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf;//send buffer
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = TX_LEN; //8
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
    
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //TIM3÷–∂œ
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //œ»’º”≈œ»º∂0º∂
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //¥””≈œ»º∂3º∂
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÕ®µ¿±ª πƒ‹
		NVIC_Init(&NVIC_InitStructure);  //≥ı ºªØNVICºƒ¥Ê∆˜
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    USART_ITConfig( USART3 , USART_IT_IDLE , ENABLE );//ø’œ–÷–∂œ
    DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);//DMA∑¢ÀÕ÷–∂œ
    
    DMA_Cmd(DMA1_Stream1,ENABLE);//RX		 
    DMA_Cmd(DMA1_Stream3,ENABLE);//TX
    
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    
		USART_Cmd(USART3,ENABLE);

}
void DMA1_Stream3_IRQHandler(void)
{
  
  DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
  DMA_Cmd(DMA1_Stream3,DISABLE);
  Flag_Uart_Busy = 0;
  
}
void New_Send_Data(uint8_t *data,uint16_t size)
{
  while(Flag_Uart_Busy);
  
  Flag_Uart_Busy = 1;
  
  memcpy(Tx_Buf,data,size); 
  
  DMA_SetCurrDataCounter(DMA1_Stream3,size);
  
  DMA_Cmd(DMA1_Stream3,ENABLE); 
}

void send_odm_msg(float * data)//??????????????,??????????±8191,????±1800,???????˜19,˜100;
{ 
	int i=0;
	//PID_out;
  uint8_t uart3_send_buff[32]; 
	uint8_t sum=0;
	uart3_send_buff[0] = 0xAA;
	uart3_send_buff[1] = 0xAA;
	uart3_send_buff[2] = 0xF1;
	uart3_send_buff[3] = 16;
	for(i=0;i<4;i++) 
	{
		uart3_send_buff[i*4+0+4] = BYTE3(*(data+i));
		uart3_send_buff[i*4+1+4] = BYTE2(*(data+i));
		uart3_send_buff[i*4+2+4] = BYTE1(*(data+i));
		uart3_send_buff[i*4+3+4] = BYTE0(*(data+i));
	}
	for(i = 0; i<20; i++)
		{
			sum +=uart3_send_buff[i];
		}
		uart3_send_buff[20] = sum;	
	
	//Uart3_Put_Buf(uart3_send_buff , 21);
    New_Send_Data(uart3_send_buff,21);
		
}
void send_check(uint16_t data)
{
  uint8_t uart3_send_buff[32]; 
	uint16_t sum=0,i;
	uart3_send_buff[0] = 0xAA;
	uart3_send_buff[1] = 0xAA;
	uart3_send_buff[2] = 0xEF;
	uart3_send_buff[3] = 2;
  uart3_send_buff[4] = 0x10;
  uart3_send_buff[5] = data;
  
  for(i=0;i<6;i++)
  {
    sum+=uart3_send_buff[i];
  }
  uart3_send_buff[6] = sum;
  New_Send_Data(uart3_send_buff,7);
}

void USART3_IRQHandler(void)
{
  static uint32_t this_time_rx_len = 0;
  uint8_t i=0;
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART3->SR;
		(void)USART3->DR;
       
   if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
		{
			DMA_Cmd(DMA1_Stream1, DISABLE);
    
			this_time_rx_len = BSP_USART3_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
			DMA1_Stream1->NDTR = (uint16_t)BSP_USART3_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			//DMA1_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream1, ENABLE);
		
      for(i=0;i<14;i++)
		{
			if((met_data[i]==0xAA)&&(met_data[i+1]==0xab)&&(met_data[i+6]==0xac))//∑¢¿¥µƒ ˝÷µŒ™0-200£¨ªªÀ„≥…Œ™-100~100
		 {
			 test_count1+=1;
			 Pitch_Hold_Info.receive_speed_raw=met_data[3]-100;
			 yaw_Hold_Info.receive_speed_raw=met_data[5]-100;
			 
//∑÷œÒÀÿµ„£¨º”œµ ˝∑Ω Ω
//yaw¥¶¿Ì£¨ªØŒ™∫Ø ˝c
      if((yaw_Hold_Info.receive_speed_raw>=90)&&(yaw_Hold_Info.receive_speed_raw<=100))
			{
				yaw_Hold_Info.receive_speed=yaw_high;
      }
			else if((yaw_Hold_Info.receive_speed_raw>=-100)&&(yaw_Hold_Info.receive_speed_raw<=-90))
			{
				yaw_Hold_Info.receive_speed=-yaw_high;
      }
			else if((yaw_Hold_Info.receive_speed_raw<=10)&&(yaw_Hold_Info.receive_speed_raw>=-10))
			{
				yaw_Hold_Info.receive_speed=0;
				
      }
			else if((yaw_Hold_Info.receive_speed_raw<90)&&(yaw_Hold_Info.receive_speed_raw>10))
			{
				yaw_Hold_Info.receive_speed=k_yaw*yaw_Hold_Info.receive_speed_raw+yaw_b;
      }
			else if((yaw_Hold_Info.receive_speed_raw<-10)&&(yaw_Hold_Info.receive_speed_raw>-90))
			{ 
				yaw_Hold_Info.receive_speed=k_yaw*yaw_Hold_Info.receive_speed_raw-yaw_b;
      }
			//pitch¥¶¿Ì
			if((Pitch_Hold_Info.receive_speed_raw<100)&&(Pitch_Hold_Info.receive_speed_raw>10))
			{
				Pitch_Hold_Info.receive_speed=0.02;
				camera_flag=0;
      }
			else if((Pitch_Hold_Info.receive_speed_raw>=-100)&&(Pitch_Hold_Info.receive_speed_raw<-10))
			{
				Pitch_Hold_Info.receive_speed=-0.02;
				camera_flag=0;
      }
			else if((Pitch_Hold_Info.receive_speed_raw<=10)&&(Pitch_Hold_Info.receive_speed_raw>=-10))
			{
				Pitch_Hold_Info.receive_speed=0;
				camera_flag=0;
      }
		  else if(Pitch_Hold_Info.receive_speed_raw==150)
			 {
				 camera_flag=1;
       }

		
}
}
}
}
}
//‘⁄÷˜∫Ø ˝µ˜”√
void Send_CheckData(void)
{
  if(check_flag)
  {
    send_check(check_flag);
    check_flag=0;
  }
}
	
void camera_receive(void)
{
		if(camera_flag==1)
		{
			go_back();
    }
		else if(camera_flag==0)
		{
			auto_shoot_mode();
}
}
void auto_shoot_mode(void)//◊‘∂Ø…‰ª˜ƒ£ Ω
{
		yaw_Hold_Info.angle_temp=yaw_Hold_Info.angle_target+yaw_Hold_Info.receive_speed/yaw_dev;
	  Pitch_Hold_Info.angle_temp=Pitch_Hold_Info.angle_target+Pitch_Hold_Info.receive_speed/pitch_dev;
	 if( Pitch_Hold_Info.angle_temp > pitch_limit_under)
			{
						Pitch_Hold_Info.angle_temp = pitch_limit_under;
			}
	 else if( Pitch_Hold_Info.angle_temp < pitch_limit_on)
			{
						Pitch_Hold_Info.angle_temp = pitch_limit_on;
			}
		if(yaw_Hold_Info.angle_temp>yaw_limit_left)
		{
         yaw_Hold_Info.angle_temp	=	yaw_limit_left;	
    }
		else if(yaw_Hold_Info.angle_temp<yaw_limit_right)
		{
			  yaw_Hold_Info.angle_temp=yaw_limit_right;
    }
		Pitch_Hold_Info.angle_target=Pitch_Hold_Info.angle_temp;
		yaw_Hold_Info.angle_target=yaw_Hold_Info.angle_temp;

}


 