
#include "USART3.h"
#include "math.h"
/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/


_PARA_REV Para_s_rev;

uint8_t para_rev[PARA_REV_NUM];
uint8_t judgment_data[PARA_REV_NUM];
_JUDGMENT_01_DATA Judgment_01_data;
_JUDGMENT_03_DATA Judgment_03_data;
//WAU:where are u?
uint8_t point_flag,WAU=0;
void uart2_Rstart_dma(void);

//CRC start
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

//over


float Send_data[4];

void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
		
		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);
		
		usart3.USART_BaudRate = 115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		USART_Cmd(USART3,ENABLE);
    
    nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)para_rev;//(uint32_t)EnemyData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = PARA_REV_NUM;//6;
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

    DMA_Cmd(DMA1_Stream1,ENABLE);
}

void uart2_Rstart_dma(void)
{  
//	DMA_InitTypeDef       DMA_InitStructure;
//	DMA_InitStructure.DMA_BufferSize = 100;            
//	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//	
//	DMA_Cmd(DMA2_Stream0, ENABLE);         
	uint16_t x;
	DMA1_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN;//失能dma
	DMA1_Stream1->M0AR =(uint32_t)para_rev;
	DMA1_Stream1->NDTR = PARA_REV_NUM;
	
	for( x = 0;x< PARA_REV_NUM;x++)
	{
		para_rev[x] = 0;
	}
	Para_s_rev.data_num_last = 0;
	Para_s_rev.data_num_now = 0;
	Para_s_rev.rev_overtime_cnt = 0;
	Para_s_rev.rev_state = 0;
	
	DMA1_Stream1->CR |= (uint32_t)DMA_SxCR_EN;//使能dma
  
}

uint8_t Detect_para_rev_dma(uint16_t cnt , struct PID_PARA *PARA_INDEX)
{
	Para_s_rev.data_num_now = PARA_REV_NUM - DMA_SxNDT_1;//??para????????
	/*
	state:0->为开始接收 1->正在接收 2->接受完成 
	*/
	if( Para_s_rev.rev_state == 0 )//??????
	{
		if(  Para_s_rev.data_num_now > 0 )
		{
			Para_s_rev.rev_state = 1;
			Para_s_rev.data_num_last = Para_s_rev.data_num_now;
			
		}
	}
	else if( Para_s_rev.rev_state == 1 )//??????
	{
		if (Para_s_rev.data_num_now == Para_s_rev.data_num_last)//??????
		{
			Para_s_rev.rev_overtime_cnt ++;
			if( Para_s_rev.rev_overtime_cnt >= cnt )//??????
			{
				Para_s_rev.rev_state = 2;
			}
			Para_s_rev.data_num_last = Para_s_rev.data_num_now;
		}
		else if( Para_s_rev.data_num_now > Para_s_rev.data_num_last )
		{
			Para_s_rev.rev_overtime_cnt = 0;
			Para_s_rev.data_num_last = Para_s_rev.data_num_now;
		}
	}

	
	if( Para_s_rev.rev_state ==2 )
	{
		//Updata_PID(PARA_INDEX);
		
		uart2_Rstart_dma();//??DMA??
	}
	else
	{
		
	}
	return 1;
}

uint8_t Updata_PID(uint8_t *judgment_data)
{	
uint8_t i;
	/*
	if( (para_rev[0]==0x8a)&&(para_rev[1]==0x8b)&&(para_rev[2]==0x1c)&&(para_rev[3]==0xae) )
	{
		
		(*PARA_INDEX).core_P = para_rev[4] * 256 + para_rev[5];
		(*PARA_INDEX).core_I = para_rev[6] * 256 + para_rev[7];
		(*PARA_INDEX).core_D = para_rev[8] * 256 + para_rev[9];
		
		(*PARA_INDEX).shell_P = para_rev[10] * 256 + para_rev[11];
		(*PARA_INDEX).shell_I = para_rev[12] * 256 + para_rev[13];
		(*PARA_INDEX).shell_D = para_rev[14] * 256 + para_rev[15];
		
		
	}
	*/

	return 1;
}

void uart3_send_multi_data(float *data)
{
	uint16_t x;
	uint16_t i;
	uint8_t sum;
	
	uint8_t data_buff[32];
	
	data_buff[0] = 0x88;
	data_buff[1] = 0xa1;
	data_buff[2] = 28;
	
	for( i=0;i<4;i++ )
	{
		data_buff[i*4+0+3] = BYTE3(*(data+i));
		data_buff[i*4+1+3] = BYTE2(*(data+i));
		data_buff[i*4+2+3] = BYTE1(*(data+i));
		data_buff[i*4+3+3] = BYTE0(*(data+i));
	}
	
	for( x=0;x<31;x++ )
	{
		sum = sum + data_buff[x];
	}
	
	data_buff[31] = sum;
	
	
	for( x = 0;x<32;x++ )
	{
		while ((USART3->SR & USART_FLAG_TC) == (uint16_t)RESET);
		USART_SendData(USART3,data_buff[x]);
	}
	
}

void uart_putchar(uint8_t data)
{
		while ((USART1->SR & USART_FLAG_TC) == (uint16_t)RESET);
		USART_SendData(USART1,data);
	  while ((USART1->SR & USART_FLAG_TC) == (uint16_t)RESET);
}

uint8_t get_judgment_data(uint8_t flag)
{
	if(flag==1)
	{
		while(WAU<=BUFFERSIZE)
		{
		if(para_rev[WAU]==0xA5)
		{
			if(((para_rev[WAU+4]+(para_rev[WAU+5]>>8))==0x01) & ((para_rev[WAU+1]+(para_rev[WAU+2]>>8))==0x25))
			{
				GetDataFrame(1);
				
			//Judgment_data.left_time:在比赛倒计时三分钟时才会有值
			Judgment_01_data.left_time = judgment_data[6]+(judgment_data[7]<<8)+(judgment_data[8]<<16)+(judgment_data[9]<<24);
			
			Judgment_01_data.left_HP = judgment_data[10]+(judgment_data[11]<<8);

			BYTE0(Judgment_01_data.voltage_V) = judgment_data[12];
			BYTE1(Judgment_01_data.voltage_V) = judgment_data[13];
			BYTE2(Judgment_01_data.voltage_V) = judgment_data[14];
			BYTE3(Judgment_01_data.voltage_V) = judgment_data[15];
			
			BYTE0(Judgment_01_data.current_A) = judgment_data[16];
			BYTE1(Judgment_01_data.current_A) = judgment_data[17];
			BYTE2(Judgment_01_data.current_A) = judgment_data[18];
			BYTE3(Judgment_01_data.current_A) = judgment_data[19];
			
			Judgment_01_data.power_W = Judgment_01_data.current_A * Judgment_01_data.voltage_V;
			
			break;
		}else if(((para_rev[WAU+4]+(para_rev[WAU+5]>>8))==0x02) & ((para_rev[WAU+1]+(para_rev[WAU+2]>>8))==0x03))
		{
			
			break;
		}else if(((para_rev[WAU+4]+(para_rev[WAU+5]>>8))==0x03) & ((para_rev[WAU+1]+(para_rev[WAU+2]>>8))==0x10))
		{
			
			GetDataFrame(3);
			
			BYTE0(Judgment_03_data.small_bullet_speed) = judgment_data[0];
			BYTE1(Judgment_03_data.small_bullet_speed) = judgment_data[1];
			BYTE2(Judgment_03_data.small_bullet_speed) = judgment_data[2];
			BYTE3(Judgment_03_data.small_bullet_speed) = judgment_data[3];
			
			BYTE0(Judgment_03_data.small_bulet_frequency) = judgment_data[4];
			BYTE1(Judgment_03_data.small_bulet_frequency) = judgment_data[5];
			BYTE2(Judgment_03_data.small_bulet_frequency) = judgment_data[6];
			BYTE3(Judgment_03_data.small_bulet_frequency) = judgment_data[7];
			
			BYTE0(Judgment_03_data.big_bullet_speed) = judgment_data[8];
			BYTE1(Judgment_03_data.big_bullet_speed) = judgment_data[9];
			BYTE2(Judgment_03_data.big_bullet_speed) = judgment_data[10];
			BYTE3(Judgment_03_data.big_bullet_speed) = judgment_data[11];
			
			BYTE0(Judgment_03_data.big_bulet_frequency) = judgment_data[12];
			BYTE1(Judgment_03_data.big_bulet_frequency) = judgment_data[13];
			BYTE2(Judgment_03_data.big_bulet_frequency) = judgment_data[14];
			BYTE3(Judgment_03_data.big_bulet_frequency) = judgment_data[15];
			
			break;
		}else
		{
			
		}
	}
		WAU++;
		if(WAU>=BUFFERSIZE)
			WAU=0;
		#if TEST_JUDGENTMENT
		Send_data[0] = Judgment_01_data.voltage_V;
		Send_data[1] = Judgment_01_data.current_A;
		Send_data[2] = Judgment_01_data.power_W;
		Send_data[3] = Judgment_01_data.left_HP;
		#endif
		
		}
	}else
	{
		return 0;
	}
}
void GetDataFrame(uint8_t DataFrameID)
{
	uint8_t data_length;
	uint8_t j;
	//WAU=point_flag;
	switch(DataFrameID)
	{
		case 1:{
			data_length=DATALENGTH_01;
						break;}
		case 2:{
			data_length=DATALENGTH_02;
						break;}
		case 3:{
			data_length=DATALENGTH_03;
						break;}
	}
	if((WAU+data_length)<=BUFFERSIZE)
	{
		for(j=0;j<data_length;j++)
		{
			judgment_data[j] = para_rev[WAU+j];
		}
		//point_flag = WAU+data_length;
	}else
	{
		for(j=0;(WAU+j)<BUFFERSIZE;j++)
		{
			judgment_data[j]=para_rev[WAU+j];	
		}
		for(j=0;j<(WAU+(data_length-BUFFERSIZE));j++)
		{
			judgment_data[BUFFERSIZE-WAU+j] = para_rev[j];
		}
		//point_flag = WAU+(data_length-BUFFERSIZE);
	}
}
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
		(uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return 0;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff)== pchMessage[dwLength - 1]);
}


