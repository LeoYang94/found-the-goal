#include "Remote.h"
#include "Holder.h"
#include "math.h"
#include "ramp.h"
#include "Chassis_Control.h"
#include "All_Config.h"
#include "OtherConfig.h"


uint8_t IsComputerControl =  0;//控制模式  1：电脑模式 0：手柄模式


unsigned char sbus_rx_buffer[18];
RC_Ctl_t RC_Ctl;


#define BSP_USART1_DMA_RX_BUF_LEN	30u

float my_min( float a , float b )
{
	if( a > b )
	{
		return b;
	}
	else
		return a;
}

float my_max( float a , float b )
{
	if( a > b )
		return a;
	else
		return b;
}


void Remote_Config(void)
{
		USART_InitTypeDef USART1_InitStructure;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef   dma;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);
		
		gpio.GPIO_Pin = GPIO_Pin_7 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);
			
		USART_DeInit(USART1);
		USART1_InitStructure.USART_BaudRate = 100000;   //D-BUS 100K baudrate
		USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART1_InitStructure.USART_StopBits = USART_StopBits_1;
		USART1_InitStructure.USART_Parity = USART_Parity_Even;
		USART1_InitStructure.USART_Mode = USART_Mode_Rx;
		USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1,&USART1_InitStructure);
		
		//USART1_FIFO_Init();
    
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    
    nvic.NVIC_IRQChannel = USART1_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
		nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
    
    DMA_DeInit(DMA2_Stream5);
    DMA_StructInit(&dma);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = 30;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5,&dma);

//    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream5,ENABLE);
		
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
    USART_Cmd(USART1, ENABLE);
}



void USART1_IRQHandler(void)
{
	float angle_plus;
	float pitch_plus;
	
static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART1->SR;
		(void)USART1->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream5) == 0)
		{
			DMA_Cmd(DMA2_Stream5, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream5, ENABLE);
      if(this_time_rx_len == 18)
			{
				RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
				RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
				RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) |(sbus_rx_buffer[4] << 10)) & 0x07ff; //!< Channel 2
				RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
				RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
				RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
				RC_Ctl.mouse.x = -(sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8)); //!< Mouse X axis
				RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
				RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
				RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
				RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
				RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); //!< KeyBoard value
			
				
				
//				if(IsComputerControl)
//				{
//					ComputerControl();

//					yaw_Hold_Info.can_angle = Chassis_Control_Info.Chassis_angle;
//					
//					/*yaw 向左*/
//					if( RC_Ctl.mouse.x > 0 )
//					{
//						if(yaw_Hold_Info.can_angle < LEFT_LIMINT_ANGLE)
//						{
//							yaw_Hold_Info.angle_temp = yaw_Hold_Info.angle_target + ( RC_Ctl.mouse.x ) * YAW_SENSITY ;//鼠标yaw轴灵敏度
//						}
//					}
//					else if( RC_Ctl.mouse.x < 0 )		/*yaw 向右*/
//					{
//						if(yaw_Hold_Info.can_angle > RIGHT_LIMINT_ANGLE)
//						{
//							yaw_Hold_Info.angle_temp = yaw_Hold_Info.angle_target + ( RC_Ctl.mouse.x ) * YAW_SENSITY ;
//						}
//					}
//		
//					yaw_Hold_Info.angle_target = yaw_Hold_Info.angle_temp ;
//			
//					
//					
//					
//					/**********计算pitch********/
////					Pitch_Hold_Info.angle_temp = Pitch_Hold_Info.angle_target + ( RC_Ctl.mouse.y ) * PITCH_SENSITY;
////					
////					if( Pitch_Hold_Info.angle_temp > DOWN_LIMIT_ANGLE)
////					{
////						Pitch_Hold_Info.angle_temp = DOWN_LIMIT_ANGLE;
////					}
////					else if( Pitch_Hold_Info.angle_temp < UP_LIMIT_ANGLE)
////					{
////						Pitch_Hold_Info.angle_temp = UP_LIMIT_ANGLE;
////					}
////					Pitch_Hold_Info.angle_target = Pitch_Hold_Info.angle_temp ;
//	
//					if( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6025) 
//					{
//						pitch_plus = ( RC_Ctl.mouse.y ) * PITCH_SENSITY ;
//					}
//					else
//					{
//						pitch_plus = -( RC_Ctl.mouse.y ) * PITCH_SENSITY ;
//					}
//					
//					
//					if( pitch_plus > 0 )
//					{
//						angle_plus = my_min(  pitch_plus, (PITCH_POSITIVE_LIMIT_ANGLE - Pitch_Hold_Info.can_angle) );
//						
//						if( angle_plus < 0 )
//						{
//							angle_plus = 0;
//						}	
//						Pitch_Hold_Info.angle_temp = Pitch_Hold_Info.angle_target + angle_plus;
//						
//					}
//					
//					
//					else if( pitch_plus < 0 )
//					{
//						angle_plus = my_max( pitch_plus , (PITCH_NEGTIVE_LIMIT_ANGLE - Pitch_Hold_Info.can_angle) );
//						
//						if( angle_plus > 0 )
//						{
//							angle_plus = 0;
//						}	
//						
//						Pitch_Hold_Info.angle_temp = Pitch_Hold_Info.angle_target + angle_plus;
//					}
//					
//					Pitch_Hold_Info.angle_target = Pitch_Hold_Info.angle_temp ;
//					
//					
//				}
//			
//				/*手动模式*/
//				else 
//				{
//						//1.处理得到控制云台角度
//					//pitch	
//					if( LaserAndPrep[TANK_SERIAL_NUMBER-1][9] == MOTOR_6025) 
//					{
//						Pitch_Hold_Info.angle_temp = Pitch_Hold_Info.angle_target + (-1024 + RC_Ctl.rc.ch3 ) * 0.0035f;//pitch 遥控器输入转换成角度
//					}
//					else
//					{
//						Pitch_Hold_Info.angle_temp = Pitch_Hold_Info.angle_target - (-1024 + RC_Ctl.rc.ch3 ) * 0.0015f;//pitch 遥控器输入转换成角度
//					}
//					
//				
//					if( Pitch_Hold_Info.angle_temp > PITCH_POSITIVE_LIMIT_ANGLE)
//					{
//						Pitch_Hold_Info.angle_temp = PITCH_POSITIVE_LIMIT_ANGLE;
//					}
//					else if( Pitch_Hold_Info.angle_temp < PITCH_NEGTIVE_LIMIT_ANGLE)
//					{
//						Pitch_Hold_Info.angle_temp = PITCH_NEGTIVE_LIMIT_ANGLE;
//					}
//	
//					Pitch_Hold_Info.angle_target = Pitch_Hold_Info.angle_temp ;	
//					
//					//yaw
//					yaw_Hold_Info.angle_temp = yaw_Hold_Info.angle_target + ( 1024 - RC_Ctl.rc.ch2 ) * 0.0035f;//测试视觉时，注释此举
//					
////					if( yaw_Hold_Info.angle_temp > YAW_POSITIVE_LIMIT_ANGLE)
////					{
////						yaw_Hold_Info.angle_temp = YAW_POSITIVE_LIMIT_ANGLE;
////					}
////					
////					else if( yaw_Hold_Info.angle_temp < YAW_NEGTIVE_LIMIT_ANGLE)
////					{
////						yaw_Hold_Info.angle_temp = YAW_NEGTIVE_LIMIT_ANGLE;
////					}
//					
//					yaw_Hold_Info.angle_target = yaw_Hold_Info.angle_temp ;
//				}
//				
//			}
//			else
//			{
//				
//			}

		}
		
				
			
	}
	}       
}



uint8_t In_range( float low , float high , float var )
{
	if( (var >= low)&&(var <= high) )
		return 1;
	else
		return 0;
}

