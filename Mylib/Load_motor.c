#include "Load_motor.h"
#include "Holder.h"
#include "Shoot.h"
#include "Remote.h"
#include "OtherConfig.h"
#include "delay.h"

void Load_Motor_Error_Out(void);


struct PID_PARA Load_motor_para_Mainfold = 
{
	150,0,0,25,0,0
};
struct PID_PARA Load_motor_para_Normal=
{
	55,0,0,0,0,0
};


	
void Load_Motor_PWM_Configuration(void)//PA2  TIM9 CH3
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM9ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9); //GPIOA2����Ϊ��ʱ��2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);    
	
	/*�ٷ�����******************************/
	if( LaserAndPrep[TANK_SERIAL_NUMBER-1][5] == 0 )
	{
		TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
		TIM_TimeBaseStructure.TIM_Period=40000-1;   //�Զ���װ��ֵ
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
		
		TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
		
			//��ʼ��TIM2 Channel3 PWMģʽ	 
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
		TIM_OCInitStructure.TIM_Pulse=40000;
	}
	
	/*��������************************/
	else if( LaserAndPrep[TANK_SERIAL_NUMBER-1][5] == 1  )
	{
		TIM_TimeBaseStructure.TIM_Prescaler=21-1;  //��ʱ����Ƶ
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
		TIM_TimeBaseStructure.TIM_Period=1000;   //�Զ���װ��ֵ
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
		
		TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
		
			//��ʼ��TIM2 Channel3 PWMģʽ	 
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
		TIM_OCInitStructure.TIM_Pulse=1000;
		
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
		
	}

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC3
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR3�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM9, ENABLE);  //ʹ��TIM9

	
	
	
}

//TIM3 connect to encoder,a-b
//TIM3_CH1 ----- PB4
//TIM3_CH2 ----- PB5

void Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		TIM_ICInitTypeDef TIM_ICInitStructure; 
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_TIM3);
	
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
		TIM_SetCounter(TIM3,0);
    TIM_Cmd(TIM3, ENABLE);
}


void Load_motor_control(uint8_t flag , uint8_t type)
{
	float shell_delta,shell_p_part ;
	
	float delta, p_part ,sum;

	int8_t i;
	
	#if LOAD_MOTOR_DETECT_OR_NOT	
	if( Shoot_Info.Load_motor_error == 1 )
	{
		Load_Motor_Error_Out();
		
		return;
	}
	#endif
	
	
	
	
	
	
	Shoot_Info.speed = -(int16_t)(TIM_GetCounter(TIM3));
	TIM_SetCounter(TIM3,0);
	
	if( LaserAndPrep[TANK_SERIAL_NUMBER-1][4] == 1 )//���Ƶ��
	{
		Shoot_Info.speed_buff[Shoot_Info.speed_filter_cnt]= Shoot_Info.speed * 100.0f / 520.0f;//ת�����ٶ�  r/s  5200Ϊһת5200������
	}
	else if( LaserAndPrep[TANK_SERIAL_NUMBER-1][4] == 0 )//�ٷ����
	{
		Shoot_Info.speed_buff[Shoot_Info.speed_filter_cnt]= Shoot_Info.speed * 100.0f / 5200.0f;//ת�����ٶ�  r/s  5200Ϊһת5200������
	}
	
	
	sum = 0;
	for( i=0;i<5;i++ )
	{
		sum += Shoot_Info.speed_buff[i];
	}
	Shoot_Info.Load_Motor_Speed = sum / 5.0f;
	Shoot_Info.speed_filter_cnt =  ( Shoot_Info.speed_filter_cnt + 1 ) % 5;
	
	//////////////////��������
	if( LaserAndPrep[TANK_SERIAL_NUMBER-1][5] == 1 )
	{
		//position circle
		if( type==1 )
		{
			Shoot_Info.pulse_cnt += Shoot_Info.speed ;
		
			shell_delta = Shoot_Info.pulse_target - Shoot_Info.pulse_cnt;
				
			Shoot_Info.shell_out = shell_delta * Load_motor_para_Mainfold.core_P / 1000.0f;//0.002f;
			
			if( Shoot_Info.shell_out > 0.7f )//���Ƶ���ٶ�
			{Shoot_Info.shell_out = 0.7f;}
			
			delta = Shoot_Info.shell_out - Shoot_Info.Load_Motor_Speed;
		}
		
		//speed circle
		else
		{
			delta = Shoot_Info.Load_Motor_Speed_Target - Shoot_Info.Load_Motor_Speed;
		}

		p_part = delta * 5.0f * Load_motor_para_Mainfold.shell_P;//Load_motor_para.shell_P;
		
		if( p_part <0  )
		{
			p_part = 0;
		}
		if( p_part > 600 )
		{
			p_part = 600;
		}

		
		Shoot_Info.Load_Motor_Out = p_part;
		

		//�ٶȻ����
		if(type == 0)
		{
			//����ģʽ
			if(  IsComputerControl == 1)
			{
				if( (Shoot_Info.load_command  == 1)&&(RC_Ctl.mouse.press_l == 1) )
				{
					Load_motor_out(Shoot_Info.Load_Motor_Out);
				}			
				else
				{
					Load_motor_out(0);
				}
			}
			
			//ң��ģʽ
			else
			{
				if( (Shoot_Info.load_command  == 1)&&(RC_Ctl.rc.s1 == 1) )
				{
					Load_motor_out(Shoot_Info.Load_Motor_Out);
				}			
				else
				{
					Load_motor_out(0);
				}
			}
			
		}
		
		//λ�û�
		else 
		{
			Load_motor_out(Shoot_Info.Load_Motor_Out);
		}

	}
	///////////////////////////�ٷ�����
	else if( LaserAndPrep[TANK_SERIAL_NUMBER-1][5] == 0 )
	{
		delta = Shoot_Info.Load_Motor_Speed_Target - Shoot_Info.Load_Motor_Speed;

		p_part = delta * 200.0f * Load_motor_para_Normal.shell_P;//Load_motor_para.shell_P;
		
		if( p_part <40  )
		{
			p_part = 40;
		}
		if( p_part > 10000 )
		{
			p_part = 10000;
		}

		
		Shoot_Info.Load_Motor_Out = p_part;
		
		if (IsComputerControl == 1)//����ģʽ
		{
			if( (flag == 1)&&( (Shoot_Info.load_command  == 1)&&(RC_Ctl.mouse.press_l == 1) ) )
			{
				Load_motor_out( 40000 - Shoot_Info.Load_Motor_Out );
			}
			else
			{
				Load_motor_out(40000);
			}
		}
	
		else//�ֱ�ģʽ
		{
			if( (flag == 1)&&( (Shoot_Info.load_command  == 1)&&(RC_Ctl.rc.s1 == 1) ) )//Shoot_Info.load_command  == 1
			{
				Load_motor_out( 40000 - Shoot_Info.Load_Motor_Out );
			}
			else
			{
				Load_motor_out(40000);
			}
		}
	}
		



	
	
}

void Bullet_Cap_Conf(void)//pb10
{
GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2????    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2); //GPIOA2??????2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOB,&GPIO_InitStructure);    
	
	TIM_TimeBaseStructure.TIM_Prescaler=8400-1;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=200;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  
 
  TIM_ARRPreloadConfig(TIM2,ENABLE);
	
	TIM_Cmd(TIM2, ENABLE);  

}

void Bullet_Cap_Close(void)
{
	TIM_SetCompare3(TIM2,LaserAndPrep[TANK_SERIAL_NUMBER-1][0]);
}

void Bullet_Cap_Open(void)
{
	TIM_SetCompare3(TIM2,LaserAndPrep[TANK_SERIAL_NUMBER-1][1]);
}

void Load_motor_out(uint32_t a)
{
	TIM_SetCompare1(TIM9,a);
}

/*
//������������
adds:
	��������ƶ����� Ĭ�ϣ�90

*/
void Load_Motor_position_plus(uint16_t adds)
{
	if( adds == 0 )
	{
		adds = 90 ;
	}
	
	Shoot_Info.pulse_cnt = Shoot_Info.pulse_target;
	Shoot_Info.pulse_target += adds;
}


void Load_Motor_Position_plus(uint32_t adds , uint8_t delay_s)
{
	uint8_t i;
	i = delay_s;
	 Shoot_Info.pulse_cnt = Shoot_Info.pulse_target;
	 Shoot_Info.pulse_target += adds;
	
	while( i-- )
	{
		delay_ms(1000);
	}
}


void Load_Motor_Fault_detet(void)
{
	static int32_t cnt = 0;
	
	if( Shoot_Info.Load_Motor_Speed < LOAD_MOTOR_SPEED_MIN )
	{
		cnt ++;
	}
	else
	{
		cnt = 0;
	}
	
	if( cnt >= LOAD_MOTOR_OVER_CNT )
	{
		cnt = 0;
		Shoot_Info.Load_motor_error = 1;
	}
	
}

void Load_Motor_Error_Out(void)
{
	//�ٷ�����
	if(  LaserAndPrep[TANK_SERIAL_NUMBER-1][5] == 0)
	{
		Load_motor_out(38000);
	}
	
	//��������
	else if( LaserAndPrep[TANK_SERIAL_NUMBER-1][5] == 1)
	{
		Load_motor_out(350);
	}
}


