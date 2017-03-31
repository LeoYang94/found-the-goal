#include "auto_shoot.h"

int flag_test=1;

void	go_back(void)
{
		if(yaw_Hold_Info.angle_target>50)
		{

		 flag_test=-1;}
		else if (yaw_Hold_Info.angle_target<-50)
		{ 

			flag_test=1;
		}
    go_back_run_right(flag_test);
}

void go_back_run_right(int flag_read) 
{
	if((flag_test==1)&&(yaw_Hold_Info.angle_target<80))
	{
		Pitch_Hold_Info.angle_target=0;
		yaw_Hold_Info.angle_target=yaw_Hold_Info.angle_target+0.03;
  }
    else if ((flag_test==-1)&&(yaw_Hold_Info.angle_target>-80))
		{
			Pitch_Hold_Info.angle_target=0;
			yaw_Hold_Info.angle_target=yaw_Hold_Info.angle_target-0.03;
    }
}