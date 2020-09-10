#include "bkd_feedmotor.h"
#include "main.h"
#include "bkd_remote.h"
#include "math.h"
extern int rank;
extern int maxenergy[3];
extern float energy;
extern uint8_t feedmotor_flag;
extern uint8_t feedspeed_flag;
int16_t Shoot_out=0;
void feedmotor(int16_t speed,int32_t feedspeed)
{
	if(feedmotor_flag==1)
	{
	 PID_Calc1(&CM5_speed_pid,CM5_Feedback.real[0],-speed,Positional);   //0x5dc
	 PID_Calc1(&CM6_speed_pid,CM6_Feedback.real[0],speed,Positional);
	 CAN_Send_Message(&hcan2, 0X200,CM5_speed_pid.output,CM6_speed_pid.output,0,0);
	
		if((CM5_Feedback.real[0]>1000)&&(CM6_Feedback.real[0]<-1000))
			{
				if(energy<(maxenergy[rank-1]-20))
					{	  
						feedspeed_flag=1;
						PID_Calc_Position(&Shoot_position_pid,Shoot_position_Feedback.ecd_value,feedspeed);//bullet_motor_flag
						PID_Calc_Position(&Shoot_speed_pid,Shoot_speed_Feedback.real[0],Shoot_position_pid.output);
						Shoot_out=Shoot_speed_pid.output;
					}
				else
					{
						feedspeed_flag=0;
						PID_Calc_Position(&Shoot_speed_pid,Shoot_speed_Feedback.real[0],0);
						Shoot_out=Shoot_speed_pid.output;
					}
			}
		else
			{
				feedspeed_flag=0;		
				PID_Calc_Position(&Shoot_speed_pid,Shoot_speed_Feedback.real[0],0);
				Shoot_out=Shoot_speed_pid.output;
			}
	}
	else if(feedmotor_flag==0)
	{
	 feedspeed_flag=0;
//	 PID_Calc1(&CM5_speed_pid,CM5_Feedback.real[0],speed,Positional);   //0x5dc
//	 PID_Calc1(&CM6_speed_pid,CM6_Feedback.real[0],-speed,Positional);
	 CAN_Send_Message(&hcan2, 0X200,0,0,0,0);
		
	 PID_Calc_Position(&Shoot_speed_pid,Shoot_speed_Feedback.real[0],0);
	 Shoot_out=Shoot_speed_pid.output;
	}

	}
