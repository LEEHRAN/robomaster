#include "bkd_Small_gyroscope.h"
#include "main.h"
#include "math.h"
double Angle_defense=0;
double vx_denfense=0;
double vy_denfense=0;
extern	int16_t CM1Speed,CM2Speed,CM3Speed,CM4Speed;
void Small_gyroscope(void)
{
		Angle_defense=(double)(Yaw_position_Feedback.real[0]-YAW_Mechanics_Angle)/8191*2*3.1415926;
		vx_denfense=vx*cos(Angle_defense)+vy*sin(Angle_defense);
		vy_denfense=vy*cos(Angle_defense)-vx*sin(Angle_defense);
		CM1Speed=(vx_denfense+vy_denfense+600)*romote_rate_speed;
		CM2Speed=(-vx_denfense+vy_denfense+600)*romote_rate_speed;
		CM3Speed=(-vx_denfense-vy_denfense+600)*romote_rate_speed;
		CM4Speed=(vx_denfense-vy_denfense+600)*romote_rate_speed;
			
		PID_Calc1(&CM1_speed_pid,CM1_Feedback.real[0],CM1Speed,Positional);
		PID_Calc1(&CM2_speed_pid,CM2_Feedback.real[0],CM2Speed,Positional);
		PID_Calc1(&CM3_speed_pid,CM3_Feedback.real[0],CM3Speed,Positional);
		PID_Calc1(&CM4_speed_pid,CM4_Feedback.real[0],CM4Speed,Positional);
		CAN_Send_Message(&hcan1, 0X200,CM1_speed_pid.output,CM2_speed_pid.output,CM3_speed_pid.output,CM4_speed_pid.output);
}
