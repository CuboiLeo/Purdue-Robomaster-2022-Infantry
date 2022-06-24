#ifndef CHASSIS_H
#define CHASSIS_H
#include "sys.h"


typedef struct Motor_Pid	//位置环pid数据结构
{
	int old_aim;			//旧目标码数值
	int old_err;			//旧差值
	int16_t output;		//输出
}M_pid;


typedef struct
{
	s16 RVx,Vx;//实际速度，设定速度
	s16 RVy,Vy;
	s16 AngleCompensate;//超前角度补偿
	float Angle;//超前角度设定
	float SAngle,RAngle;//遥控器给定角度，实际角度值
}SpinTop_t;

extern SpinTop_t SpinTop;
static void motor_speed_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed);
static void Inverse_Kinematic_Ctrl(float Vx,float Vy,float Wz);
void FRT_Inverse_Kinematic_Ctrl(void *pvParameters);


#endif

