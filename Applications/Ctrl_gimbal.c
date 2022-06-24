#include "Ctrl_gimbal.h"
#include "Kalman.h"
#include "MPU6500.h"
#include "IMU.h"
#include "PID.h"
#include "can1.h"
#include "Configuration.h"
#include "User_Api.h"
#include <math.h>
#include <stdlib.h> 
#include "RemoteControl.h"
#include "delay.h"
#include "pwm.h"
#include "Higher_Class.h"
#include "User_Api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Comm_umpire.h"
#include "laser.h"
#include "Ctrl_shoot.h"
#include "can2.h"
#include "Usart_SendData.h"

#define DPI 0.0439453125f	 //��ֵ�Ƕȷֱ��ʣ�360/8192
#define PITCH_IMU_CTRL  //pitch�ᷴ�����Ʒ�ʽ
#define SHOOT_FRIC_PWM_ADD_VALUE 40  //Ħ�����źŵ���ֵ
#define fri_min_value 0  //Ħ�����ź���С���ֵ

float fri_out = 0;  //Ħ�����źŵ������ֵ
float fri_out1 = 0;  //Ħ�����źŵݼ����ֵ
float fri_max_value = 1000;  //Ħ�����ź�������ֵ //1560
float speed_17mm_level = 1000;
signed long Fri_Left;
signed long Fri_Right;

//����Yaw����Ƶı���
float angleYaw;
float E_yaw;  //δУ׼��yaw�Ƕ�ֵ����Ҫ��������̨����ǰ��ֵE_yaw=imu.yaw
float T_yaw = 0;  //Ŀ��Ƕ�
float Yaw_AnglePid_i;  //�ǶȻ�������
float angle_output1 = 0;
float gryoYaw = 0;//���ٶ�
float gryoPITCH = 0;
float Yaw_Encode_Angle = 0; 
float Pitch_Encode_Angle = 0;  //Yaw��Pitch�Ƕ�
float Yaw_Angle_Output = 0;
float Target_Yaw = 0;
int Yaw_Prev_Angle = 0;

int16_t pitch_moto_current_final;
int16_t Yaw_PID_current;
int16_t yaw_moto_current_final;
int32_t rev_count = 0;

//Jscope����
float Yaw_Encode_Angle_jscope;
float gryoYaw_jscope;
float Pitch_angle_pid_Control_OutPut;

static void Pitch_pid(float Target_angle)//pitch�ᴮ��PID����
{	
	gryoPITCH = -0.3f * (imu.gyro[0] - imu.gyroOffset[0]) * 57.2957795f;
	Pitch_angle_pid_Control_OutPut = PID_Control(&Pitch_angle_pid, Target_angle, Pitch_Encode_Angle);
  PID_Control(&Pitch_speed_pid, Pitch_angle_pid.Control_OutPut, motor2_data[0].ActualSpeed);
}


static void Yaw_angle_pid(float Targrt_Yaw)//yaw�ᴮ��PID����
{ 
	if(Gimbalmode_flag) //Gimbalmode_flag = 1 ��̨���̲�����
	{
		Yaw_Angle_Output = PID_Control(&Yaw_pid, Target_Yaw, Yaw_Encode_Angle);
		Yaw_PID_current = PID_Control(&Yaw_speed_pid, Yaw_Angle_Output, -motor_data[4].ActualSpeed);
	}	 
	else
	{
		Yaw_Angle_Output = PID_Control(&Yaw_pid, Target_Yaw, Yaw_Encode_Angle);
		Yaw_PID_current = PID_Control(&Yaw_speed_pid, Yaw_Angle_Output, -motor_data[4].ActualSpeed);
	}
 } 


void Encoder_angle_Handle(void)//��̨�����ת�ǶȺ���
{
	float Yaw_encode = motor_data[4].NowAngle;
	float Pitch_encode = motor2_data[0].NowAngle;
	float tmp = 0;
	Yaw_Encode_Angle = 0;
	
	tmp = Yaw_encode - Yaw_Mid_encode;
	if((motor_data[4].NowAngle - Yaw_Prev_Angle) > 4000)
		rev_count--;
	else if((motor_data[4].NowAngle - Yaw_Prev_Angle) < -4000)
		rev_count++;
	
	
	Yaw_Encode_Angle = Yaw_Direction * tmp * DPI + rev_count * 360;
  Pitch_Encode_Angle = (Pitch_encode - Pitch_Mid_encode) * DPI * Pitch_Direction;
	Yaw_Prev_Angle = motor_data[4].NowAngle;
}


static void ramp_calc(void)//Ħ����pwmֵ����
{
	fri_out += (SHOOT_FRIC_PWM_ADD_VALUE * 0.01);
	if(fri_out > speed_17mm_level)
	{
		fri_out = speed_17mm_level;
	}
	else if(fri_out < fri_min_value)
	{
		fri_out = fri_min_value;
	}
	
}	


static void ramp_calc1(void)//Ħ����pwmֵ�ݼ�
{ 
	fri_out1 = fri_out-1;
	fri_out--;
	if(fri_out > speed_17mm_level)
	{
		fri_out = speed_17mm_level;
	}
	else if(fri_out < fri_min_value)
	{
		fri_out = fri_min_value;
	}
	
}

//Ħ���ֿ��غ���
void Control_on_off_friction_wheel(void)  
{
	if (RC_Ctl.rc.Switch_Right == RC_SW_MID)
	{
		speed_17mm_level_Start_Flag = 1;
	}
	else if(RC_Ctl.rc.Switch_Left == RC_SW_DOWN)
	{
		speed_17mm_level_Start_Flag = 0;
	}
}

void friction_wheel_ramp_function(void)//Ħ���ֿ��ƺ���
{
	if(speed_17mm_level_Start_Flag == 1)//Ħ���ֿ���
	{
		if(Fire_mode ==0 && (Game_robot_state.robot_level > 1))//��ȴ���ȸߵȼ�����
		  speed_17mm_level = 1000;  // 18m/s //1630
		else
		  speed_17mm_level = 1000;  //1500// 15m/s
			
		ramp_calc();
		
		Fri_Left = -fri_out;
		Fri_Right = fri_out;
		VAL_LIMIT(Fri_Left,-fri_max_value,fri_max_value);
		VAL_LIMIT(Fri_Right,-fri_max_value,fri_max_value);
		CAN2_Send_Msg_Shoot(Fri_Left,Fri_Right);
		//PWM_Write(PWM2_CH1,Fric_Wheel_pid[0].Control_OutPut);
    //PWM_Write(PWM2_CH2,Fric_Wheel_pid[1].Control_OutPut);
	} 
	
	//�ر�Ħ����
	else if(speed_17mm_level_Start_Flag == 0)
	{
		ramp_calc1();
		Fri_Left = 0;
		Fri_Right = 0;
		CAN2_Send_Msg_Shoot(Fri_Left,Fri_Right);
		//PWM_Write(PWM2_CH1,fri_out1);
    //PWM_Write(PWM2_CH2,fri_out1);
	}
}

float Yaw_Set_Angle(float Yaw_Raw)
{
	Target_Yaw += Yaw_Raw;

	return Target_Yaw;
}

//��̨�������
void Gimbal_Ctrl(float pitch,float yaw_rate)
{
  Encoder_angle_Handle();  //��̨�����ת�ǶȺ���
	
	Pitch_pid(pitch);  //pitch�ᴮ��PID����
	Yaw_angle_pid(yaw_rate);//yaw�ᴮ��PID����
	
	yaw_moto_current_final = Yaw_PID_current;
	
	pitch_moto_current_final = Pitch_Direction * Pitch_speed_pid.Control_OutPut;
	CAN1_Send_Msg_gimbal(yaw_moto_current_final, trigger_moto_current);//can������̨������
	CAN2_Send_Msg_gimbal(pitch_moto_current_final);
}


//��̨��������
void FRT_Gimbal_Ctrl(void *pvParameters)
{
	vTaskDelay(201);
	while(1)
	{
		if (RC_Ctl.rc.Switch_Right == RC_SW_DOWN)
			Power_off_function();
		else
		{	
			gimbal_control_acquisition();  //��̨��������ȡ��״̬����
			Target_Yaw = Yaw_Set_Angle(Control_data.Gimbal_Wz);
			Gimbal_Ctrl(Control_data.Pitch_angle,Target_Yaw);  //��̨������Ŀ��ƽӿ�	
		}
	  vTaskDelay(1);  //������������
	}
}
void Gimbal_Init(void)
{
	E_yaw = imu.yaw;	
}

