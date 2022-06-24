#include "Ctrl_chassis.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "Configuration.h"
#include "PID.h"
#include "can1.h"
#include <math.h>
#include <stdlib.h>
#include "Higher_Class.h"
#include "RemoteControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "User_Api.h"
#include "Comm_umpire.h"
#include "Ctrl_gimbal.h"
#include "laser.h"
#include "math.h"
#include "Usart_SendData.h"

#define CHASSIS_MAX_SPEED ((LIM_3510_SPEED/60)*2*PI*WHEEL_R/MOTOR_P)//底盘最大速度，mm/s
SpinTop_t SpinTop={0};

/*********************************************************************
 *实现函数：void virtual_encoders(int16_t aim[4])
 *功    能：计算虚拟码盘
 *输    入：控制器目标值
 *说    明：
 ********************************************************************/
int16_t Encoders[4];		//虚拟码盘值
void virtual_encoders(int16_t aim[4])
{
	uint8_t i;
	for(i=0; i<4; i++)
	{
		Encoders[i] += aim[i];
		if (Encoders[i]>8191) Encoders[i] -= 8192;
		else if (Encoders[i]<0) Encoders[i] += 8192;
	}
}


/*********************************************************************
 *实现函数：int encoders_err(int8_t i,int error)
 *功    能：计算实际目标码盘 与 目标虚拟码盘差 值
 *输    入：电机识别符，周期差值
 *返    回：实际目标码盘 与 目标虚拟码盘差 值
 *说    明：
 ********************************************************************/
int encoders_err(int8_t i,int error)
{
	int temp;
	
	temp = motor_data[i].NowAngle + error;
	temp = temp%8192;	
	if (temp<0) temp += 8192;						//实际目标码盘值
	temp = Encoders[i] - temp;
	if (temp<-3000) temp += 8192;
	else if (temp>3000) temp -= 8192;
	
	return temp;
}


/***************************
 *底盘电机速度控制
 *输入：4个电机的目标速度
 ***************************/
static void motor_speed_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	u8 i;
	int16_t target_speed[4];
	
	target_speed[0]=M1_speed;	target_speed[1]=M2_speed;
	target_speed[2]=M3_speed;	target_speed[3]=M4_speed;
	
	for(i=0;i<4;i++)
		PID_Control(&motor_speed_pid[i],target_speed[i],motor_data[i].ActualSpeed);
}


/***************************
 *底盘电机码值控制，位置闭环v2.0
 *输入：4个电机的每周期目标速度
 *计算码盘差值补偿到目标速度上，相较与用码盘差值做pid，这种方法响应性能更好更平滑
 ***************************/
static M_pid angle_pid[4];		//电机pid结构体
void motor_angle_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	u8 i;
	float P,D;
	int error,temp;
	
	int16_t target_speed[4];
	int16_t target_angle[4];
	
	target_speed[0]=M1_speed;	target_speed[1]=M2_speed;
	target_speed[2]=M3_speed;	target_speed[3]=M4_speed;
	
	for(i=0;i<4;i++)//转换为每周期目标码值
		target_angle[i] = (((float)target_speed[i]/ 60) * 8192)/PID_Hz;
	
	if(Encoders[0]==0)
	{
		if(Encoders[1]==0)
		{
			Encoders[0] = motor_data[0].NowAngle;
			Encoders[1] = motor_data[1].NowAngle;
			Encoders[2] = motor_data[2].NowAngle;
			Encoders[3] = motor_data[3].NowAngle;
		}
	}
	virtual_encoders(target_angle);	//计算虚拟码盘值
	
	for (i=0; i<4; i++)
	{
		P=0; D=0;									//中间值归零
		error = angle_pid[i].old_aim - motor_data[i].D_Angle;	//上一次未完成差值
		error += encoders_err(i,error);					//累加实际码盘值与虚拟码盘值的差
		angle_pid[i].old_aim = error + target_angle[i];					//更新旧目标值
		
		//**********P项**************
		P = PID_MOTOR_ANGLE_KP * error;
		//**********D项**************
		D = (error - angle_pid[i].old_err) * PID_MOTOR_ANGLE_KD;
		angle_pid[i].old_err = error;
		temp = P - D;
		if (temp > LIM_3508_SPEED) angle_pid[i].output = LIM_3508_SPEED;			//目标转速补偿输出限幅
		else if(temp < -LIM_3508_SPEED) angle_pid[i].output = -LIM_3508_SPEED;
		else angle_pid[i].output = temp;
		if ((angle_pid[i].output<70) && (angle_pid[i].output>-70)) angle_pid[i].output = 0;
		
		//输入到速度环
		temp = angle_pid[i].output + target_speed[i];	//补偿后的目标速度
		PID_Control(&motor_speed_pid[i],temp,motor_data[i].ActualSpeed);
	}
}


/***************************
 *逆运动学控制
 *制底盘的水平速度、角速度
 *输入为底盘的运动目标，分别为Vx、Vy、Wv，单位mm/s,rad/s,前后为Vx
 *定义右前轮为w0，逆时针顺数
 ***************************/
#ifdef CHASSIS_POSITION_CRTL
//#define ANGLE_CTRL		//使用位置环
#endif

//void Spintop_handle(void)
//{
//	SpinTop.Angle  = (motor_data[4].NowAngle - Yaw_Mid_encode + 8192) % 8192 / 22.7555556f;
//		if(SpinTop.Angle > 360)
//			SpinTop.Angle = (SpinTop.Angle - 360) * 0.0174532f;
//		else
//			SpinTop.Angle *= 0.0174532f;
//	

//		SpinTop.Vx = Control_data.Vx * cos(SpinTop.Angle) + Control_data.Vy * sin(SpinTop.Angle);
//		SpinTop.Vy = -Control_data.Vx * sin(SpinTop.Angle) + Control_data.Vy * cos(SpinTop.Angle);
//	
//}


static void Inverse_Kinematic_Ctrl(float Vx,float Vy,float Wz)
{
	float w[4];		//四个轮子的实际转速rad/s
	int16_t n[4];	//转换为码盘速度的四个电机的转速
	uint8_t i=0;
	
	SpinTop.Angle  = (motor_data[4].NowAngle - Yaw_Mid_encode + 8192) % 8192 / 22.7555556f;
	if(SpinTop.Angle > 360)
		SpinTop.Angle = (SpinTop.Angle - 360) * 0.0174532f;
	else
		SpinTop.Angle *= 0.0174532f;
	
	SpinTop.Vx = Control_data.Vx * cos(SpinTop.Angle) + Control_data.Vy * sin(SpinTop.Angle);
	SpinTop.Vy = -Control_data.Vx * sin(SpinTop.Angle) + Control_data.Vy * cos(SpinTop.Angle);
		
	w[0] = (SpinTop.Vy - SpinTop.Vx +CHASSIS_K*Wz)/WHEEL_R*speed_zoom;    //speed_zoom 功率限制系数  rear right id = 1
	w[1] = (SpinTop.Vy + SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;   //front right id = 2
	w[2] = (-SpinTop.Vy + SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;  //front left id = 3
	w[3] = (-SpinTop.Vy - SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;  //rear left id = 4

	// direction is determined by the battery switch direction, with switch facing backward
	
	for(i=0;i<4;i++)
	  n[i] = ((float)w[i]*MOTOR_P / (2*PI)) * 60;	//转换为电机码盘速度

	//限制斜着走的速度
	for(i=0;i<4;i++)
	{
		if(n[i] > LIM_3508_SPEED)
		{
			uint8_t j=0;
			float temp = (float)n[i] / LIM_3508_SPEED;	//比例
			
			for(j=0;j<4;j++)	n[j] = (float)n[j]/temp;		//等比例缩小
		}
		else if(n[i] < -LIM_3508_SPEED) 
		{
			uint8_t j=0;
			float temp = -(float)n[i] / LIM_3508_SPEED;	//比例
			
			for(j=0;j<4;j++)	n[j] = (float)n[j]/temp;		//等比例缩小
		}
	}
	motor_speed_ctrl(n[0],n[1],n[2],n[3]);
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
	                         motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);
}


void FRT_Inverse_Kinematic_Ctrl(void *pvParameters)//底盘控制任务
{
	vTaskDelay(357);
	while(1)
	{ 
		if (RC_Ctl.rc.Switch_Right == RC_SW_DOWN)
		{
	    Power_off_function();		
		}
		else
		{
		  Chassis_Power_Limit(); //通过修改斜坡启动斜率，以及限制轮组系数来达到效果
	    chassis_control_acquisition();//底盘控制量获取，状态机设置
		  chassis_set_control();//底盘控制量设置
		  Inverse_Kinematic_Ctrl(Control_data.Vx,Control_data.Vy,Control_data.Chassis_Wz);//底盘控制接口
		}
	  vTaskDelay(2);
	}
}

