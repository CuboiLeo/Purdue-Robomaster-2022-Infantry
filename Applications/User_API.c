#include "Kalman.h"
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "IMU.h"
#include "MPU6500.h"
#include "Configuration.h"
#include "led.h"
#include "buzzer.h"
#include "RemoteControl.h"
#include "Comm_umpire.h"
#include "Higher_Class.h"
#include "can2.h"
#include "PID.h"
#include "can1.h"
#include "pwm.h"
#include "laser.h"
#include "Ctrl_shoot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "Usart_SendData.h"
#include "Ctrl_gimbal.h"

#define DITHERING_TIMNE 150 

//״̬��
u8 SpinTop_Flag = 0;             //С���ݱ�־
float SpinTop_Direction = 0;
u8 Twist_Flag = 0;			        //Ť����־
u8 Vision_Flag = 0;			        //�Ӿ������־
u8 Gimbal_180_flag = 0;         //180�ȱ�־
u8 Stronghold_flag = 0;         //�������Ʊ�־
u8 Inverse_flag = 0;            //���ַ�ת��־
u8 speed_17mm_level_Start_Flag = 0;  //Ħ���ֿ�����־
u8 Fire_Loading_Flag = 0;  //������־λ
u8 Fire_Loading_End_Flag = 0;   //����������־λ
int Chassismode_flag = 0;       //����ģʽ��־λ
int Gimbalmode_flag = 0;        //��̨ģʽ��־λ

//����
u8 TwistCnt,Inverse_cnt,Stronghold_resist_cnt,SpinTop_cnt,SpinTop_close_cnt;
u8 speed_17mm_level_Close_Cnt,speed_17mm_level_Start_Cnt;
u8 Shoot_mode_cnt,Chassis_mode_cnt,Fire_Loading_Cnt,Fire_Loading_End_Cnt;

//�������
int16_t Target_chassis_power = 0;
int32_t VerticalCnt = 0;
int32_t HorizontalCnt = 0;
float Ramp_K = 1.5f;  
float Wz_increasing = 0;
float PC_Speed_Vx =0;
float PC_Speed_Vy =0;
float speed_zoom = 1.0f ; //ң�������Ƶ��ٶ�
u8 Chassis_mode = Chassis_mode_init;

//�������
int Heatmax = 0;  //�������
int Shootnumber = 0;  //�ɷ����ӵ�����
int SafeHeatflag = 1;  //��ȫ������־
int Shootnumber_fired = 0;  //�ѷ����ӵ���
int cycle_time = 0;  //һ����������ʱ��
float Shootnumber_limit = 4;

ControlDATA_TypeDef Control_data;//��������


void chassis_control_acquisition(void)
{
	if (RC_Ctl.rc.Switch_Right == RC_SW_MID)//ң�ؿ���
	{
		Control_data.Vx = (RC_Ctl.rc.Left_VRy - 1024) * 6.2;
		Control_data.Vy = (RC_Ctl.rc.Left_VRx - 1024) * 6.2; 
		Control_data.Chassis_Wz = 0; //(RC_Ctl.rc.Right_VRx - 1024) * 0.035f;
		Chassismode_flag = 0;
		offline_flag++;//����ϵͳ���߼���
		if(offline_flag >= 2000)
			offline_flag = 2000;
	}
	else if(RC_Ctl.rc.Switch_Right == RC_SW_UP && RC_Ctl.rc.Switch_Left == RC_SW_UP) //���Կ���
	{  
		//����ֵ����б��
		if(RC_Ctl.key.Vertical)
		{ 
			VerticalCnt++;
			PC_Speed_Vx = Ramp_K * sqrt(5000 * VerticalCnt);  //�������������ٶ��𽥼�С�ļ���б��
		}  
		else 
		{
			PC_Speed_Vx =0;
			VerticalCnt=0;
		}
		if (RC_Ctl.key.Horizontal)
		{   
			 HorizontalCnt++;
			 PC_Speed_Vy = Ramp_K*sqrt(4000*HorizontalCnt);//б�������й�  //3000
		}	 
		else 
		{
			 PC_Speed_Vy=0;
			 HorizontalCnt=0;
		}
		Control_data.Vx = RC_Ctl.key.Vertical * PC_Speed_Vx;
		Control_data.Vy = RC_Ctl.key.Horizontal * PC_Speed_Vy;
	 
		//��תб��
		if(RC_Ctl.mouse.x != 0) 
			Wz_increasing = 0.015f;    
		else
			Wz_increasing = 0;
	 
		if(Wz_increasing >= 2.3f)   
			Wz_increasing = 2.3f;       
		Control_data.Chassis_Wz = Wz_increasing * RC_Ctl.mouse.x;

		//������ٶ��޷�
		VAL_LIMIT(Control_data.Vx, -MAX_VX, MAX_VX);  
		VAL_LIMIT(Control_data.Vy, -MAX_VY, MAX_VY);
		VAL_LIMIT(Control_data.Chassis_Wz, -MAX_WZ, MAX_WZ);
  }
}


void gimbal_control_acquisition(void)
{
	if (RC_Ctl.rc.Switch_Right == RC_SW_MID)//ң�ؿ���
	{
		Control_data.Gimbal_Wz = -(RC_Ctl.rc.Right_VRx - 1024) * 0.0002f;  
		Control_data.Pitch_angle += (RC_Ctl.rc.Right_VRy - 1024) * 0.0001f;
		VAL_LIMIT(Control_data.Pitch_angle,-30,35);
		
		Control_on_off_friction_wheel();//Ħ���ֿ��غ���
		friction_wheel_ramp_function();//Ħ���ֿ��ƺ���
		shoot_task();
		/*if (RC_Ctl.rc.Side_Wheel !=1024)
		{
			SpinTop_Flag =1;
			Control_data.Gimbal_Wz = (RC_Ctl.r	c.Right_VRx-1024)*0.001f;
		}else SpinTop_Flag =0;
			*/
		if (RC_Ctl.rc.Side_Wheel > 1024)
		{
			SpinTop_Flag = 1;
			SpinTop_Direction = -1.0f;
			Control_data.Gimbal_Wz = (RC_Ctl.rc.Right_VRx - 1024)*0.001f;
		}
		
		else if (RC_Ctl.rc.Side_Wheel < 1024)
		{
			SpinTop_Flag = 1;
			SpinTop_Direction = 1.0f;
			Control_data.Gimbal_Wz = (RC_Ctl.rc.Right_VRx - 1024)*0.001f;
		}else SpinTop_Flag =0;
		
		Gimbalmode_flag = 0;
	}
	else if(RC_Ctl.rc.Switch_Right == RC_SW_UP && RC_Ctl.rc.Switch_Left == RC_SW_UP)//���Կ���
	{
		Control_data.Gimbal_Wz = -0.006f * RC_Ctl.mouse.x;  
		VAL_LIMIT(Control_data.Gimbal_Wz, -0.696f, 0.696f);	
		Control_data.Pitch_angle = Control_data.Pitch_angle -RC_Ctl.mouse.y * 0.003f;  
		//С����
		if (SpinTop_cnt>DITHERING_TIMNE)
		{
			if (RC_Ctl.key.G && Gimbal_180_flag==0)
			{	
				SpinTop_Flag = 1;
				SpinTop_Direction = -1.0f;
				SpinTop_cnt = 0;
				SpinTop_Flag_cnt = 0;
			}
		}
		else SpinTop_cnt++;
		
	
		//С���ݹر�
		if (SpinTop_close_cnt>DITHERING_TIMNE)
		{
			if (RC_Ctl.key.F && Gimbal_180_flag==0)
			{	
				SpinTop_Flag_cnt = 0;
				SpinTop_close_cnt = 0;
				SpinTop_Flag = 0;
			}
		}
		else SpinTop_close_cnt++;
		
    //17mm���ٵ��ٵ����ٵ��л�(��Ħ����Ĭ������ٶ�)
		if (speed_17mm_level_Start_Cnt > DITHERING_TIMNE)
		{
			if (RC_Ctl.key.E)//��Ħ���֣�Ĭ������ٶ�
			{
				speed_17mm_level_Start_Flag = 1;
				speed_17mm_level_Start_Cnt = 0;
			}
		}speed_17mm_level_Start_Cnt++;
		
		
	  //B���ر�
		if (speed_17mm_level_Close_Cnt > DITHERING_TIMNE)  //��Ħ����
		{
			if (RC_Ctl.key.B)
			{
				speed_17mm_level_Start_Flag = 0;  			
				speed_17mm_level_Close_Cnt = 0;
			}
		}speed_17mm_level_Close_Cnt++;
	

    //CTRL+v ��ը����ģʽ ��������ջ�
		if (Stronghold_resist_cnt > DITHERING_TIMNE)
	  {
			if (RC_Ctl.key.Ctrl&&RC_Ctl.key.V)
			{
				Stronghold_flag = !Stronghold_flag;  
				Stronghold_resist_cnt = 0;
				Stronghold_Flag_cnt = 0;
			}
	  }else Stronghold_resist_cnt++;
		
	
		if (RC_Ctl.key.Shift)
		{
			PowerLimit_Flag = 1; //������ʱջ����ĵ��ݵĵ�
		}else PowerLimit_Flag =0;
			
/*
		if (Inverse_cnt > DITHERING_TIMNE)  //�����ֶ���ת
		{
			if (RC_Ctl.key.X == 1)
			{
				Inverse_flag = 1;
				Inverse_cnt = 0;
			}
			else Inverse_flag = 0;
		}else Inverse_cnt++;
*/
	
		offline_flag++;  //����ϵͳ���߼���
		if (offline_flag >= 2000)
		{
			offline_flag = 2000;
			Stronghold_flag = 1; //����ϵͳ���߽�������ջ���Ȼ���䲻���ӵ�
		}
	
	
	  friction_wheel_ramp_function();  //Ħ���ֿ���
    ShooterHeat_Ctrl();	
		
	  if(speed_17mm_level_Start_Flag && SafeHeatflag)  //Ħ���ִ�ʱ���ֲ�������
		  shoot_task1();  //���ֿ���			
	  if(!speed_17mm_level_Start_Flag)
		  trigger_moto_current = 0;						
		}		
  }		


//��������
static void ShooterHeat_Ctrl(void)
{		
	//��������
	if (Fire_mode == Booming_fire)
	{
		Shootnumber_limit=4;
		if (Game_robot_state.robot_level == 1)  //�����˵ȼ���Ӧ����������
			Heatmax = 150;
		else if (Game_robot_state.robot_level == 2)
			Heatmax = 280;
		else if (Game_robot_state.robot_level == 3)
			Heatmax = 400;
  }
	else //��ȴ����
	{
		Shootnumber_limit=4;
		if (Game_robot_state.robot_level == 1)//�����˵ȼ���Ӧ����������
			Heatmax = 50;
		else if (Game_robot_state.robot_level == 2)
			Heatmax = 100;
		else if (Game_robot_state.robot_level == 3)
			Heatmax = 150;
	}
	
	Shootnumber = (Heatmax - Umpire_PowerHeat.shooter_id1_17mm_cooling_heat)/10;//�ɷ����ӵ�����

  //������������4�ŵ��裬��ȴ��������
	if (Shootnumber < Shootnumber_limit)  //�ѷ����ӵ������ڵ��ڿɷ����ӵ���ʱ����ͣת
	{  
	  SafeHeatflag = 0;
	  trigger_moto_current = 0;
	  Shootnumber_fired = 0;
	}
	else
	{
	  SafeHeatflag	= 1;
	}

	if (Stronghold_flag)  //����ϵͳ���߽�������ջ���Ȼ���䲻���ӵ�
	  SafeHeatflag = 1;
	
}

