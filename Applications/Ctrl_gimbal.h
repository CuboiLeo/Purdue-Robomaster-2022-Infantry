#ifndef GIMBAL_H
#define GIMBAL_H

#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "can1.h"

extern int16_t trigger_moto_current;
extern int16_t pitch_moto_current_final;
extern int16_t yaw_moto_current;
extern int16_t yaw_moto_current_final;
extern int32_t rev_count;
extern signed long Fri_Left;
extern signed long Fri_Right;

extern float Yaw_Encode_Angle, Pitch_Encode_Angle;
extern float speed_17mm_level;
extern float gryoYaw;
extern float gryoPITCH;
extern float E_yaw;
extern float P1;
extern float I1;
extern float D1;
extern float angleYaw;
extern float T_yaw;		//目标角度
extern float Yaw_AnglePid_i;//角度环积分项
extern float angle_output1;
extern float error;
extern float Target_Yaw;
extern int Yaw_Prev_Angle;

static void Pitch_pid(float Target_angle);
static void Yaw_angle_pid(float Targrt_d_angle);
static void ramp_calc(void);
static void ramp_calc1(void);

void Encoder_angle_Handle(void);
void Gimbal_Init(void);
void friction_wheel_ramp_function(void);
void Control_on_off_friction_wheel(void);
void Power_off_function(void);
float Yaw_Set_Angle(float Yaw_Raw);
void Gimbal_Ctrl(float pitch,float yaw_rate);
void FRT_Gimbal_Ctrl(void *pvParameters);

#endif

