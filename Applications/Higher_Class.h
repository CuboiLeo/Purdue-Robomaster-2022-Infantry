#ifndef __H_CLASS_H
#define __H_CLASS_H
#include "sys.h"

extern float vision_yaw_angle_target;
extern float vision_yaw_angle_target_last;
extern float vision_pitch_angle_target;
extern float vision_pitch_angle_target_last;
extern float armor_angle_forcast;
extern float vision_run_time;
extern float vision_run_fps;

extern u8 PowerLimit_Flag;
extern int yawcnt;
extern float MAX_VX;
extern float MAX_VY;
extern float MAX_WZ;
extern float imu_yaw1;
extern float SpinTop_timecount;
extern float speed_zoom_double;

void Twist(void);//Å¤ÑüÄ£Ê½
void Power_off_function(void);
void chassis_set_control(void);
void gimbal_set_control(void);
void Chassis_Power_Limit(void);
void Powerlimit_Decision(void);
void Chassis_Maxspeed_ctrl(void);
void Powerlimit_Ctrl(int16_t Target_power);

#endif

