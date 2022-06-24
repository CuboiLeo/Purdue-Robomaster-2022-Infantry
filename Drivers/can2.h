#ifndef __CAN2_H
#define __CAN2_H
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "can1.h"

typedef struct		//视觉相关数据结构体
{
	u8 flash;				//刷新                                    
	u8 mode;				//模式
	u8 target_lose;	//无目标		
}Vision_InitTypeDef;

extern Vision_InitTypeDef Vision_Data;
extern M_Data motor2_data[3];		//电机数据

void CAN2_Configuration(void);
void CAN2_Send(u8 Msg);
void USART_Get_Vision_Data(void);
void CAN2_Send_Msg_gimbal(int16_t control2_20A);
void CAN2_Send_Msg_Shoot(signed long Fri_Left,signed long Fri_Right);

#endif

