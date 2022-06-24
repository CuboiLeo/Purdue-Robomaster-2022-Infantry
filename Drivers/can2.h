#ifndef __CAN2_H
#define __CAN2_H
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "can1.h"

typedef struct		//�Ӿ�������ݽṹ��
{
	u8 flash;				//ˢ��                                    
	u8 mode;				//ģʽ
	u8 target_lose;	//��Ŀ��		
}Vision_InitTypeDef;

extern Vision_InitTypeDef Vision_Data;
extern M_Data motor2_data[3];		//�������

void CAN2_Configuration(void);
void CAN2_Send(u8 Msg);
void USART_Get_Vision_Data(void);
void CAN2_Send_Msg_gimbal(int16_t control2_20A);
void CAN2_Send_Msg_Shoot(signed long Fri_Left,signed long Fri_Right);

#endif

