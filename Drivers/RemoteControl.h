#ifndef __REMOTECONTROL_H
#define __REMOTECONTROL_H

#include "sys.h"
#include "usart.h"
#include "delay.h"

/* ----------------------- RC Channel Definition---------------------------- */
/*#define RC_CH_VALUE_MIN    ((uint16_t)364 )			//通道最小值
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)			//通道中间值
#define RC_CH_VALUE_MAX 	 ((uint16_t)1684)*/			//通道最大值
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP 	((uint16_t)1)
#define RC_SW_MID 	((uint16_t)3)
#define RC_SW_DOWN 	((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)   //
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)      //---弹舱盖
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)      
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)      //云台180，底盘不动
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)     //小陀螺
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)     //云台底盘180
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)     //摩擦轮
/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct
{
	int16_t Right_VRx;//右摇杆左右
	int16_t Right_VRy;//右摇杆前后
	int16_t Left_VRx;//左摇杆左右
	int16_t Left_VRy;//左摇杆前后
	int16_t Side_Wheel;//侧键滚轮
	int8_t Switch_Left;//左按键
	int8_t Switch_Right;//右按键
	int8_t Switch_Left_last;
	int8_t Switch_Right_last;
}Remote;


typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
	uint8_t press_l_last;
}Mouse;	


typedef	__packed struct
{
	uint16_t buff;
	
	int8_t Vertical;//前后
	int8_t Horizontal;//左右
	
	uint8_t Q;
	uint8_t E;
	uint8_t R;
	uint8_t F;
	uint8_t G;
	uint8_t Z;
	uint8_t X;
	uint8_t C;
	uint8_t V;
	uint8_t B;
	
	uint8_t Shift;
	uint8_t Ctrl;
}Key;


typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;


extern volatile unsigned char sbus_rx_buffer[25];
extern RC_Ctl_t RC_Ctl;

void RC_Init(void);				//遥控接收初始化
void FailSafe_Check(void);		//失败检查
void DMA2_Stream5_IRQHandler(void);
static void KeyDataProcess(Mouse *mouse, Key *key);
static void Get_RC_Data(void);			//获取数据

#endif
