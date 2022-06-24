#ifndef __USART_SEND_H
#define __USART_SEND_H
#include "sys.h"
#include "usart.h"

extern int SpinTop_Flag_cnt;
extern int Vision_Flag_cnt;
extern int Fire_Mode_cnt;
extern int Chassis_Mode_cnt;
extern int Stronghold_Flag_cnt;

void graphic_1t(u8 Variable);
void number_t(u8 Variable);
//***************************************************************自定义*********************************************************************************
/* 帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} FrameHeader_struct;


/* 交互数据 */
typedef __packed struct
{
  uint16_t cmd_ID;//数据段的内容 ID
  uint16_t send_ID;//发送者的 ID  ， 需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
  uint16_t receiver_ID;//接收者的 ID ， 需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
} ID_struct;

/***************************数据处理*******************************/
/******** - 图形 -*************/
typedef __packed struct
{
	uint8_t  graphic_name[3]; 
	uint32_t operate_tpye:3;//图形操作
	uint32_t graphic_tpye:3;//图形类型
	uint32_t layer:4;//图形层数
	uint32_t color:4;//图形颜色
	uint32_t start_angle:9;//开始角度
	uint32_t end_angle:9;//结束角度
	uint32_t width:10;//线宽
	uint32_t start_x:11;//开始x坐标
	uint32_t start_y:11;//开始y坐标
	uint32_t radius:10;//字体大小或半径
	uint32_t end_x:11;//结束x坐标
	uint32_t end_y:11;//结束y坐标
} graphic_data_struct;


/************** - 浮点数 - - 整数型 -***************/
typedef __packed struct
{
	uint8_t  number_name[3]; 
	uint32_t operate_tpye:3;//图形操作
	uint32_t graphic_tpye:3;//图形类型
	uint32_t layer:4;//图形层数
	uint32_t color:4;//图形颜色
	uint32_t start_angle:9;//开始角度
	uint32_t end_angle:9;//结束角度
	uint32_t width:10;//线宽
	uint32_t start_x:11;//开始x坐标
	uint32_t start_y:11;//开始y坐标
	int32_t  data;//数据
} number_data_struct;


/************** - 字符 -***************/

typedef __packed struct
{
	uint8_t  character_name[3]; 
	uint32_t operate_tpye:3;//图形操作
	uint32_t graphic_tpye:3;//图形类型
	uint32_t layer:4;//图形层数
	uint32_t color:4;//图形颜色
	uint32_t start_angle:9;//开始角度
	uint32_t end_angle:9;//结束角度
	uint32_t width:10;//线宽
	uint32_t start_x:11;//开始x坐标
	uint32_t start_y:11;//开始y坐标
	uint32_t radius:10;//字体大小或半径
	uint32_t end_x:11;//结束x坐标
	uint32_t end_y:11;//结束y坐标
  unsigned char     data[30];
} character_data_struct;
/*******************************************************************************/	

/************** - 发送结构体 -***************/
typedef __packed struct
{
	FrameHeader_struct  	 Frame;
	uint16_t 							 CmdID;//命令码
	ID_struct           	 ID;
	graphic_data_struct  	 data[7];
	uint16_t  						 CRC16;
} graphic_struct;

typedef __packed struct
{
	FrameHeader_struct  	 Frame;
	uint16_t 							 CmdID;//命令码
	ID_struct           	 ID;
	number_data_struct 	 	 data[2];
	uint16_t  						 CRC16;
} number_struct;

typedef __packed struct
{
	FrameHeader_struct  	 Frame;
	uint16_t 							 CmdID;//命令码
	ID_struct            	 ID;
	character_data_struct  data;
	uint16_t  						 CRC16;
} character_struct;

void FRT_USART_DATA_Ctrl(void *pvParameters);

#endif
