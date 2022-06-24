/*
 *can2初始化、接收、发送程序
 *用于与tx2视觉上位机数据交换
 */

#include "can2.h"
#include "led.h"
#include "Configuration.h"
#include "can1.h"
#include "Ctrl_gimbal.h"
#include "Higher_Class.h"

float vision_error = 0;
M_Data motor2_data[3];			//定义电机数据结构体
Vision_InitTypeDef Vision_Data={0};

void CAN2_Configuration(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef       GPIO_InitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//打开GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);//配置引脚复用功能
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_CAN2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//复用推挽输出
	GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;				//中断配置 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure); 

	CAN_DeInit(CAN2);                        //将外围寄存器初始化到它们的默认重置值
	CAN_StructInit(&CAN_InitStructure);      //用它的默认值填充每个CAN_InitStructure成员
	
  /************CAN总线的配置*******************/
	CAN_InitStructure.CAN_TTCM = DISABLE;			//非时间触发通信模式
	CAN_InitStructure.CAN_ABOM = DISABLE;			//软件自动离线管理模式
	CAN_InitStructure.CAN_AWUM = DISABLE;			//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART = DISABLE;			//非自动重传输模式，禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM = DISABLE;			//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
	CAN_InitStructure.CAN_TXFP = ENABLE;			//发送FIFO优先迹，优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	//模式设置： mode:0,普通模式;1,回环模式;
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS1_8tq;
	CAN_InitStructure.CAN_Prescaler = 3;   //brp    CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &CAN_InitStructure);

	/******CAN总线的过滤配置(接收配置)********/
	CAN_FilterInitStructure.CAN_FilterNumber=14;                  //过滤器 0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化   

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		//接收中断  启用或禁用指定的CANx中断  CAN_IT_FMP0:等待中断的FIFO 0消息
//	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		//发送中断
}


//can中断服务函数，接受数据
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	//LED0 =! LED0;	//红色灯闪，代表接收到数据
	
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 
		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
		
		switch(RxMessage.StdId)
		{
			case 0x20A://云台电机pitch轴
			{
				motor2_data[0].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor2_data[0].ActualSpeed = (RxMessage.Data[2]<<8)|RxMessage.Data[3];
				motor2_data[0].Intensity = (RxMessage.Data[4]<<8)+RxMessage.Data[5];
				break;
			}
			case 0x201://can2摩擦轮
			{
				motor2_data[1].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor2_data[1].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor2_data[1].Intensity = (RxMessage.Data[4]<<8)+RxMessage.Data[5];
				break;
			}		
			case 0x202://can2摩擦轮
			{
				motor2_data[2].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor2_data[2].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor2_data[2].Intensity = (RxMessage.Data[4]<<8)+RxMessage.Data[5];
				break;
			}
			default:
				break;
		}
	}	
}


void CAN2_Send(u8 Msg)   //can2发送给tx2测试
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;				//定义一个发送信息的结构体
	
  TxMessage.StdId=0x002;	 // 标准标识符为0
  TxMessage.IDE = CAN_ID_STD;			//指定将要传输的消息的标识符的类型
  TxMessage.RTR = CAN_RTR_DATA;		//指定的帧将被传输的消息的类型   数据帧或远程帧
  TxMessage.DLC = 1;			//指定数据的长度
	TxMessage.Data[0] = Msg;
	
  mbox = CAN_Transmit(CAN2, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;	//等待发送结束
}


void USART_Get_Vision_Data(void)   ////获取usart中的视觉信息
{
	Vision_Data.flash = Rx_Visual[1];//&0xf0;
	Vision_Data.mode = Rx_Visual[2];//&0x0f;
	
	if (Rx_Visual[3] == 1)
	  vision_yaw_angle_target = -Rx_Visual[4] * 0.1f - 0.8f;//*255 - Rx_Visual[4]) * 0.1f;// - VISION_X_Pixels/2;
	else if (Rx_Visual[3] == 2)
		vision_yaw_angle_target = Rx_Visual[4] * 0.1f - 0.8f;
	
	if (Rx_Visual[5] == 1)
	  vision_pitch_angle_target = Rx_Visual[6] * 0.1f;//*255 - Rx_Visual[4]) * 0.1f;// - VISION_X_Pixels/2;
	else if (Rx_Visual[5] == 2)
	  vision_pitch_angle_target = -Rx_Visual[6] * 0.1f;			
	
	vision_run_time = Rx_Visual[7] * 0.01f * 1.15f;  //单位s
	vision_run_fps = Rx_Visual[8];          //单位ms
	
	if (Rx_Visual[9] == 1)
	  vision_yaw_angle_target_last = -Rx_Visual[10] * 0.1f - 0.8f;// * 0.1f;//*255 - Rx_Visual[4]) * 0.1f;// - VISION_X_Pixels/2;
	else if (Rx_Visual[9] == 2)
	  vision_yaw_angle_target_last = Rx_Visual[10] * 0.1f - 0.8f;// * 0.1f;			
		
	if (Vision_Data.flash)
	{
		Vision_Data.target_lose = 0;
		LED1 =! LED1;	//绿色灯闪
		vision_error = 1;
	}
	else 	
	{
		Vision_Data.target_lose = 1;	
    vision_yaw_angle_target = 0;	
		vision_yaw_angle_target_last = 0;
		vision_error = 0;
		vision_run_time = 0;
  }
}


void CAN2_Send_Msg_gimbal(int16_t control2_20A)  //ID 6
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;				//定义一个发送信息的结构体
  TxMessage.StdId=0x2ff;	 // 标准标识符为0
  TxMessage.IDE = CAN_ID_STD;			//指定将要传输的消息的标识符的类型
  TxMessage.RTR = CAN_RTR_DATA;		//指定的帧将被传输的消息的类型   数据帧或远程帧
  TxMessage.DLC = 8;			//指定数据的长度
  TxMessage.Data[2] = control2_20A>>8;
	TxMessage.Data[3] = control2_20A;
	
  mbox = CAN_Transmit(CAN2, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;	//等待发送结束
}


void CAN2_Send_Msg_Shoot(signed long Fri_Left,signed long Fri_Right)//ID 7 8
{
	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage_Tri;						//定义一个发送信息的结构体
	
  TxMessage_Tri.StdId = 0x200;
	TxMessage_Tri.IDE = CAN_ID_STD;
	TxMessage_Tri.RTR = CAN_RTR_DATA;
	TxMessage_Tri.DLC = 8 ;

	TxMessage_Tri.Data[0] = (unsigned char)(Fri_Left>>8);  //摩擦轮速度
	TxMessage_Tri.Data[1] = (unsigned char)(Fri_Left);
	TxMessage_Tri.Data[2] = (unsigned char)(Fri_Right>>8);  //摩擦轮速度
	TxMessage_Tri.Data[3] = (unsigned char)(Fri_Right);
	
	mbox=CAN_Transmit(CAN2,&TxMessage_Tri);   	//发送信息
	
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;	//等待发送结束
}

