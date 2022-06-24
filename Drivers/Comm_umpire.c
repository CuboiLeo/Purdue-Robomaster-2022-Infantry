/*
 *与裁判系统通信
 */
#include "Comm_umpire.h"
#include "led.h"
#include "CRC.h"

u8 Rx_buffer[UART_RX_LEN];		//接收缓冲
u8 SendBuff[UART_RX_LEN];			//发送数据缓冲区
static u8 Pointer;						//接收缓冲指针
int bullet_speed_last;
int bullet_speed;
int offline_flag;
//ext_game_state_t Match_status_data;//比赛状态数据
//ext_game_result_t Match_Result_data;//比赛结果数据
//ext_game_robot_HP_t Robot_survival_data;//机器人存活数据
//ext_event_data_t Site_event_data;//场地事件数据
//ext_supply_projectile_action_t Stop_action_sign;//补给站动作标识
//ext_supply_projectile_booking_t Request_the_depot_to_reload;//请求补给站补弹子弹
ext_game_robot_state_t Game_robot_state;//比赛机器人状态
ext_power_heat_data_t Umpire_PowerHeat;//实时功率热量数据
//ext_game_robot_pos_t Robot_position;//机器人位置
//ext_buff_musk_t Robot_gain;//机器人增益
//aerial_robot_energy_t Air_robot_energy_state;//空中机器人能量状态
//ext_robot_hurt_t The_damage_state;//伤害状态
//ext_shoot_data_t Real_time_shooting_information;//实时射击信息
//ext_bullet_remaining_t The_bullet_state;
Frame_TypeDef Frame = {0};
uint8_t CRC_8=0;
uint16_t CRC_16=0;

/****串口dma初始化，用于接收裁判系统数据****/
/*-----USART6_RX-----PG9----DMA2CH5Stream1&2*/
void Comm_umpire_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* config clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	/* gpio config */
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;				//Rx，PG9                                                                                                                                                                                                                                                                                                       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	/* USART6 mode config */
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6,&USART_InitStructure);
	
	USART_Cmd(USART6,ENABLE);											//使能USART6
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);	//使能串口DMA接收
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);	//使能串口空闲中断
		
	/* 中断配置 */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* DMA配置 */
	DMA_DeInit(DMA2_Stream2);		//重置为缺省值
	DMA_InitStructure.DMA_Channel= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);		//源地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_buffer;					//目的地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								//数据传输方向为外设到内存
	DMA_InitStructure.DMA_BufferSize = UART_RX_LEN;												//设置数据的缓冲大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//内存缓冲区地址自加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//工作在常规模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;										//高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								//不使用FIFO模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	
//	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//不使用dma中断
  DMA_Cmd(DMA2_Stream2,ENABLE);
}


//串口空闲中断
void USART6_IRQHandler(void)                                
{
	uint16_t Length = 0;//数据长度
	//LED1 =! LED1;	//绿色灯闪
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) 
	{ 
		DMA_Cmd(DMA2_Stream2, DISABLE); //关闭DMA,防止处理其间有数据
		Length = USART6->SR; 
		Length = USART6->DR; //清USART_IT_IDLE标志 
		Length = UART_RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);//得到数据长度
		
		#ifdef printf_buff
		/*printf("%d,%d,%d,%d,%d,%d,%d\r\n",Rx_buffer[0],Rx_buffer[1],Rx_buffer[2],Rx_buffer[3],Rx_buffer[4],Rx_buffer[27],Rx_buffer[28]);
		Rx_buffer[0] = 0;
		Rx_buffer[1] = 0;
		Rx_buffer[2] = 0;
		Rx_buffer[3] = 0;
		Rx_buffer[4] = 0;
		Rx_buffer[27] = 0;
		Rx_buffer[28] = 0;
		printf("%d\r\n",Length);*/
		//printf("%d,%d\r\n",Rx_buffer[5],Rx_buffer[6]);
		printf("%d,%d,%d,%d,%d,%d\r\n",Rx_buffer[0],Rx_buffer[1],Rx_buffer[2],Rx_buffer[3],Rx_buffer[4],Rx_buffer[5]);
		#else
		//printf("123");
		Get_UmpireData(Length);
    //printf("%d,%d,%d\r\n",Match_status_data.game_progress,Match_status_data.game_type,Match_status_data.stage_remain_time);
		//printf("%d\r\n",Match_Result_data.winner);
		//printf("%d\r\n",Robot_survival_data.robot_legion);
		//printf("%d\r\n",Site_event_data.event_type);
		//printf("%d,%d,%d,%d\r\n",Stop_action_sign.supply_projectile_id,Stop_action_sign.supply_projectile_num,Stop_action_sign.supply_projectile_step,Stop_action_sign.supply_robot_id);
		//printf("%d,%d,%d\r\n",Request_the_depot_to_reload.supply_num,Request_the_depot_to_reload.supply_projectile_id,Request_the_depot_to_reload.supply_robot_id);
		//printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",Game_robot_state.mains_power_chassis_output,Game_robot_state.mains_power_gimbal_output,Game_robot_state.mains_power_shooter_output,Game_robot_state.max_HP,Game_robot_state.remain_HP,Game_robot_state.robot_id,Game_robot_state.robot_level,Game_robot_state.shooter_heat0_cooling_limit,Game_robot_state.shooter_heat0_cooling_rate,Game_robot_state.shooter_heat1_cooling_limit,Game_robot_state.shooter_heat1_cooling_rate);
		//printf("%d,%f,%d,%d,%d,%d\r\n",Umpire_PowerHeat.chassis_current,Umpire_PowerHeat.chassis_power,Umpire_PowerHeat.chassis_power_buffer,Umpire_PowerHeat.chassis_volt,Umpire_PowerHeat.shooter_heat0,Umpire_PowerHeat.shooter_heat1);
		//printf("%d,%f,%d\r\n",Real_time_shooting_information.bullet_freq,Real_time_shooting_information.bullet_speed,Real_time_shooting_information.bullet_type);
		//printf("%f,%f,%f,%f\r\n",Robot_position.x,Robot_position.y,Robot_position.yaw,Robot_position.z);
		//printf("%f\r\n",Umpire_PowerHeat.chassis_power);
		//printf("%d\r\n",Umpire_PowerHeat.chassis_power_buffer);
		//printf("%d\r\n",Umpire_PowerHeat.shooter_id1_17mm_cooling_heat);
		//printf("%f\r\n",Real_time_shooting_information.bullet_speed);
		//printf("%d",Game_robot_state.robot_level);
		#endif
		
		//清除传输完成标志
		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
		DMA_SetCurrDataCounter(DMA2_Stream2, UART_RX_LEN);
		DMA_Cmd(DMA2_Stream2, ENABLE);//处理完,重开DMA
		offline_flag = 0;
	}
}


//获取裁判系统数据
static void Get_UmpireData(uint16_t Length)
{
	for(Pointer=0;Pointer<Length;Pointer++)
	{
		if(Rx_buffer[Pointer] == 0xA5)//检测到起始标志
		{
			Frame.DataLength = Rx_buffer[Pointer+1] | (Rx_buffer[Pointer+2]<<8);//数据段data长度
			Frame.Seq = Rx_buffer[Pointer+3];//包序号
			Frame.CRC8 = Rx_buffer[Pointer+4];//帧头检验crc8
			Frame.CmdID = Rx_buffer[Pointer+5] | (Rx_buffer[Pointer+6]<<8);//命令id
			CRC_8 = UNVerify_CRC8_Check_Sum((u8 *)&Rx_buffer[Pointer],5);
			CRC_16 = UNVerify_CRC16_Check_Sum((u8 *)&Rx_buffer[Pointer],9+Frame.DataLength);
			Frame.FrameTail = Rx_buffer[Pointer+7+Frame.DataLength] | (Rx_buffer[Pointer+8+Frame.DataLength]<<8);//整包检验crc16
//      Real_time_shooting_information.bullet_speed_last = Real_time_shooting_information.bullet_speed;
//			printf("%d\r\n",Length);
//			printf("%d\r\n",Frame.DataLength);
//			printf("%d\r\n",Frame.Seq);
//			printf("%d\r\n",Frame.CRC8);
//			printf("%d\r\n",Frame.CmdID);
//			printf("%d\r\n",Frame.FrameTail);
			//crc校验
			if(Frame.DataLength + 6 > Length-Pointer)
			{
				break;
			}
			if((Verify_CRC8_Check_Sum((u8 *)&Rx_buffer[Pointer], 5)) && (Verify_CRC16_Check_Sum((u8 *)&Rx_buffer[Pointer], 9+Frame.DataLength)))
			{
				DealDataPack(Frame.CmdID,Frame.DataLength);//解析数据
			}
			else 
			{
				break;
			}
		}
	}
}


static void DealDataPack(uint16_t CmdID, uint16_t DataLength)
{
	Pointer += 7;//偏移7，指到数据段第一个字节
	switch(CmdID)
	{
		case 0x0201:
		{
			Game_robot_state.robot_id = Rx_buffer[Pointer];
			Game_robot_state.robot_level = Rx_buffer[Pointer+1];
			Game_robot_state.remain_HP = Rx_buffer[Pointer+2] | ((Rx_buffer[Pointer+3]) << 8);
			Game_robot_state.max_HP = Rx_buffer[Pointer+4] | (Rx_buffer[Pointer+5] << 8);
			Game_robot_state.shooter_id1_17mm_cooling_rate = Rx_buffer[Pointer+6] | ((Rx_buffer[Pointer+7]) << 8);
			Game_robot_state.shooter_id1_17mm_cooling_limit = Rx_buffer[Pointer+8] | ((Rx_buffer[Pointer+9]) << 8);
			Game_robot_state.shooter_id1_17mm_speed_limit = Rx_buffer[Pointer+10] | ((Rx_buffer[Pointer+11]) << 8);
			Game_robot_state.shooter_id2_17mm_cooling_rate = Rx_buffer[Pointer+12] | ((Rx_buffer[Pointer+13]) << 8);
			Game_robot_state.shooter_id2_17mm_cooling_limit = Rx_buffer[Pointer+14] | ((Rx_buffer[Pointer+15]) << 8);
			Game_robot_state.shooter_id2_17mm_speed_limit = Rx_buffer[Pointer+16] | ((Rx_buffer[Pointer+17]) << 8);
			Game_robot_state.shooter_id1_42mm_cooling_rate = Rx_buffer[Pointer+18] | ((Rx_buffer[Pointer+19]) << 8);
			Game_robot_state.shooter_id1_42mm_cooling_limit = Rx_buffer[Pointer+20] | ((Rx_buffer[Pointer+21]) << 8);
			Game_robot_state.shooter_id1_42mm_speed_limit = Rx_buffer[Pointer+22] | ((Rx_buffer[Pointer+23]) << 8);
			Game_robot_state.chassis_power_limit = Rx_buffer[Pointer+24] | ((Rx_buffer[Pointer+25]) << 8);
		   break;
		}
		case 0x0202:
		{
			Umpire_PowerHeat.chassis_volt =  Rx_buffer[Pointer] | ((Rx_buffer[Pointer+1]) << 8);
			Umpire_PowerHeat.chassis_current = Rx_buffer[Pointer+2] | ((Rx_buffer[Pointer+3]) << 8) | ((Rx_buffer[Pointer+4]) << 16) | ((Rx_buffer[Pointer+5]) << 24);
			Umpire_PowerHeat.chassis_power = Rx_buffer[Pointer+6] | ((Rx_buffer[Pointer+7]) << 8);
			Umpire_PowerHeat.chassis_power_buffer = Rx_buffer[Pointer+8] | ((Rx_buffer[Pointer+9]) << 8);
			Umpire_PowerHeat.shooter_id1_17mm_cooling_heat = Rx_buffer[Pointer+10] | ((Rx_buffer[Pointer+11]) << 8);
			Umpire_PowerHeat.shooter_id2_17mm_cooling_heat = Rx_buffer[Pointer+12] | ((Rx_buffer[Pointer+13]) << 8);
			Umpire_PowerHeat.shooter_id1_42mm_cooling_heat = Rx_buffer[Pointer+14] | ((Rx_buffer[Pointer+15]) << 8);
			break;
		}
		default:
			break;
	}
}

