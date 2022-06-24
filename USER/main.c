#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timer.h"
#include "Power.h"
#include "pwm.h"
#include "can1.h"
#include "Comm_umpire.h"
#include "RemoteControl.h"
#include "spi.h"
#include "IMU.h"
#include "PID.h"
#include "can2.h"
#include "laser.h"
#include "Configuration.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "User_Api.h"
#include "Higher_Class.h"
#include <math.h>
#include <stdlib.h>
#include "Ctrl_shoot.h"
#include "Start_Task.h"
#include "mpu6050out.h"
#include "inv_mpu.h"
#include "Kalman.h"
#include "buzzer.h"
#include "Usart_SendData.h"
/********************************************************************************************************************
//                                                          _ooOoo_
//                                                         o8888888o
//                                                         88" . "88
//                                                         (| -_- |)
//                                                          O\ = /O
//                                                      ____/`---'\____
//                                                    .   ' \\| |// `.
//                                                     / \\||| : |||// \
//                                                   / _||||| -:- |||||- \
//                                                     | | \\\ - /// | |
//                                                   | \_| ''\---/'' | |
//                                                    \ .-\__ `-` ___/-. /
//                                                 ___`. .' /--.--\ `. . __
//                                              ."" '< `.___\_<|>_/___.' >'"".
//                                             | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                                               \ \ `-. \_ __\ /__ _/ .-` / /
//                                       ======`-.____`-.___\_____/___.-`____.-'======
//                                                          `=---='
//                                                                                                                              
//                                              佛祖保佑            永无BUG
************************************************************************************************************************/                                                         
 /*****************************************************************************                                                                 
                                                                  
                             |,,-                     _-_                   
                             \,|_,,,_   ____,,..,,,,-|\.|,                  
                            `-``.._```````````,______`_|_|                  
                            |,````.--/_,   -. |_-''''`/|\|                  
                            \/,``'   |||  |_| ||,   .\.,_|,                 
                           \'/,      |||' ||- '\\      \|/|                 
                           .,'        /   |/|  |        `'\                 
                          ,_|              ,|             \,                
                         /-                                _'\              
                        /.      __                  __      '\              
                       ,|      (OO)                (OO)      \\             
                      _|                                      -,            
                      |\                   ||                  /_,          
                     ,|                    ||                  /|           
                    _|\                 <__||__>                \|          
                   /|\                     __                   ||          
                   |/,     ....                      .....      \|          
                   |\\.,,_,,,,,,,_,,,,,,,,,,,,,,,,,,,,,,,,,,,...'|          
                   \\      -`--.,,                 ..----,,      |.         
                \,                                                \,,,         
                ./                 -              \,                 |      
                                                                     |      
                |                                                    |      
                |                                                    |      
                |          KINETIC                 ROBOTICS          |                              
								|                                                    |      
								|                                                    |      
								|                                                    |      
							 -|                                                   |      
                ,                                                    |      
                \                                                    |             
                                                           |      
                /                                                    |      
                            ` `|?        ---------..-------''-            _----        
                         |-,,        --     /,,,,,,,,,,,,,,,,    ,,         --/      
`--------------------------------------------           
                                                                  
                                                                  
                                                                    
 *****************************************************************************************************************************/                                                                   

void BSP_init(void);

int main(void)
{
	BSP_init();//外设等初始化
	startTast();//创建任务函数             
  vTaskStartScheduler();//开启任务调度
}
 
void BSP_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	POWER_Init();        //开发板24V电源初始化
	delay_init(168);		//初始化延时函数
	usart_init(115200);     	//初始化串口
	LED_Init();		        //初始化LED端口
	TIM3_PWM_Init();				//imu温度补偿器
	TIM2_PWM_Init();				//舵机pwm
	fric_PWM_configuration();//摩擦轮PWM
	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)//24v 输出 依次上电
	{
		 power_ctrl_on(i);
		 delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
	}
	CAN1_Configuration();		//初始化can总线
	Comm_umpire_Init();      //初始化裁判系统
	RC_Init();							//初始化遥控器
	buzzer_init(65535, 84);	//蜂鸣器初始化 //370, 90
	SPI5_Init();						//初始化SPI1口	
	MPU6500_Init();					//初始化MPU6500
	IMU_Init();	            //初始化滤波器
	IMU_Calibration();		//IMU校准
	buzzer_off();	        //IMU校准完毕关闭蜂鸣器
	Powerlimit_Ctrl(44);  //开场默认45w
	//GPIOIIC_Init();//外置MPU6050引脚初始化	
	//while(MPU_Init())//外置MPU6050寄存器初始化		
	Total_PID_Init();       //pid初始化
	CAN2_Configuration();   //can2初始化
	laser_configuration();  //激光初始化
	kalmanCreate();		      //卡尔曼初始化
	TIM4_Int_Init(79,17999);
	Gimbal_Init();
	Yaw_Prev_Angle = motor_data[4].NowAngle;
}





