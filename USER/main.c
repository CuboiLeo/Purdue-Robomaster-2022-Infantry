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
//                                              ????????            ????BUG
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
	BSP_init();//????????????
	startTast();//????????????             
  vTaskStartScheduler();//????????????
}
 
void BSP_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//??????????????????????4
	POWER_Init();        //??????24V??????????
	delay_init(168);		//??????????????
	usart_init(115200);     	//??????????
	LED_Init();		        //??????LED????
	TIM3_PWM_Init();				//imu??????????
	TIM2_PWM_Init();				//????pwm
	fric_PWM_configuration();//??????PWM
	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)//24v ???? ????????
	{
		 power_ctrl_on(i);
		 delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
	}
	CAN1_Configuration();		//??????can????
	Comm_umpire_Init();      //??????????????
	RC_Init();							//????????????
	buzzer_init(65535, 84);	//???????????? //370, 90
	SPI5_Init();						//??????SPI1??	
	MPU6500_Init();					//??????MPU6500
	IMU_Init();	            //????????????
	IMU_Calibration();		//IMU????
	buzzer_off();	        //IMU??????????????????
	Powerlimit_Ctrl(44);  //????????45w
	//GPIOIIC_Init();//????MPU6050??????????	
	//while(MPU_Init())//????MPU6050????????????		
	Total_PID_Init();       //pid??????
	CAN2_Configuration();   //can2??????
	laser_configuration();  //??????????
	kalmanCreate();		      //????????????
	TIM4_Int_Init(79,17999);
	Gimbal_Init();
	Yaw_Prev_Angle = motor_data[4].NowAngle;
}





