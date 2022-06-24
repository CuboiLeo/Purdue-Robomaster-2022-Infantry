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
//                                              ���汣��            ����BUG
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
	BSP_init();//����ȳ�ʼ��
	startTast();//����������             
  vTaskStartScheduler();//�����������
}
 
void BSP_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	POWER_Init();        //������24V��Դ��ʼ��
	delay_init(168);		//��ʼ����ʱ����
	usart_init(115200);     	//��ʼ������
	LED_Init();		        //��ʼ��LED�˿�
	TIM3_PWM_Init();				//imu�¶Ȳ�����
	TIM2_PWM_Init();				//���pwm
	fric_PWM_configuration();//Ħ����PWM
	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)//24v ��� �����ϵ�
	{
		 power_ctrl_on(i);
		 delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
	}
	CAN1_Configuration();		//��ʼ��can����
	Comm_umpire_Init();      //��ʼ������ϵͳ
	RC_Init();							//��ʼ��ң����
	buzzer_init(65535, 84);	//��������ʼ�� //370, 90
	SPI5_Init();						//��ʼ��SPI1��	
	MPU6500_Init();					//��ʼ��MPU6500
	IMU_Init();	            //��ʼ���˲���
	IMU_Calibration();		//IMUУ׼
	buzzer_off();	        //IMUУ׼��Ϲرշ�����
	Powerlimit_Ctrl(44);  //����Ĭ��45w
	//GPIOIIC_Init();//����MPU6050���ų�ʼ��	
	//while(MPU_Init())//����MPU6050�Ĵ�����ʼ��		
	Total_PID_Init();       //pid��ʼ��
	CAN2_Configuration();   //can2��ʼ��
	laser_configuration();  //�����ʼ��
	kalmanCreate();		      //��������ʼ��
	TIM4_Int_Init(79,17999);
	Gimbal_Init();
	Yaw_Prev_Angle = motor_data[4].NowAngle;
}





