#include "pwm.h"

/***************************************************************************
 *ʵ�ֺ�����void TIM2_PWM_Init(void)
 *��    �ܣ���ʼ��Ħ����pwm����ʼ��Ϊ20ms���ڣ�1.5ms�ߵ�ƽ
 ***************************************************************************/
 
void TIM2_PWM_Init(void)
{		 					 	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    //TIM2ʱ��ʹ�� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //ʹ��GPIOAʱ��	

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);  //GPIOA0����Ϊ��ʱ��2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);  //GPIOA1����Ϊ��ʱ��2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);  //GPIOA2����Ϊ��ʱ��2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2;    //GPIOA0.A1.A2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��GPIOA0, A1,A2
	  
	TIM_TimeBaseStructure.TIM_Prescaler=89;  //��ʱ����Ƶ  90000000/900=1MHZ ��1us
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=19999;   //�Զ���װ��ֵ      t=0.001ms*20000=20ms
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ����ʱ��2
	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC2
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);


  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2
	
	TIM2->CCR1 = 999;  //����ȽϼĴ���1
	TIM2->CCR2 = 999;
	TIM2->CCR3 = 600;
	
}


/***************************************************************************
 *ʵ�ֺ�����void TIM3_PWM_Init(void)
 *��    �ܣ���ʼ��imu���¶Ȳ�����
 ***************************************************************************/
void TIM3_PWM_Init(void)
{		 					 	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��GPIOBʱ��	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_TIM3); //GPIOB5����Ϊ��ʱ��3�����ȵ���
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;	//GPIOB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//����
	GPIO_Init(GPIOB,&GPIO_InitStructure);										//��ʼ��GPIOB
	  	
	TIM_TimeBaseStructure.TIM_Prescaler=89;  //��ʱ����Ƶ  90000000/90=1MHZ ��0.001ms
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=999;   //�Զ���װ��ֵ      t=0.001ms*1000=1ms,1kHz
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
	
	//��ʼ��TIM12 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2�����ȵ���

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���

	TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	
	TIM3->CCR2 = 999;	//imu�¶Ȳ�������
	
}


/***************************************************************************
 *ʵ�ֺ�����void TIM12_PWM_Init(void)
*��    ��:  buzzer
 ***************************************************************************/
void TIM12_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//ʹ��GPIOHʱ��	
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6, GPIO_AF_TIM12); //GPIOH6����Ϊ��ʱ��12
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;	//GPIOH6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//����
	GPIO_Init(GPIOH,&GPIO_InitStructure);										//��ʼ��GPIOH
	
	TIM_TimeBaseStructure.TIM_Prescaler=89;  //��ʱ����Ƶ  90000000/90=1MHZ ��0.001ms
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=999;   //�Զ���װ��ֵ      t=0.001ms*1000=1ms,1kHz
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//��ʼ����ʱ��12
	
	//��ʼ��TIM12 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM12 OC1�����ȵ���
	
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //ʹ��TIM12��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPEʹ�� 

	TIM_Cmd(TIM12, ENABLE);  //ʹ��TIM12
	
	TIM12->CCR1 = 0;	//imu�¶Ȳ�������
}


void fric_PWM_configuration(void) //
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);

	TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 180 - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = 1000;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);

	fric_off();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF, GPIO_Pin_10);
}


void fric_off(void)
{
  TIM_SetCompare1(TIM1, 1000);
  TIM_SetCompare4(TIM1, 1000);
}


void fric1_on(uint16_t cmd)
{
  TIM_SetCompare1(TIM1, cmd);
}


void fric2_on(uint16_t cmd)
{
  TIM_SetCompare4(TIM1, cmd);
}


/***************************************************************************
 *ʵ�ֺ�����void PWM_Write(uint8_t PWM_CH,int16_t PWM_Value)
 *��    �ܣ����pwmֵ��ĳ��pwmͨ��
 *��    �룺PWM_CH��pwmͨ����PWM_Value��pwm��ֵ
 *˵    ����PWM_CH��ѡ��PWM12_CH1��PWM12_CH2
 *          ģ�����ź�ֵ��ΧΪ1000-1500-2000
 ***************************************************************************/
void PWM_Write(uint8_t PWM_CH,float pwm)
{
	int PWM_Value;
	
	switch(PWM_CH)
	{
		case PWM2_CH1:
		{
			TIM1->CCR4 = pwm-1;
			break;
		}  
		case PWM2_CH2:
		{
			TIM1->CCR1 = pwm-1;
			break;
		}
		case PWM2_CH3:
		{
			TIM2->CCR3 = PWM_Value-1;
			break;
		}
		default:
			break;
	}
}


