#include "pwm.h"

/***************************************************************************
 *实现函数：void TIM2_PWM_Init(void)
 *功    能：初始化摩擦轮pwm，初始化为20ms周期，1.5ms高电平
 ***************************************************************************/
 
void TIM2_PWM_Init(void)
{		 					 	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    //TIM2时钟使能 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //使能GPIOA时钟	

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);  //GPIOA0复用为定时器2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);  //GPIOA1复用为定时器2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);  //GPIOA2复用为定时器2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2;    //GPIOA0.A1.A2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化GPIOA0, A1,A2
	  
	TIM_TimeBaseStructure.TIM_Prescaler=89;  //定时器分频  90000000/900=1MHZ 即1us
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=19999;   //自动重装载值      t=0.001ms*20000=20ms
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器2
	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC1
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC2
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);


  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2
	
	TIM2->CCR1 = 999;  //捕获比较寄存器1
	TIM2->CCR2 = 999;
	TIM2->CCR3 = 600;
	
}


/***************************************************************************
 *实现函数：void TIM3_PWM_Init(void)
 *功    能：初始化imu的温度补偿器
 ***************************************************************************/
void TIM3_PWM_Init(void)
{		 					 	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能GPIOB时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_TIM3); //GPIOB5复用为定时器3，加热电阻
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;	//GPIOB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);										//初始化GPIOB
	  	
	TIM_TimeBaseStructure.TIM_Prescaler=89;  //定时器分频  90000000/90=1MHZ 即0.001ms
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=999;   //自动重装载值      t=0.001ms*1000=1ms,1kHz
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM12 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2，加热电阻

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器

	TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	
	TIM3->CCR2 = 999;	//imu温度补偿电阻
	
}


/***************************************************************************
 *实现函数：void TIM12_PWM_Init(void)
*功    能:  buzzer
 ***************************************************************************/
void TIM12_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//使能GPIOH时钟	
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6, GPIO_AF_TIM12); //GPIOH6复用为定时器12
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;	//GPIOH6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//上拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);										//初始化GPIOH
	
	TIM_TimeBaseStructure.TIM_Prescaler=89;  //定时器分频  90000000/90=1MHZ 即0.001ms
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=999;   //自动重装载值      t=0.001ms*1000=1ms,1kHz
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//初始化定时器12
	
	//初始化TIM12 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM12 OC1，加热电阻
	
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPE使能 

	TIM_Cmd(TIM12, ENABLE);  //使能TIM12
	
	TIM12->CCR1 = 0;	//imu温度补偿电阻
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
 *实现函数：void PWM_Write(uint8_t PWM_CH,int16_t PWM_Value)
 *功    能：输出pwm值到某个pwm通道
 *输    入：PWM_CH：pwm通道，PWM_Value：pwm数值
 *说    明：PWM_CH可选：PWM12_CH1，PWM12_CH2
 *          模拟舵机信号值范围为1000-1500-2000
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


