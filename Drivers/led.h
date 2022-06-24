#ifndef __LED_H
#define __LED_H
//#include "main.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

//LED 端口定义
#define LED0 PEout(11) // DS0，红
#define LED1 PFout(14)// DS1，绿
#define LED2 PDout(12)
#define LED3 PDout(13)
#define LED4 PDout(14)

#define LASER PGout(13) //LASER0

void LED_Init(void);//初始化
void Laser_Init(void);//初始化激光

#endif

