#ifndef __TIME_H
#define __TIME_H

#include "sys.h"
#include "usart.h"
#include "delay.h"

#define TRIGGER PAout(3)

extern u8 Flag50Hz,Flag100Hz_Thread1,Flag100Hz_Thread2,Flag200Hz_Thread1,Flag200Hz_Thread2,Flag500Hz;	//Hz Flag

void TIM4_Int_Init(u16 arr,u16 psc);

#endif

