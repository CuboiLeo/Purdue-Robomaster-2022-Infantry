#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

u8 SPI5_ReadWriteByte(u8 TxData);
 	    													  
void SPI5_Init(void);			 //��ʼ��SPI1��
void SPI5_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
	 
#endif

