#include "sys.h"
#include "usart.h"	
#include "can2.h"
#include "led.h"
#include "Configuration.h"
#include "Comm_umpire.h"

u8 Res[1];
u8 Rx_Cache[USART_RX_LEN];		//接收缓冲
u8 Rx_Visual[14];              //视觉数据 
static u8 i;					//接收缓冲指针
char aa = 0;

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_UART7_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((UART7->SR&0X40)==0);//循环发送,直到发送完毕   
	UART7->DR = (u8) ch;      
	return ch;
}
//#endif
// 
//#if EN_UART7_RX   //如果使能了接收
////串口3中断服务程序
////注意,读取USARTx->SR能避免莫名其妙的错误   	
//u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
////接收状态
////bit15，	接收完成标志
////bit14，	接收到0x0d
////bit13~0，	接收到的有效字节数目
//u16 USART_RX_STA=0;       //接收状态标记	

////初始化IO 串口3
////bound:波特率
void usart_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//使能UART7时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//使能DMA1时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //GPIOD8复用为UART7
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //GPIOD9复用为UART7
	
	//UART7端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7; //GPIOD8与GPIOD9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化PD8，PD9

   //UART7 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART7, &USART_InitStructure); //初始化串口3
	
  USART_Cmd(UART7, ENABLE);  //使能串口3 
	USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);	//使能串口DMA接收
	
  //#if EN_UART7_RX	
  USART_ITConfig(UART7,USART_IT_IDLE,ENABLE);

	//UART7 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

  /* DMA配置 */
	DMA_DeInit(DMA1_Stream3);		//重置为缺省值
	DMA_InitStructure.DMA_Channel= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART7->DR);		//源地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Cache;					//目的地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								//数据传输方向为外设到内存
	DMA_InitStructure.DMA_BufferSize = USART_RX_LEN;												//设置数据的缓冲大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//内存缓冲区地址自加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//工作在常规模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;										//高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								//不使用FIFO模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	
  DMA_Cmd(DMA1_Stream3,ENABLE);
}  


void UART7_IRQHandler(void)                                
{
	uint16_t Length = 0;//数据长度
	DMA_Cmd(DMA1_Stream3, DISABLE); //关闭DMA,防止处理其间有数据
	Length = UART7->SR; 
	Length = UART7->DR; //清USART_IT_IDLE标志 
	Length = USART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);//得到数据长度
	Get_VisualData(Length);
	
	//清除传输完成标志
	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF4 | DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4);
	DMA_SetCurrDataCounter(DMA1_Stream3, USART_RX_LEN);
	DMA_Cmd(DMA1_Stream3, ENABLE);//处理完,重开DMA
}


static void Get_VisualData(uint16_t Length)
{
	
  Res[0] = USART_ReceiveData(UART7);
	for(int Pointer1 = 0;Pointer1 < Length;Pointer1++)
	{
		if(Rx_Cache[Pointer1] == 0xA4)
		{
			for( i = 0;i < 14;i++)
			{
			  Rx_Visual[i] = Rx_Cache[Pointer1 + i];			
			}
			USART_Get_Vision_Data();
		}
	}
}	
 
