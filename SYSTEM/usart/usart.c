#include "sys.h"
#include "usart.h"	
#include "can2.h"
#include "led.h"
#include "Configuration.h"
#include "Comm_umpire.h"

u8 Res[1];
u8 Rx_Cache[USART_RX_LEN];		//���ջ���
u8 Rx_Visual[14];              //�Ӿ����� 
static u8 i;					//���ջ���ָ��
char aa = 0;

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_UART7_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((UART7->SR&0X40)==0);//ѭ������,ֱ���������   
	UART7->DR = (u8) ch;      
	return ch;
}
//#endif
// 
//#if EN_UART7_RX   //���ʹ���˽���
////����3�жϷ������
////ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
//u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
////����״̬
////bit15��	������ɱ�־
////bit14��	���յ�0x0d
////bit13~0��	���յ�����Ч�ֽ���Ŀ
//u16 USART_RX_STA=0;       //����״̬���	

////��ʼ��IO ����3
////bound:������
void usart_init(u32 bound)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//ʹ��UART7ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//ʹ��DMA1ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //GPIOD8����ΪUART7
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //GPIOD9����ΪUART7
	
	//UART7�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7; //GPIOD8��GPIOD9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��PD8��PD9

   //UART7 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART7, &USART_InitStructure); //��ʼ������3
	
  USART_Cmd(UART7, ENABLE);  //ʹ�ܴ���3 
	USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���DMA����
	
  //#if EN_UART7_RX	
  USART_ITConfig(UART7,USART_IT_IDLE,ENABLE);

	//UART7 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

  /* DMA���� */
	DMA_DeInit(DMA1_Stream3);		//����Ϊȱʡֵ
	DMA_InitStructure.DMA_Channel= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART7->DR);		//Դ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Cache;					//Ŀ�ĵ�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								//���ݴ��䷽��Ϊ���赽�ڴ�
	DMA_InitStructure.DMA_BufferSize = USART_RX_LEN;												//�������ݵĻ����С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�ڴ滺������ַ�Լ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//�����ڳ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;										//�����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								//��ʹ��FIFOģʽ
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	
  DMA_Cmd(DMA1_Stream3,ENABLE);
}  


void UART7_IRQHandler(void)                                
{
	uint16_t Length = 0;//���ݳ���
	DMA_Cmd(DMA1_Stream3, DISABLE); //�ر�DMA,��ֹ�������������
	Length = UART7->SR; 
	Length = UART7->DR; //��USART_IT_IDLE��־ 
	Length = USART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);//�õ����ݳ���
	Get_VisualData(Length);
	
	//���������ɱ�־
	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF4 | DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4);
	DMA_SetCurrDataCounter(DMA1_Stream3, USART_RX_LEN);
	DMA_Cmd(DMA1_Stream3, ENABLE);//������,�ؿ�DMA
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
 
