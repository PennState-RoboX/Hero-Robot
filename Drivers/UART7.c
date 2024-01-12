#include "sys.h"
#include "UART7.h"	
#include "led.h"
#include "Configuration.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 


#if EN_USART7_RX   //���ʹ���˽���
//����7�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	


#define b_uart_head 0x80
#define b_rx_over 0x40
volatile unsigned char u7_rx_buffer[u7_rx_buffer_SIZE];	//���ջ���,��󳤶�Ϊu7_rx_buffer_SIZE
volatile unsigned char rx_wr_index;	//����дָ��
volatile unsigned char U7_RC_Flag;		//�����жϱ�־
cv_Data_TypeDef cv_Data;//cv���ݸ�ֵ

//��ʼ��IO ����6
//bound:������
void usart7_init(u32 bound){
   //GPIO�˿�����
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//ʹ��USART7ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //GPIOE7����ΪUSART7
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //GPIOE8����ΪUSART7
	
	//USART3�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; //GPIOE7��GPIOE8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��PE7��PE8

   //USART3 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART7, &USART_InitStructure); //��ʼ������7
	
  USART_Cmd(UART7, ENABLE);  //ʹ�ܴ���7
	
	//USART_ClearFlag(USART7, USART_FLAG_TC);
	
#if EN_USART7_RX	
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);//��������ж�
	

	//USART3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//����7�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}

unsigned char SumCkeck(void){
	unsigned int sum = 0;
	unsigned char i;
	for(i=0;i<u7_rx_buffer[0]-4;i++){
		sum += u7_rx_buffer[i];
	}
	//if(sum == u7_rx_buffer[u7_rx_buffer[0]-4]){
		return 1;
	//}
	//return 0;
}

void Decode_Frame(unsigned char data){
  if(data==0xa5) 
  { 
 U7_RC_Flag|=b_uart_head; //������յ�A5 ��λ֡ͷ��רλ
    u7_rx_buffer[rx_wr_index++]=data; //��������ֽ�.
  }
  else if(data==0x5a)
       { 
    if(U7_RC_Flag&b_uart_head) //�����һ���ֽ���A5 ��ô�϶� �����֡��ʼ�ֽ�
      { rx_wr_index=0;  //���� ������ָ��
     U7_RC_Flag&=~b_rx_over; //���֡�Ÿոտ�ʼ��
         }
         else //��һ���ֽڲ���A5
    u7_rx_buffer[rx_wr_index++]=data;
         U7_RC_Flag&=~b_uart_head; //��֡ͷ��־
       }
    else
    { u7_rx_buffer[rx_wr_index++]=data;
   U7_RC_Flag&=~b_uart_head;
   //if(rx_wr_index==u7_rx_buffer[0]) //�չ����ֽ���.
		if(data == 0xFF)
      {  
   U7_RC_Flag|=b_rx_over; //��λ ����������һ֡����
   UART7_CommandRoute(); //������ȡ���ݡ�
          }
    }

  if(rx_wr_index==u7_rx_buffer_SIZE) //��ֹ���������
  rx_wr_index--;
}
void UART7_CommandRoute(void){
	if (U7_RC_Flag&b_rx_over){ //check if already finish recive
		U7_RC_Flag&=~b_rx_over; //reset end of frame flag
		if(SumCkeck()){
			//TODO: RUN AUTO AIM ROUTEIN
			int pitch_int = u7_rx_buffer[1];
			int pitch_deci = u7_rx_buffer[2];
			int yaw_int = u7_rx_buffer[3];
			int yaw_deci = u7_rx_buffer[4];
			cv_Data.pitch = (float)(pitch_int) - 50.0 + (float)pitch_deci/100;
			cv_Data.yaw = (float)(yaw_int) - 50.0 + (float)yaw_deci/100;
			if (cv_Data.pitch == 0.5 || cv_Data.yaw == 0.5){
				cv_Data.yaw = 0.0;
				cv_Data.pitch = 0.0;
			}
		}
	}
}
	

void UART7_IRQHandler(void)                	//����3�жϷ������
{
	u8 Res;
	unsigned char tmp;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	LED1 =! LED1;
	if(USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(UART7);//(USART6->DR);	//��ȡ���յ������� 
		Decode_Frame(Res);
		USART_ClearITPendingBit(UART7,USART_IT_RXNE);	//����жϱ�־ this method is not working
		
		/**** TODO: Clear Unused code before merge ****/
		
		//tmp = UART7->SR;
		//tmp = UART7->DR; //use this method instead
		
		/*
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   
		*/		 
  } 
}

int UART7_Tx(int ch)
{ 	
	USART_SendData(UART7,ch);
	 while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET);
	//while((UART7->SR&0X40)==0){//ѭ������,ֱ���������   
	//UART7->DR = (u8) ch;}  
	return ch;

} 
/*****************  �����ַ��� **********************/
void Usart_SendString( char *str)
{
    unsigned int k=0;
    do {
        UART7_Tx(*(str + k) );
        k++;
    } while (*(str + k)!='\0');

    /* �ȴ�������� */
    while (USART_GetFlagStatus(UART7,USART_FLAG_TC)==RESET) {
    }
}
#endif	