#include "sys.h"
#include "UART7.h"	
#include "led.h"
#include "Configuration.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 


#if EN_USART7_RX   //如果使能了接收
//串口7中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	


#define b_uart_head 0x80
#define b_rx_over 0x40
volatile unsigned char u7_rx_buffer[u7_rx_buffer_SIZE];	//接收缓冲,最大长度为u7_rx_buffer_SIZE
volatile unsigned char rx_wr_index;	//数据写指针
volatile unsigned char U7_RC_Flag;		//接收中断标志
cv_Data_TypeDef cv_Data;//cv数据赋值

//初始化IO 串口6
//bound:波特率
void usart7_init(u32 bound){
   //GPIO端口设置
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//使能USART7时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //GPIOE7复用为USART7
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //GPIOE8复用为USART7
	
	//USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; //GPIOE7与GPIOE8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化PE7，PE8

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART7, &USART_InitStructure); //初始化串口7
	
  USART_Cmd(UART7, ENABLE);  //使能串口7
	
	//USART_ClearFlag(USART7, USART_FLAG_TC);
	
#if EN_USART7_RX	
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);//开启相关中断
	

	//USART3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//串口7中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

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
 U7_RC_Flag|=b_uart_head; //如果接收到A5 置位帧头标专位
    u7_rx_buffer[rx_wr_index++]=data; //保存这个字节.
  }
  else if(data==0x5a)
       { 
    if(U7_RC_Flag&b_uart_head) //如果上一个字节是A5 那么认定 这个是帧起始字节
      { rx_wr_index=0;  //重置 缓冲区指针
     U7_RC_Flag&=~b_rx_over; //这个帧才刚刚开始收
         }
         else //上一个字节不是A5
    u7_rx_buffer[rx_wr_index++]=data;
         U7_RC_Flag&=~b_uart_head; //清帧头标志
       }
    else
    { u7_rx_buffer[rx_wr_index++]=data;
   U7_RC_Flag&=~b_uart_head;
   //if(rx_wr_index==u7_rx_buffer[0]) //收够了字节数.
		if(data == 0xFF)
      {  
   U7_RC_Flag|=b_rx_over; //置位 接收完整的一帧数据
   UART7_CommandRoute(); //立即提取数据。
          }
    }

  if(rx_wr_index==u7_rx_buffer_SIZE) //防止缓冲区溢出
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
	

void UART7_IRQHandler(void)                	//串口3中断服务程序
{
	u8 Res;
	unsigned char tmp;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	LED1 =! LED1;
	if(USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(UART7);//(USART6->DR);	//读取接收到的数据 
		Decode_Frame(Res);
		USART_ClearITPendingBit(UART7,USART_IT_RXNE);	//清除中断标志 this method is not working
		
		/**** TODO: Clear Unused code before merge ****/
		
		//tmp = UART7->SR;
		//tmp = UART7->DR; //use this method instead
		
		/*
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
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
	//while((UART7->SR&0X40)==0){//循环发送,直到发送完毕   
	//UART7->DR = (u8) ch;}  
	return ch;

} 
/*****************  发送字符串 **********************/
void Usart_SendString( char *str)
{
    unsigned int k=0;
    do {
        UART7_Tx(*(str + k) );
        k++;
    } while (*(str + k)!='\0');

    /* 等待发送完成 */
    while (USART_GetFlagStatus(UART7,USART_FLAG_TC)==RESET) {
    }
}
#endif	