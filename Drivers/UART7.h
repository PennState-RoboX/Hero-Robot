#ifndef __UART7_H
#define __UART7_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//����3��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2011/6/14
//�汾��V1.4
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
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
//4,�޸���EN_USART3_RX��ʹ�ܷ�ʽ
////////////////////////////////////////////////////////////////////////////////// 	

#define u7_rx_buffer_SIZE 40 //recieve buffer size
#define EN_USART7_RX 			1		//ʹ�ܣ�1��/��ֹ��0������6����


/***** TODO: Clear Unnecessary comments before merge*****/
//extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
//extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void usart7_init(u32 bound);
unsigned char SumCkeck(void);
void UART7_CommandRoute(void);
void Decode_Frame(unsigned char data);
int UART7_Tx(int ch); //Send Data On UART6
void UART7_IRQHandler(void);

typedef struct
{
  float yaw;
	float pitch;
	float depth;
	
}cv_Data_TypeDef;
extern cv_Data_TypeDef cv_Data;
void Usart_SendString( char *str);
#endif


