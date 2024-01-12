/****************************************
 Standard Robot (Jun 17 2022)
 
 Shun Ye
 ****************************************/

// Import libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "Configuration.h"          // Global variable setup 
#include "can1.h"
#include "can2.h"
#include "led.h"
#include "RemoteControl.h"
#include "timer.h"
#include "spi.h"
#include "IMU.h"                    // Inertial measurement unit for pose analysis
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "pwm.h"
#include "User_Api.h"
#include "PID.h"
#include "Higher_Class.h"
#include "Shoot.h"
#include "uart7.h"

#include "digital.h"
#include "fric.h"
#include "Comm_umpire.h"
#include "power_ctrl.h"
#include "user_lib.h"


u16 loopcnt=0;
int fricOn = 0;
int check = 0;
int c1 = 0;
static float cv_yaw_input_angle, cv_pitch_input_angle; //cv 传输到a板的yaw角度和pitch角度，使云台的yaw 和pitch旋转对应的角度。角度从1到359。正为逆时针，负角度为顺时针
																											//转动角度为绝对角度，所以需要传入一个对地面的绝对角度进去

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);	//SysTick开启系统tick定时器并初始化其中断，1ms
	cycleCounterInit();		//初始化系统时间函数
	LED_Init();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	usart7_init(115200);			//串口初始化波特率为115200
	
	Comm_umpire_Init();
	
	TIM3_PWM_Init();				//imu温度补偿器接口初始化
	//TIM12_PWM_Init();				//蜂鸣器初始化
//	TIM1_PWM_Init();				//摩擦轮pwm
	Laser_Init();						//激光初始化
	
	
	digital_init();					//初始化GPIO口
	CAN1_Configuration();		//初始化can总线
	RC_Init();							//初始化遥控器
	
	SPI5_Init();						//初始化SPI1口
	MPU6500_Init();					//初始化MPU6500
	IMU_Init();
	
	TIM4_Int_Init(9,8999);	//10Khz频率计数到10为1ms，系统时基定时器
	Total_PID_Init();    
	
	CAN2_Configuration();
	Gimbal_Ctrl_init();		//初始化云台波轮电机，需将波轮盘与落弹孔手动对齐
	
	//24输出控制口 初始化
  power_ctrl_configuration();
	
//	power_ctrl_on(2);//输出3号24V可控电源

	fric_PWM_configuration();//摩擦轮pwm初始化
	
	delay_ms(100);					//等待电机初始化
	
	
	while(1)
	{
		
		if(Flag500Hz)
		{
			Flag500Hz = 0;
			
			IMUSO3Thread();		//imu姿态解算
			IMU_Temp_Ctrl();	//imu温度闭环
			
			Get_Ctrl_Data();

			if(loopcnt>1500)
			{
				Get_Ctrl_Data();
				Shoot_Ctrl();
			}else loopcnt++;
							char* x = "Y: ";
				char* y = "P: ";
				char* z = "R: ";
				char yaw[20];
				char pitch[20];
				char roll[20];
				sprintf(yaw, "%g", imu.yaw);
				sprintf(pitch, "%g", imu.pitch);
				sprintf(roll, "%g", imu.roll);

				Usart_SendString(x);
				Usart_SendString(yaw);
				Usart_SendString("\n");
				Usart_SendString(y);
				Usart_SendString(pitch);
				Usart_SendString("\n");
				Usart_SendString(z);
				Usart_SendString(roll);
				Usart_SendString("\n");

			// Choose current control device and movement mode
			if(RC_Ctl.rc.s2 == RC_SW_MID){//Emergency Stop
				//Inverse_Kinematic_Ctrl(-neg_velocity_rate*Vx_Lpf,-neg_velocity_rate*Vy_Lpf,-neg_velocity_rate*Wz_Lpf);
				c1 = 0;
				Gimbal_Ctrl(0,0,0,0);
				Vx_Lpf = 0;
				Vy_Lpf = 0;
				Wz_Lpf = 0;
				//Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);
				fric_off();
			}
			else if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_DOWN)//遥控器 Up Down
			{
				c1 = 0;
				//cv_yaw_pitch_autorotate(-(cv_Data.yaw), -(cv_Data.pitch));
		    //Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				//cv_yaw_pitch_autorotate(-10,10);
			  Chassis_Task();
				//shoot_control_loop(Fric_DOWN);
				//UART7_Tx(imu.yaw);
				
				//Chassis_AutoRotate(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle);
				Encoder_angle_conversion();
				Gimbal_Ctrl(-Control_data.Pitch_angle, Control_data.Yaw_angle, 0,0);
				
				//Gimbal_Ctrl(-Control_data.Pitch_angle, Control_data.Wz, 0, 0);
			}
			else if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP)//遥控器 Up Down
			{
				
				//cv_yaw_pitch_autorotate(-(cv_Data.yaw), -(cv_Data.pitch));
		    //Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				//cv_yaw_pitch_autorotate(-10,10);
			  Chassis_Task1();
				//shoot_control_loop(Fric_DOWN);
				//UART7_Tx(imu.yaw);
				
				//Chassis_AutoRotate(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle);
				if(c1 == 0)
				{Encoder_angle_conversion();
				c1 = 1;}
				
				Gimbal_Ctrl(10000, Control_data.Yaw_angle, 0,0);
				
				//Gimbal_Ctrl(-Control_data.Pitch_angle, Control_data.Wz, 0, 0);
			}
			else if(RC_Ctl.rc.s1 == RC_SW_MID && RC_Ctl.rc.s2 == RC_SW_DOWN){//遥控器 Midden Down
			//	Chassis_Task();
//				shoot_control_loop(Fric_DOWN);
				c1 = 0;
				if(check == 0)
				{
				CAN1_Send_Msg_gimbal(0,-20000,0);
					check = 1;
				}
					cv_yaw_pitch_autorotate(cv_Data.yaw,cv_Data.pitch);
				
			}
			else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN){//遥控器 Down Down 
//				PWM_Write(PWM1_CH1, 1000+abs(RC_Ctl.rc.ch3-1024)*0.6f);//摩擦轮	
//				PWM_Write(PWM1_CH4, 1000+abs(RC_Ctl.rc.ch3-1024)*0.6f);//摩擦轮					
					//Gimbal_Ctrl(Control_data.Pitch_angle, Control_data.Wz,shootSpeed,0);
				
					//shootSpeed  = MID_SPEED;
					//targetSpeed = Fric_MID; //全局变量
					//bprotect_fire_speed(shootSpeed,targetSpeed);
				  //Chassis_AutoRotate(Control_data.Vx,Control_data.Vy,Control_data.Wz,shootSpeed);
					//cv_yaw_pitch_autorotate(1,1);
				  shoot_control_loop(Fric_DOWN);
  				//Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				
			}
			else if(RC_Ctl.rc.s2 == RC_SW_UP){ // computer control
				//shoot_control_loop(targetSpeed);
				//Chassis_AutoFollow(-Control_data.Vy,Control_data.Vx,Control_data.Wz,Control_data.Pitch_angle,shootSpeed);		
				
				if (RC_Ctl.key.V){
						targetSpeed = Fric_DOWN;
				}
				else{
					targetSpeed = Fric_OFF;
				}

				if(RC_Ctl.mouse.press_l){//摩擦轮高速开启，拨盘电机转动
					if(RC_Ctl.key.Shift){
						targetSpeed = Fric_UP;
						shootSpeed = 500;
					}
					else if(RC_Ctl.key.Ctrl){
						targetSpeed = Fric_MID;
						shootSpeed = 300;
					}
					else{//普通射速
						targetSpeed = Fric_MID;
						shootSpeed = 400;
					}
				}
				else if(RC_Ctl.mouse.press_r){//摩擦轮高速开启，拨盘电机转动
					shootSpeed = -300;
				}
				else{
					shootSpeed = 0;
				}
				
				if(RC_Ctl.key.R){
					Chassis_AutoRotate(Control_data.Vx,Control_data.Vy,Control_data.Wz,0);
				}
				
			}
			
			
			//Astrict_Acc(Control_data.Vx,Control_data.Vy,Control_data.Wz);//加速度限制
			//Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//底盘控制接口
			//Gimbal_Ctrl2(3,0);
			
			/*
			if(RC_Ctl.rc.s1 == RC_SW_DOWN) Shoot_Flag =1;
			else Shoot_Flag = 0; */
		
		if(Flag200Hz_Thread1)
		{
			Flag200Hz_Thread1 = 0;	
		}
		
		if(Flag200Hz_Thread2)
		{
			Flag200Hz_Thread2 = 0;	
		}
		
		if(Flag100Hz_Thread1)    
		{
			Flag100Hz_Thread1 = 0;	
		}
		
		if(Flag100Hz_Thread2)
		{
			Flag100Hz_Thread2 = 0;	
		}
		
		if(Flag50Hz)
		{
			Flag50Hz = 0;
			
			LED0 = !LED0;
//			PWM_Write(PWM1_CH1,1000+abs(RC_Ctl.rc.ch3-1024)*1.5);//摩擦轮	
//      PWM_Write(PWM1_CH4,1000+abs(RC_Ctl.rc.ch3-1024)*1.5);//摩擦轮
		}
	}
}
}
