/***********************************
 *ȫ�������ļ�
 *Ӣ�۳����԰� 20180709
 ***********************************/
#ifndef __CONFIG_H
#define __CONFIG_H
//#define PI 	3.1415926f

/********************************************************************************************************************
 **********************************************�û��㹦������*********************************************************/

#define CHASSIS_POSITION_CRTL	//����λ�û����ƣ�ע�͵�ʱ���ʹ���ٶȻ�
#define PITCH_IMU_CTRL	//��̨pitch��Ŀ���ʹ��imu�����ľ��ԽǶ�������������ע�͵���ʹ�õ�����̵���ԽǶ�������
#define YAW_ENCO_CTRL		//��̨yaw��ʹ�õ������������

#define PC_HIGH_SPEED_X		2500	//���԰�סshift����ʱ������ģʽ���ٶȣ�ǰ��,mm/s
#define PC_HIGH_SPEED_Y		2500	//���԰�סshift����ʱ������ģʽ���ٶȣ�����,mm/s
#define PC_LOW_SPEED_X		500	//���԰�סCTRL����ʱ������ģʽ���ٶȣ�ǰ��,mm/s
#define PC_LOW_SPEED_Y		500	//���԰�סCTRL����ʱ������ģʽ���ٶȣ�����,mm/s
#define PC_SPEED_X				1500	//���Կ�����ͨ�ٶȣ�ǰ��
#define PC_SPEED_Y				1500	//���Կ�����ͨ�ٶȣ�����

#define PC_MouseSpeed_X		1			//����ٶȱ���
#define PC_MouseSpeed_Y		1			//����ٶȱ���
#define REMOTE_Vx_MAX			2500	//ң������������ٶȣ�ǰ��
#define REMOTE_Vy_MAX			2500	//ң������������ٶȣ�����
#define REMOTE_Wz_MAX			PI		//ң������������ٶȣ�����
#define REMOTE_Yaw_Angle	2050	//ң������������ٶȣ�����
#define REMOTE_Pitch_Angle	25	//pitch�����Ƕ�
//20

/********************************************************************************************************************
 ***************************************************��������*********************************************************/

//*********************���̻�е����*********************
#define MOTOR_P		19		//������ٳݱ�
#define WHEEL_R		76		//���Ӱ뾶
#define CHASSIS_K	445		//K=0.5*(���᳤��+���᳤��)mm

//******************3510�����������********************
/*��19�����������ʵ��õ�
 ************************/
#define LIM_3510_SPEED	6880		//������������ٶ�ֵ
#define LIM_3510_ANGLE	940			//ÿ���������ֵ����

//******************ҡ����λƫ��************************
#define RC_ch0_Offset 0
#define RC_ch1_Offset 0
#define RC_ch2_Offset 0
#define RC_ch3_Offset 0
 
//****************��̨�����������********************
#define Yaw_MID					3400		//�е���ֵ
#define Yaw_MAX					5423		//��ֵ��Χ���е�+-1760
#define Yaw_MIN					1327

#define Total_Step			8192    // Total steps in the C6020 Moter
#define Half_Step				4096		// Half steps in the C6020 Moter
#define Yaw_INIT        2735    // Forward Angle, Initial Angle (Yaw_EncodeAngle = 180)
#define Yaw_Error     	40	    // Yaw_EncodeAngle Range Error (|Actual Angle = Yaw_EncodeAnge| = 2)
#define Wz_Rate					5				// Rotation Rate Coefficient
#define Min_Wz					1.5			// Minimal Yaw Rotation Rate

#define Pitch_MID				6850		//�е���ֵ
#define Pitch_MAX				7450		//��ֵ��Χ���е�+-1760
#define Pitch_MIN				6400
#define Yaw_Direction		 -1			//�����װ����1��-1
#define Pitch_Direction	 -1			//�����װ����1��-1

//****************���̵����������********************
#define M2006_Direction	 -1			//�����װ����1��-1

//*******************pidƵ������**********************
#define PID_Hz 					500


/********************************************************************************************************************
 ***************************************************��������*********************************************************/

//******************����ٶȡ����ٶ�*********************
#define Vx_MAX 				1890			// mm/s��ӦС��LIM_3510_27_Vx
//1280
//1890
#define Vy_MAX				1890			// mm/s��ӦС��LIM_3510_27_Vy
//1890
//1280
#define Wz_MAX 				PI				// rad/s,����ÿ��
#define X_ACC_MAX			1000			//�����ٶȣ������ٵ�X_ACC_MAX��Ҫһ����
//2000
#define Y_ACC_MAX			1000			//�����ٶȣ������ٵ�Y_ACC_MAX��Ҫһ����
//2000
//1000
#define Z_ACC_MAX			(4*PI)		//�����ٶȣ������ٵ�Z_ACC_MAX��Ҫһ����

//****************���̸�����̨ģʽ���**********************
#define GIM_FOLLOW_ANGLE			5			//���̿�ʼ������̨�ĽǶ�,����5��
#define GIM_FOLLOW_ANGLE_D		10			//���̿�ʼ������̨�ĽǶȲ�������GIM_FOLLOW_ANGLE���ɶ�̬��������
																			//��̬������GIM_FOLLOW_ANGLE+-(GIM_FOLLOW_ANGLE_D*Wz_MAX)�ڸ���
#define GIM_FOLLOW_BUFFERING	20			//����Ƕȣ����Ҹ�25��
#define FLLOW_RATE						1.5f		//������FLLOW_RATE���ٶ�׷����̨��
#define FLLOW_ACC							(2*PI)	//���̸�����̨�������ٶȣ������ٵ�FLLOW_ACC��Ҫһ����

#define Chassis_Follow_Error  50			// Chassis follow error range (|Actual Angle = Yaw_EncodeAnge| = 2.17) 
#define Min_Chassis_Wz				0.3f    // Minimal following Wz when the chasiss follows the gimbal
#define Chassis_Wz_Rate				5		// Chassis Wz following rate coefficient
#define Rotate_Rate						1				// Consistent rotating rate

//****************�����Զ�����ģʽ���**********************
#define GIM_HIGH_SPEED_POS		20			//��̨�����˶����䣬��
#define GIM_HIGH_SPEED_RATE		2				//�����˶�����ʱ���ٶȱ���

//*********************Ť��ģʽ���***********************
#define TWIST_ANGLE						5			//Ť���ĽǶ�
#define TWIST_SPEED						PI			//Ť�����ٶ�
#define Twist_Vy_P	         	60       //Since the Twist will affect y axis of velocity, this parameter is used to implement this part


//*********************�Զ���׼ģʽ��أ��߼��***********************
#define VISION_X_Pixels				640	//x������
#define VISION_Y_Pixels				480	//y������
#define VISION_X_Offset				0		//x���е�ƫ��
#define VISION_y_Offset				0		//y���е�ƫ��


/********************************************************************************************************************
 ***************************************************��������*********************************************************/

//****************���̵��pid����************************
//�ٶȻ�pid
#define PID_MOTOR_SPEED_KP			(3.8f)
#define PID_MOTOR_SPEED_KI			(0.0f) //0.2
#define PID_MOTOR_SPEED_KD			(3.5f)
#define PID_MOTOR_SPEED_I_MAX		(1500)	//�������ֵ
#define PID_MOTOR_SPEED_I_Err		(500)		//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���
//λ�û�pid
#define PID_MOTOR_ANGLE_KP			(0.02f)
#define PID_MOTOR_ANGLE_KI			(0.0f)
#define PID_MOTOR_ANGLE_KD			(0.0f)

//****************��̨���pid����************************
//Pitch����ٶȻ�
#define PID_PITCH_SPEED_KP		(8.0f)
//100
#define PID_PITCH_SPEED_KI		(2.9f)
//1.0
//0.5
#define PID_PITCH_SPEED_KD		(5.0f)
//1000
#define PID_PITCH_SPEED_I_MAX	(400)	//�������ֵ
#define PID_PITCH_SPEED_I_Err	(100)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���
//Pitch��ǶȻ�
#define PID_PITCH_ANGLE_KP		(44.0f)
//20
//17
#define PID_PITCH_ANGLE_KI		(1.0f)
#define PID_PITCH_ANGLE_KD		(40)
#define PID_PITCH_ANGLE_I_MAX	(50)	//�������ֵ
#define PID_PITCH_ANGLE_I_Err	(10)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���

//////Yaw����ٶȻ�
//#define PID_YAW_SPEED_KP			(500.0f)
////150
//#define PID_YAW_SPEED_KI			(0.5f)
////0.5
//#define PID_YAW_SPEED_KD			(1000.0f)
////2000
//#define PID_YAW_SPEED_I_MAX		(400)	//�������ֵ
//#define PID_YAW_SPEED_I_Err		(100)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���

//Yaw����ٶȻ�
#define PID_YAW_SPEED_KP			(7.0f)
#define PID_YAW_SPEED_KI			(0.1f)
#define PID_YAW_SPEED_KD			(300.0f)
#define PID_YAW_SPEED_I_MAX		(400)	//�������ֵ
#define PID_YAW_SPEED_I_Err		(100)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���

//Yaw��ǶȻ�
#define PID_YAW_ANGLE_KP			(20.0f)
#define PID_YAW_ANGLE_KI			(0.0f)
#define PID_YAW_ANGLE_KD			(0.0f)
#define PID_YAW_ANGLE_I_MAX		(50)	//�������ֵ
#define PID_YAW_ANGLE_I_Err		(10)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���




//CV_Yaw �ǶȻ�pid
#define CV_PID_YAW_ANGLE_KP			(200.0f)
#define CV_PID_YAW_ANGLE_KI			(50.0f)
#define CV_PID_YAW_ANGLE_KD			(10.0f)
#define CV_PID_YAW_ANGLE_I_MAX		(5)	//�������ֵ
#define CV_PID_YAW_ANGLE_I_Err		(20)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���


//CV_YAW �ٶȻ�pid
#define CV_PID_YAW_SPEED_KP     (7.0f);
#define CV_PID_YAW_SPEED_KI     (0.1f);
#define CV_PID_YAW_SPEED_KD     (300.0f);
#define CV_PID_YAW_SPEED_I_MAX   (400.0);
#define CV_PID_YAW_SPEED_I_Err		(100.0);

//CV_Pitch �ǶȻ�pid
//Pitch��ǶȻ�
#define CV_PID_PITCH_ANGLE_KP		(40.0f)
#define CV_PID_PITCH_ANGLE_KI		(2.0f)
#define CV_PID_PITCH_ANGLE_KD		(10.0f)
#define CV_PID_PITCH_ANGLE_I_MAX	(5)	//�������ֵ
#define CV_PID_PITCH_ANGLE_I_Err	(20)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���

//CV_pitch �ٶȻ�pid
#define CV_PID_PITCH_SPEED_KP		(7.0f)
#define CV_PID_PITCH_SPEED_KI		(0.1f)
#define CV_PID_PITCH_SPEED_KD		(150.0f)
#define CV_PID_PITCH_SPEED_I_MAX	(400)	//�������ֵ
#define CV_PID_PITCH_SPEED_I_Err	(100)	//���ַ���ֵ����ֵ�ڴ�ֵ���ڲŽ��л���






//�������
#define PID_FIRE_SPEED_KP			(3.8f)
#define PID_FIRE_SPEED_KI			(0.8f)
#define PID_FIRE_SPEED_KD			(2.5f)
#define PID_FIRE_SPEED_I_MAX	(50)	//�������ֵ

//�Զ���׼
#define VISION_X_KP						(0.01f)
#define VISION_X_KI						(0)
#define VISION_X_KD						(0)
#define VISION_Y_KP						(0)
#define VISION_Y_KI						(0)
#define VISION_Y_KD						(0)


//***********************IMU parameter************************
#define IMU_TEMP				9000		//target temperature
#define TEMP_Tolerance	60			//tolerance of temperature error
#define TEMP_KP					2.4f
#define TEMP_KI					1
#define TEMP_KD					1

//***********************����������************************
#define BUZZER_TIME			300			//����������һ�ε�ʱ�䳤�ȣ�ms



//***********************snail motor 2132 parameter************************
#define Snail_Direction 	1
#define Fric_UP   				1500
#define Fric_MID					1400
#define Fric_DOWN 				1250
#define Fric_OFF  				1000

#endif
