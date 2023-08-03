#ifndef _MACRO_H_
#define _MACRO_H_
//��ź궨�塢���ļ�ȫ��
#include "stepper.h"
#include "connect.h"
#include "control.h"

/********************** �궨�� ****************************/
#define L 1000	//����1000mm
#define H_RED	250		//��ɫ����ʸ߶�
#define D_RED 250		//��ɫ����ʵ�����Ե����
#define PI 3.14159f
#define PID_START 1
#define PID_STOP 2
extern int PID_F;
/********************** control�ļ� ***********************/
#define STEP_LEN 2	//�岹����
#define OK 1	//ĳ�¼���ɵı�־λ
#define NOT_OK 2
#define Laser_Off HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET)
#define Laser_On HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET)
extern int calibration_point_list[10];
#define D_ANG 0.1125f/2
extern pid pid_stp1,pid_stp2;	//1����ɫ�����x�ᣬ2����ɫ�����y��
extern laser laser_ins;
extern int mission2_point_list[8];//�������Ļ�Ľǵ㣬���¡����ϣ����ϣ�����
/********************** stepper�ļ� ***********************/
extern Stepper stepper1,stepper2;	//1��x�ᣨbase�������2��y��

/********************** connect�ļ� ***********************/
#define PAUSE 1	//ϵͳ��ͣ��־λ
#define RESTART 2	//ϵͳ��������
extern UartBuff uart_ins1;	//��ݮ�����ݽ���
extern UartBuff uart_ins2;	
extern hmi_data hmi_data_ins;	//����������
extern cv_data cv_ins;
#endif
