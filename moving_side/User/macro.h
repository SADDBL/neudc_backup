#ifndef _MACRO_H_
#define _MACRO_H_
//��ź궨�塢���ļ�ȫ��
#include "stepper.h"
#include "connect.h"

/********************** �궨�� ****************************/
#define L 1000	//����1000mm
#define H_RED	250		//��ɫ����ʸ߶�
#define D_RED 250		//��ɫ����ʵ�����Ե����

/********************** control�ļ� ***********************/
#define STEP_LEN 1

/********************** stepper�ļ� ***********************/
extern Stepper stepper1,stepper2;	//1��x�ᣨbase�������2��y��

/********************** connect�ļ� ***********************/
extern UartBuff uart_ins1;	//��ݮ�����ݽ���

#endif
