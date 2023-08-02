#ifndef __STEPPER_H
#define __STEPPER_H
#include "main.h"
#include "tim.h"
#include "math.h"

/*******************************************************************************
note :
	�ж���ʵ��������������
	����������¸�����ʱʹ�ø������Ľӿڣ�λ�ã��ٶ� �����ٶ� �����ٶ�

��� ʳ�÷�����
	���岽���������


	main�г�ʼ��
	eg.Init_Stepper(&Stepper1 , GPIOB , GPIO_PIN_4  , &htim4 , TIM_CHANNEL_3 , 0.1125);
					�ṹ��     ����˿�    ��������   stp��ʱ��		��ʱ��ͨ��      �����


����λ�õ��ã�Ŀǰ���ٶȺ��ٶȲ���û���ã��ֱ���Ϊ50 500�ǱȽϺ��ʵ�
void StpDistanceSetNonBlocking( struct Stepper* stepper , 	float angdistance , 	float accel 	, 	float tagv );
   void StpDistanceSetBlocking( struct Stepper* stepper , 	float angdistance , 	float accel 	, 	float tagv );
																			�ṹ��									��λ�ƣ�������				�Ǽ��ٶȣ�����ֵ��		���ٶȣ�����ֵ��

����Ƚ��ж�������������
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)------>��ʱ��
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)------>��ʱ��ͨ��
			StepperInOC(&Stepper1);------>��ʱ��ͨ����Ӧ�Ĳ������
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)------>��ʱ��ͨ��
			StepperInOC(&Stepper2);------>��ʱ��ͨ����Ӧ�Ĳ������
	}
}

 *******************************************************************************/
//------------------------------------------------ȫ�ֱ���&����------------------------------------------------------//
extern const int Freq_t;				  // ����Ԥ��Ƶ֮��ʱ����Ԫ�ĵ���
extern const uint16_t Tim_Period;		  // ��ʱ������
extern const uint16_t Tim_Prescaler;	  // ��ʱ��Ԥ��Ƶϵ��
extern const uint16_t PulseWide_Negetive; // ������ʱ��
extern const uint16_t Min_Period;		  // �������
extern const uint16_t Max_Period;		  // �����

//------------------------------------------------������������ṹ��------------------------------------------------------//
typedef struct Stepper
{
	__IO uint8_t CC_FLAG;
	__IO uint8_t motor_state; //���״̬�Ĵ����λ�ֱ��в�ͬ���� \
																						MSB������1cw0ccw��\
																						[X]		Ԥ��	 \
																						[6 :3]  �˶�ģ̬\
																						[2:1]   �˶��׶�\
																						[0]			�˶���ֹ
	__IO uint16_t CCR_ValSetter;
	__IO uint16_t EnPort; //
	uint16_t EnPin;		  //

	GPIO_TypeDef *DirPort;			   //
	TIM_HandleTypeDef *StpTim;		   //
	uint16_t StpChannel;			   //
	uint16_t DirPin;				   //
	__IO uint32_t acceldistancebuffer; // ����Ӽ��ٶν�λ��
	__IO uint32_t accletperiod0;	   // ����Ӽ��ٶε�һ������

	__IO uint32_t period_buffer;	// Ԥ��ĳ��ٶȣ��ڼ��ٶ���ͬʱ����Ҫ���¼���
	__IO uint32_t period_now;		// ��ǰ����
	__IO uint32_t period_rest;		// ���������������µ����ں󱻸�ֵ
	__IO uint32_t step_counter;		// ��ǰ�׶εĲ�������
	__IO uint32_t step_threshold;	// ��ǰ�׶ε��ܲ���
	__IO uint32_t stepff_memory[6]; // ������������ [0]���� [1]���� [2]����

	int position_ctnow; // �Լ���������ʽ����ĵ�ǰλ�ã�ת���ɽǶ���Ҫ�˲����

	//	float								angv_now;
	float position_ang; // �Ը���Ƕȵ���ʽ��ʾ��λ�ã�
	float stepangle;	// �����
	float TagAngV;		// Ŀ����ٶ�
	float AcceAng;		// �Ǽ��ٶ�

} Stepper;

void Init_Stepper(Stepper *stp, GPIO_TypeDef *stpdir_gpio_port, uint16_t stpdir_gpio_pin,
				  TIM_HandleTypeDef *stps_timer, uint16_t stps_pin, float angpp); // ��ʼ�������ò�����������ͨ����IO��

//---------------------------------------------------�߼�����----------------------------------------------------\\
void			StopSetBit(uint8_t* flag)       {*flag  &= 0xfe ;}								//ֹͣ

/*
0 0000 00 0
0 : ����
000 0������ƽ�� 000 1������ⶥ 100 0������ƽ�� 100 1������ⶥ

00 �׶� 01 10 11 00
0  �˶����Ǿ�ֹ

*/
#define MOVINGFLAG (uint8_t)0x01
#define DIRFLAG (uint8_t)0x80
#define TYPEFLAG (uint8_t)0x08
#define STEPFLAG (uint8_t)0x06
#define DIRCHANGEFLAG (uint8_t)0x40

#define FULLSTEP (uint8_t)0x00
#define MIDPOINT (uint8_t)0x08

#define ACCELING (uint8_t)0x02
#define RUNNING (uint8_t)0x04
#define DECELING (uint8_t)0x06
#define STOPING (uint8_t)0x00

#define STATETOFULLSTEP(flag)   \
	{                           \
		(*flag) &= (~TYPEFLAG); \
		(*flag) |= FULLSTEP;    \
	} // �л���ֹͣ״̬
#define STATETOMIDPOINT(flag)   \
	{                           \
		(*flag) &= (~TYPEFLAG); \
		(*flag) |= MIDPOINT;    \
	} // �л���ֹͣ״̬
#define DIRCHANGENEEDED(flag)       \
	{                               \
		(*flag) |= (DIRCHANGEFLAG); \
	} // �л���Ҫ����״̬
#define DIRCHANGECANCEL(flag)        \
	{                                \
		(*flag) &= (~DIRCHANGEFLAG); \
	} // �л�������ҪҪ����״̬

#define MoveSetBit(flag)          \
	{                             \
		(*flag) &= (~MOVINGFLAG); \
		(*flag) |= MOVINGFLAG;    \
	} // �л����˶�״̬							//
#define StateToAcc(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= ACCELING;    \
	} // �л�������״̬
#define StateToRun(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= RUNNING;     \
	} // �л�������״̬
#define StateToDec(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= DECELING;    \
	} // �л�������״̬
#define StateToStp(flag)        \
	{                           \
		(*flag) &= (~STEPFLAG); \
		(*flag) |= STOPING;     \
	} // �л���ֹͣ״̬

#define IFDIRCHANGE(flag) ((flag) & (uint8_t)DIRCHANGEFLAG)
#define IFMOVING(flag) ((flag) & (uint8_t)MOVINGFLAG)
#define GETDIRECT(flag) ((flag) & (uint8_t)DIRFLAG)
#define GETTYPE(flag) ((flag) & (uint8_t)TYPEFLAG)
#define GETRUNSTEP(flag) ((flag) & (uint8_t)STEPFLAG)

#define CW (uint8_t)0x80
#define CCW (uint8_t)0x00
#define IfCW(flag) ((flag) & (uint8_t)CW)
#define DIRTOCW(flag)          \
	{                          \
		(*flag) &= (~DIRFLAG); \
		(*flag) |= CW;         \
	} // �л���˳ʱ��״̬
#define DIRTOCCW(flag)         \
	{                          \
		(*flag) &= (~DIRFLAG); \
		(*flag) |= CCW;        \
	} // �л�����ʱ��״̬
// #define			CHANGEDIR(flag)				{(IfCW(*flag))? {DIRTOCCW(flag)}:{DIRTOCW(flag)}}//����

/*----------------------- ���Ŷ��壬ע��һ��Ҫ����hal�������ú� -----------------------------------*/

/*----------------------- �������ſ��� -----------------------------------*/

/*----------------------- ʧ�����ſ��� -----------------------------------*/
/* ʹ��ʧ�� x = 1 ��Ч��x = 0ʱ��Ч*/

/******************************************************************************************/
/* �ⲿ�ӿں���*/
extern struct Stepper Stepper1;

void Init_Stepper(Stepper *stp, GPIO_TypeDef *stpdir_gpio_port, uint16_t stpdir_gpio_pin,
				  TIM_HandleTypeDef *stps_timer, uint16_t stps_ch, float angpp);

float _Cal_PositionAng(struct Stepper *stepper);
// ���ü��������㵱ǰ���꣨�����ʾ��������

void StpDistanceSetNonBlocking(struct Stepper *stepper, float angdistance, float accel, float tagv);
// �����µ��˶�ʱ�����˶��ھ��˶�ͬ���򣬽���Ҫ���ߵĲ�����ӵ���ǰ���˶���

void StpDistanceSetBlocking(struct Stepper *stepper, float angdistance, float accel, float tagv);
// �����µ��˶�ʱ�����˶��ھ��˶�ͬ���򣬽���Ҫ���ߵĲ�����ӵ���ǰ���˶���

void StepperInOC(Stepper *stp);

void Stepper_Pause(struct Stepper *stepper);

void Stepper_Stop(Stepper *stepper);

#endif

