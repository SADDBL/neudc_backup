#include "macro.h"
int PID_F;
/********************** stepper�ļ� ***********************/
Stepper stepper1,stepper2;	//1��x�ᣨbase�������2��y��

/********************** connect�ļ� ***********************/
UartBuff uart_ins1;	//��ݮ�����ݽ��գ�����1
UartBuff uart_ins2;	//���������ݽ��գ�����2
hmi_data hmi_data_ins = {0,0,0,0};	//����������
cv_data cv_ins={0};

/********************** control�ļ� ***********************/

int calibration_point_list[10] = {0};
int mission2_point_list[8] = {0};
pid pid_stp1,pid_stp2;	//1����ɫ�����x�ᣬ2����ɫ�����y��
laser laser_ins;

