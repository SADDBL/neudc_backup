#ifndef _CONNECT_H_
#define _CONNECT_H_
#include "main.h"
#include <string.h>
#define datasize (uint8_t)14 // uint8_t  ��Ϊunsigned char ��λ
#define HMIUart huart2
#define LINE_MID_DATA 320
#define X_MID 345
#define Y_MID 270
#define OUT_RANGE 999
#define WAIT 'p'
#define STOP 's'
typedef struct UartRecieve
{
    uint8_t Rxbuf;
    uint8_t Uart_Data[datasize];
    uint8_t Flag;    // 0xff��ʾ������ɣ�0x0e��ʾ�ȴ����գ�0xee��ʾ���ճ�������������������չ���ݴ�������
    uint8_t Pointer; // ����ָ��
} UartBuff;

typedef struct{
	int sp_F;	//��ͣ���յ��־λ
	int line_err;	//Ѳ������
	int target_err_x,target_err_y;
}cv_data;
#define BufLen 44 
//���������ݽṹ��
//��������ʹ��DMA�����ж�
typedef struct JY901_DMA{
	uint8_t DataBuf[BufLen];
	float yaw;
	float wz;
	float wz_last;
}JY901;

extern UartBuff UartBuff_Ins1;
extern cv_data cv_ins;
extern JY901 jy901_ins;
extern int state_val;
extern UartBuff UartBuff_Ins;;
extern int mission_select;
void UartDateHandler(UartBuff *UartBuff1);
void HMISends(char *buf1);
void HMISendb(uint8_t k);
void get_yaw_az(JY901 *j);
#endif
