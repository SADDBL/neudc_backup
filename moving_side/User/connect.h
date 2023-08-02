#ifndef _CONNECT_H_
#define _CONNECT_H_
#include "main.h"
#include <string.h>
#define datasize (uint8_t)14 // uint8_t  即为unsigned char 八位
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
    uint8_t Flag;    // 0xff表示接收完成，0x0e表示等待接收，0xee表示接收出错，定义其他可用于扩展数据处理类型
    uint8_t Pointer; // 数组指针
} UartBuff;

typedef struct{
	int sp_F;	//等停和终点标志位
	int line_err;	//巡线数据
	int target_err_x,target_err_y;
}cv_data;
#define BufLen 44 
//陀螺仪数据结构体
//接收数据使用DMA空闲中断
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
