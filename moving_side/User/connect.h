#ifndef _CONNECT_H_
#define _CONNECT_H_
#include "main.h"
#include <string.h>
#define datasize (uint8_t)25 // uint8_t  即为unsigned char 八位
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
	int laser_axis[2];	//激光点坐标
	int rectangular_axis[8];	//纸靶视觉坐标系坐标
	int rectangular_axis_screen[8];	//纸靶屏幕坐标系坐标
	int rectangular_det_F;
}cv_data;

typedef struct{
	int stop_F;	//暂停标志位
	int mission_select;
	int set_pos;	//选择校正的点位1-5对应：原点：左上、右上、左下、右下
	int set_pos_last;
}hmi_data;

#define BufLen 44 
//陀螺仪数据结构体
//接收数据使用DMA空闲中断
typedef struct JY901_DMA{
	uint8_t DataBuf[BufLen];
	float yaw;
	float wz;
	float wz_last;
}JY901;

void UartDateHandler(UartBuff *UartBuff1);
void HMISends(char *buf1);
//void HMISendb(uint8_t k);
void get_yaw_az(JY901 *j);
#endif
