#ifndef _MACRO_H_
#define _MACRO_H_
//存放宏定义、跨文件全局
#include "stepper.h"
#include "connect.h"

/********************** 宏定义 ****************************/
#define L 1000	//距离1000mm
#define H_RED	250		//红色激光笔高度
#define D_RED 250		//红色激光笔到左侧边缘距离

/********************** control文件 ***********************/
#define STEP_LEN 1

/********************** stepper文件 ***********************/
extern Stepper stepper1,stepper2;	//1：x轴（base电机），2：y轴

/********************** connect文件 ***********************/
extern UartBuff uart_ins1;	//树莓派数据接收

#endif
