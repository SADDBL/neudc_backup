#ifndef _MACRO_H_
#define _MACRO_H_
//存放宏定义、跨文件全局
#include "stepper.h"
#include "connect.h"
#include "control.h"

/********************** 宏定义 ****************************/
#define L 1000	//距离1000mm
#define H_RED	250		//红色激光笔高度
#define D_RED 250		//红色激光笔到左侧边缘距离
#define PI 3.14159f
/********************** control文件 ***********************/
#define STEP_LEN 1	//插补步长
#define OK 1	//某事件完成的标志位
#define NOT_OK 2
#define Laser_Off HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET)
#define Laser_On HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET)
extern int calibration_point_list[10];
extern pid pid_stp1,pid_stp2;	//1：红色激光笔x轴，2：红色激光笔y轴
extern laser laser_ins;
/********************** stepper文件 ***********************/
extern Stepper stepper1,stepper2;	//1：x轴（base电机），2：y轴

/********************** connect文件 ***********************/
#define PAUSE 1	//系统暂停标志位
#define RESTART 2	//系统继续运行
extern UartBuff uart_ins1;	//树莓派数据接收
extern UartBuff uart_ins2;	
extern hmi_data hmi_data_ins;	//串口屏数据
extern cv_data cv_ins;
#endif
