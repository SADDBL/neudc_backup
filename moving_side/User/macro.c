#include "macro.h"
int PID_F;
/********************** stepper文件 ***********************/
Stepper stepper1,stepper2;	//1：x轴（base电机），2：y轴

/********************** connect文件 ***********************/
UartBuff uart_ins1;	//树莓派数据接收，串口1
UartBuff uart_ins2;	//串口屏数据接收，串口2
hmi_data hmi_data_ins = {0,0,0,0};	//串口屏数据
cv_data cv_ins={0};

/********************** control文件 ***********************/

int calibration_point_list[10] = {0};
int mission2_point_list[8] = {0};
pid pid_stp1,pid_stp2;	//1：红色激光笔x轴，2：红色激光笔y轴
laser laser_ins;

