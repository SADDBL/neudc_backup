#ifndef _CONTROL_H_
#define _CONTROL_H_

/***** 结构体定义 *****/
typedef struct PID
{
    float kp, ki, kd;
    float target_val, cur_val;
    float err, err_k1,err_k2;
    float i,d;
    float output, output_last;
} pid;

typedef struct {
	int x_axis;	//x轴坐标
	int y_axis;	//y轴坐标
	int x_target;
	int y_target;
	int x_cur;	//摄像头反馈的激光点位置
	int y_cur;
}laser;	//激光点结构体

/************************** LYF ************************/

void drawline(int X0,int Y0,int Xe,int Ye);
void turn_coordinate(float x, float y);

/***** 底层 *****/
void laser_init(laser *l,int x0,int y0);
void laser_set_target(laser *l,int x,int y);
int focus_calibration_point(laser *l);
void pid_init(pid *pid_controller, float p, float i, float d);
float pid_incremental(pid *p, float err, float outMax, float outMin);
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max);
void pid_reset(pid* pid_controller);
float first_order_filter(float new_value,float last_value,float a);
void pid_resetpara(pid *pid,float p,float i,float d);

/************************** LYF ************************/

#endif
