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

/************************** LYF ************************/

/***** PID底层 *****/
void pid_init(pid *pid_controller, float p, float i, float d);
float pid_incremental(pid *p, float err, float outMax, float outMin);
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max);
void pid_reset(pid* pid_controller);
float first_order_filter(float new_value,float last_value,float a);
void pid_resetpara(pid *pid,float p,float i,float d);

/************************** LYF ************************/

#endif
