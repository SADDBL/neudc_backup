#include "control.h"

/************************** LYF Start ************************/
//电机插补
void drawline(float x0,float y0,float xe,float ye){
	
}



/***** PID底层 *****/
/**
 * @brief  重设PID参数
 * @param  *p PID结构体
 * @retval None
 */
void pid_resetpara(pid *pid,float p,float i,float d){
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
}
void pid_init(pid *pid_controller, float p, float i, float d)
{
    pid_controller->kp = p;
    pid_controller->ki = i;
    pid_controller->kd = d;
    pid_controller->cur_val = 0;
    pid_controller->target_val = 0;
    pid_controller->err = 0;
    pid_controller->err_k1 = 0;
		pid_controller->err_k2 = 0;
    pid_controller->output = 0;
    pid_controller->output_last = 0;
    pid_controller->i = 0;
		pid_controller->d = 0;
}
/**
 * @brief  位置式PID实现函数
 * @param  *p PID结构体
 * @param  err 误差值：err = current - target
 * @param  outMax 输出最大值
 * @param  outMin 输出最小值
 * @param  i_Max 积分上限
 * @retval p->output，（p->output在该函数中已经被赋值）
 */
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max){
//  if(a<1&&a>0)
//		p->err = first_order_filter(err,p->err_k1,a);
//	else if(a==1){
//		p->err = err;
//	}
	
    // 抗积分饱和
    if (p->output_last > outMax || p->output_last < outMin)
    {
        if (p->output_last * p->err < 0) // err使积分项绝对值减小
            p->i += p->err;
        else
            p->i = p->i;
    }
    else
        p->i += p->err;
    // 积分限幅
    if (p->i > i_Max)
        p->i = i_Max;
    else if (p->i < -i_Max)
        p->i = -i_Max;
		p->d = p->err - p->err_k1;
    p->output = p->kp * p->err + p->ki * p->i + p->kd * (p->d);
    p->err_k1 = p->err;
    p->output_last = p->output;

    // 输出限幅
    if (p->output > outMax)
        p->output = outMax;
    if (p->output < outMin)
        p->output = outMin;

    return p->output;
}

/**
 * @brief  增量式PID实现函数
 * @param  *p PID结构体
 * @param  err 误差值：err = current - target
 * @param  outMax 输出最大值
 * @param  outMin 输出最小值
 * @retval p->output，（p->output在该函数中已经被赋值）
 */
float pid_incremental(pid *p, float err, float outMax, float outMin){
//	int i_flag=1;
	p->err = err;
//	if(fabs(err)>80) i_flag = 0;
	// 抗积分饱和
  if (p->output_last > outMax || p->output_last < outMin){
		if (p->output_last * p->err < 0) // err使积分项绝对值减小
			p->i = p->err;
    else
      p->i = p->i;
	}
  else
		p->i = p->err;
	
	p->d =  p->err + p->err_k2 -2*p->err_k1;
	
	p->output = p->kp * (p->err - p->err_k1) + p->ki * p->i + p->kd * (p->d);
	p->err_k2 = p->err_k1;
	p->err_k1 = p->err;
  p->output_last = p->output;
	
	// 输出限幅
  if (p->output > outMax)
		p->output = outMax;
  if (p->output < outMin)
    p->output = outMin;

   return p->output;
}

/**
  * @brief  PID结构体复位，在切换任务时调用
  * @param  *pid_controller PID结构体
  * @retval None
  */
void pid_reset(pid* pid_controller){
	pid_controller->cur_val = 0;
	pid_controller->target_val = 0;
	pid_controller->err = 0;
	pid_controller->err_k1 = 0;
	pid_controller->err_k2 = 0;
	pid_controller->output = 0;
	pid_controller->output_last = 0;
	pid_controller->i = 0;
	pid_controller->d = 0;
}

/**
  * @brief  一阶滤波
  * @param  new_value 新值
	* @param  last_value 上一次的值
  * @param  a 滤波系数
  * @retval 滤波后的值
  */
float first_order_filter(float new_value,float last_value,float a){
	//a的取值决定了算法的灵敏度，a越大，新采集的值占的权重越大，算法越灵敏，但平顺性差
	//相反，a越小，新采集的值占的权重越小，灵敏度差，但平顺性好。
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}


/************************** LYF End **************************/
