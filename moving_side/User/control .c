#include "control.h"
#include "macro.h"
#include "usart.h"
#include <math.h>
#include <math.h>


/************************** LYF Start ************************/
/***** 顶层 *****/
//任务二，沿边线走
void mission2(void){
	static int state_val,count,i,j,exit = NOT_OK,flag = NOT_OK;
	while(1){
		switch(state_val){
			case 0:{
			//	count++;
				//if(count==30){
					state_val++;
				//	count = 0;
					Laser_On;
				flag = NOT_OK;
			//	}
				break;
			}
			case 1:{
				if(flag==NOT_OK){
					move_derectly(mission2_point_list[0],mission2_point_list[1],3000);
					flag = OK;
				}
				//drawline(laser_ins.x_axis,laser_ins.y_axis,mission2_point_list[0],mission2_point_list[1],30);
				//drawline(laser_ins.x_axis,laser_ins.y_axis,mission2_point_list[0],mission2_point_list[1],50);
//				for(i=0;i<20;i++)
//					for(j=0;j<1000;j++);
				if(!IFMOVING(stepper1.motor_state)&&!IFMOVING(stepper2.motor_state)&&flag==OK)
					state_val++;
				break;
			}
			case 2:{
				drawline(laser_ins.x_axis,laser_ins.y_axis,mission2_point_list[2],mission2_point_list[3],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				state_val++;
				break;
			}
			case 3:{
				drawline(laser_ins.x_axis,laser_ins.y_axis,mission2_point_list[4],mission2_point_list[5],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				state_val++;
				break;
			}
			case 4:{
				drawline(laser_ins.x_axis,laser_ins.y_axis,mission2_point_list[6],mission2_point_list[7],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				state_val++;
				break;
			}
			case 5:{
				drawline(laser_ins.x_axis,laser_ins.y_axis,mission2_point_list[0],mission2_point_list[1],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				state_val=6;
				break;
			}
			case 6:{
				exit = OK;
				break;
			}
		}
		
		if(exit==OK){
			hmi_data_ins.mission_select = 0;
			state_val=0;
			exit = NOT_OK;
			break;
		}
	}
}

//任务三、四，沿纸靶走
void mission3(void){
	static int state_val,count,exit = NOT_OK;
	int i,j;
	while(1){
		switch(state_val){
			case 0:{
				if(cv_ins.rectangular_det_F==0){
					RECTANGULAR_DETECT;
				}
				if(cv_ins.rectangular_det_F==1){
					axis_cv2screen(cv_ins.rectangular_axis[0],cv_ins.rectangular_axis[1],
												 &cv_ins.rectangular_axis_screen[0],&cv_ins.rectangular_axis_screen[1]);
					axis_cv2screen(cv_ins.rectangular_axis[2],cv_ins.rectangular_axis[3],
												 &cv_ins.rectangular_axis_screen[3],&cv_ins.rectangular_axis_screen[3]);
					axis_cv2screen(cv_ins.rectangular_axis[4],cv_ins.rectangular_axis[5],
												 &cv_ins.rectangular_axis_screen[4],&cv_ins.rectangular_axis_screen[5]);
					axis_cv2screen(cv_ins.rectangular_axis[6],cv_ins.rectangular_axis[7],
												 &cv_ins.rectangular_axis_screen[6],&cv_ins.rectangular_axis_screen[7]);
					cv_ins.rectangular_det_F = 0;
					for(i=0;i<8;i++)
						cv_ins.rectangular_axis_screen[i]*=K;
					state_val++;
					LASER_DETECT;
					Laser_On;
				}
				break;
			}
			case 1:{
				//move_derectly(cv_ins.rectangular_axis_screen[0],cv_ins.rectangular_axis_screen[1],6000);
				drawline(laser_ins.x_axis,laser_ins.y_axis,cv_ins.rectangular_axis_screen[0],cv_ins.rectangular_axis_screen[1],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				HAL_Delay(4000);
				state_val++;
				break;
			}
			case 2:{
				//move_derectly(cv_ins.rectangular_axis_screen[2],cv_ins.rectangular_axis_screen[3],6000);
				drawline(laser_ins.x_axis,laser_ins.y_axis,cv_ins.rectangular_axis_screen[2],cv_ins.rectangular_axis_screen[3],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				HAL_Delay(4000);
				state_val++;
				break;
			}
			case 3:{
				//move_derectly(cv_ins.rectangular_axis_screen[4],cv_ins.rectangular_axis_screen[5],6000);
				drawline(laser_ins.x_axis,laser_ins.y_axis,cv_ins.rectangular_axis_screen[4],cv_ins.rectangular_axis_screen[5],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				HAL_Delay(4000);
				state_val++;
				break;
			}
			case 4:{
				//move_derectly(cv_ins.rectangular_axis_screen[6],cv_ins.rectangular_axis_screen[7],6000);
				drawline(laser_ins.x_axis,laser_ins.y_axis,cv_ins.rectangular_axis_screen[6],cv_ins.rectangular_axis_screen[7],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				HAL_Delay(4000);
				state_val++;
				break;
			}
			case 5:{
				//move_derectly(cv_ins.rectangular_axis_screen[0],cv_ins.rectangular_axis_screen[1],6000);
			  drawline(laser_ins.x_axis,laser_ins.y_axis,cv_ins.rectangular_axis_screen[0],cv_ins.rectangular_axis_screen[1],3000);
				for(i=0;i<20;i++)
					for(j=0;j<1000;j++);
				state_val++;
				HAL_Delay(4000);
				break;
			}
			case 6:{
				exit = OK;
				break;
			}
		}
		
		if(exit==OK){
			hmi_data_ins.mission_select = 0;
			state_val=0;
			exit = NOT_OK;
			break;
		}
	}
}

/*
 * @brief：校正屏幕位置
 * @param：None
 * @return： 无
   */
void calibration(void){
	static uint8_t state_val = 0,flag = NOT_OK,exit = NOT_OK;
	while(1){
		switch (state_val){
			case 0:{
				pid_reset(&pid_stp1);
				pid_reset(&pid_stp2);
				state_val = hmi_data_ins.set_pos;
				//退出
				if(state_val==6){
					state_val=2;
					break;
				}
				//继续
				else if(state_val==0)
					break;
				laser_set_target(&laser_ins,calibration_point_list[2*(state_val-1)],calibration_point_list[2*state_val-1]);
				Laser_On;
				state_val++;
				flag  =NOT_OK;
				break;
			}
			//校正原点
			case 1:{
				flag = focus_calibration_point(&laser_ins);
				if(flag==OK){
					flag = NOT_OK;
					state_val=0;
					HMISends("set_page.t0.txt=\"完成\"");
					hmi_data_ins.set_pos = 0;
				}
				break;
			}
			case 2:{
				exit = OK;
				break;
			}
		}
		if(exit==OK){
			hmi_data_ins.mission_select = 0;
			break;
			state_val=0;
		}
	}
}


void reset2origin(void){
	move_derectly(0,0,1000);
}

/*
 * @brief：将视觉坐标系转换到屏幕坐标系
 * @param：None
 * @return：None
   */
void axis_cv2screen(int cvx,int cvy,int *screenx,int *screeny){
	float x,y;
	x = (cvx - 137.5f)/SCREEN_K;
	y = -(cvy - 137.5f)/SCREEN_K;
	*screenx = x;
	*screeny = y;
}

void motor_reset2origin(void){
	float angle_x,angle_y;
	while(stepper1.position_ctnow!=0||stepper2.position_ctnow!=0){//stepper1.position_ctnow>1||stepper2.position_ctnow>1||stepper1.position_ctnow<-1||stepper2.position_ctnow<-1){
		if(!IFMOVING(stepper1.motor_state)&&stepper1.position_ctnow!=0){
			angle_y = stepper1.stepangle*stepper1.position_ctnow;
			if(fabs(angle_y)<0.05626f)
				angle_y = 0.05626f*stepper1.position_ctnow;
			StpDistanceSetBlocking(&stepper1,-angle_y,1000,1000);
		}
		if(!IFMOVING(stepper2.motor_state)&&stepper2.position_ctnow!=0) {
			angle_x = stepper2.stepangle*stepper2.position_ctnow;
			if(fabs(angle_x)<0.028126f)
				angle_x = 0.028126f*stepper2.position_ctnow;
			StpDistanceSetBlocking(&stepper2,-angle_x,1000,1000);
		}
	}
	Laser_On;
}

/*
 * @brief：激光点打在屏幕原点时，计算屏幕坐标系的原点（即电机与屏幕垂直时激光点坐标）
 * @param：None
 * @return：None
   */
void set_origin(void){
	float x0,y0,sqx,temp;
	temp = tanf(-stepper2.position_ctnow*stepper2.stepangle);
	sqx = L*L*(1+temp*temp);
	sqx = sqrtf(sqx);
	x0 = laser_ins.x_axis + L*temp;
	y0 = laser_ins.y_axis - sqx*tanf(-stepper1.position_ctnow*stepper1.stepangle);
	
	laser_ins.x0 = x0;
	laser_ins.y0 = y0;
}

void cal_axis(void){
	float x0,y0,sqx,temp;
	temp = tanf(-stepper2.position_ctnow*stepper2.stepangle);
	sqx = L*L*(1+temp*temp);
	sqx = sqrtf(sqx);
	x0 = L*temp;
	y0 = sqx*tanf(-stepper1.position_ctnow*stepper1.stepangle);
	
	laser_ins.x0 = x0;
	laser_ins.y0 = y0;
}
	

//电机插补
/*
 * @brief：转到插补目标坐标，并更新激光点坐标
 * @param：x，y	目标坐标
 * @return： 无
   */
void turn_coordinate(float x, float y,float tar_v)
{
  float angle_x, angle_y;
  float d_angx, d_angy;
  float sqx;
	//y+=50;
	//计算需要转到的角度
  sqx = sqrtf(L * L + x * x);
  angle_x = atanf(x / L) * 180 / PI;
  angle_y = atanf(y / sqx) * 180 / PI;
	
	//计算需要转过的角度
	d_angx = angle_x - stepper2.position_ctnow*stepper2.stepangle;
	d_angy = angle_y - stepper1.position_ctnow*stepper1.stepangle;
	
	if(fabs(d_angx)>stepper2.stepangle){
		StpDistanceSetBlocking(&stepper2,d_angx,500,tar_v);
		laser_ins.x_axis = x;
		}
	if(fabs(d_angy)>stepper1.stepangle){
		StpDistanceSetBlocking(&stepper1,d_angy,500,tar_v);
		laser_ins.y_axis = y;
	}
}

/*
 * @brief：直线运动插补
 * @param：起点坐标（X0, Y0），终点坐标（Xe, Ye）
 * @param：tar_v	调节电机速度，数字越大越慢，若只移动一个轴，数字尽量给大，500速度适中
 * @return： 无
   */
void drawline(int X0,int Y0,int Xe,int Ye,float tar_v){
	int NXY;              //总步数
  int Fm = 0;           //偏差
  int Xm , Ym; //当前坐标
  uint8_t XOY;            //象限
//	X0 = -X0;
//	Y0 = -Y0;
	if(X0!=laser_ins.x_axis)X0 = laser_ins.x_axis;
	if(Y0!=laser_ins.y_axis)Y0 = laser_ins.y_axis;
	//Ye +=50;
	Xm = X0;
	Ym = Y0;
  Xe = Xe - X0;
  Ye = Ye - Y0;
  NXY = (fabsf((float)Xe) + fabsf((float)Ye)) / STEP_LEN;

  if(Xe > 0 && Ye >= 0) XOY = 1;
  else if(Xe <= 0 && Ye > 0) XOY = 2;
  else if(Xe < 0 && Ye <= 0) XOY = 3;
  else if(Xe >= 0 && Ye < 0) XOY = 4;

  while(NXY > 0)
  {
		if(!IFMOVING(stepper1.motor_state)&&!IFMOVING(stepper2.motor_state)){
			switch (XOY)
			{
			case 1: (Fm >= 0) ? (Xm += STEP_LEN) : (Ym += STEP_LEN); break;
			case 2: (Fm <  0) ? (Xm -= STEP_LEN) : (Ym += STEP_LEN); break;
			case 3: (Fm >= 0) ? (Xm -= STEP_LEN) : (Ym -= STEP_LEN); break;
			case 4: (Fm <  0) ? (Xm += STEP_LEN) : (Ym -= STEP_LEN); break;
			default: break;
			}
			
			NXY -= 1;
			Fm = (Ym - Y0) * Xe - (Xm - X0) * Ye;
			turn_coordinate((float)Xm,(float)Ym,tar_v);
		}
	}	
}

/*
 * @brief：直接移动到目标点
 * @param：x,y
 * @param：tar_v	调节电机速度，数字越大越慢，1000速度适中
 * @return： 无
   */
void move_derectly(int x,int y,float tar_v){
	float angle_x, angle_y;
  float d_angx, d_angy;
  float sqx;
//	y+=50;
//	x=-x;
//	y=-y;
	//计算需要转到的角度
  sqx = sqrtf(L * L + x * x);
  angle_x = atanf(x / L) * 180 / PI;
  angle_y = atanf(y / sqx) * 180 / PI;
	
	//计算需要转过的角度
	d_angx = angle_x - stepper2.position_ctnow*stepper2.stepangle;
	d_angy = angle_y - stepper1.position_ctnow*stepper1.stepangle;
	
	if(fabs(d_angx)>stepper2.stepangle){
		StpDistanceSetBlocking(&stepper2,d_angx,500,tar_v);
		laser_ins.x_axis = x;
		}
	if(fabs(d_angy)>stepper1.stepangle){
		StpDistanceSetBlocking(&stepper1,d_angy,500,tar_v);
		laser_ins.y_axis = y;
	}
}

/***** 底层 *****/
void laser_init(laser *l,int x0,int y0){
	l->x_axis = x0;
	l->y_axis = y0;
	l->x_cur = 0;
	l->x_target = 0;
	l->y_cur = 0;
	l->y_target = 0;
	l->x0 = 0;
	l->y0 = 0;
}

//设置激光点目标位置
void laser_set_target(laser *l,int x,int y){
	l->x_target = x;
	l->y_target = y;
}

//使用PID逼近校正点
int focus_calibration_point(laser *l){
	static int count;
	
	//增量式PID
//	pid_incremental(&pid_stp1,laser_ins.x_target-cv_ins.laser_axis[0],15,-15);
//	pid_incremental(&pid_stp2,laser_ins.y_target-cv_ins.laser_axis[1],15,-15);
	if(!IFMOVING(stepper1.motor_state))
		StpDistanceSetBlocking(&stepper1,pid_stp1.output,200,100);
	if(!IFMOVING(stepper2.motor_state))
		StpDistanceSetBlocking(&stepper2,pid_stp2.output,200,100);
	if(fabs(pid_stp1.err)<2&&fabs(pid_stp2.err)<2){
		count++;
	}
	else count = 0;
	if(count==30){
		count = 0;
		return OK;
	}
	return NOT_OK;
}

/**
 * @brief  开启PID
 * @param  if_reset 0：不reset pid结构体；1	reset pid结构体
 * @retval None
 */
void pid_start(int if_reset){
	PID_F = PID_START;
	if(if_reset==1){
		pid_reset(&pid_stp1);
		pid_reset(&pid_stp2);
	}
}

/**
 * @brief  关闭PID
 * @param  None
 * @retval None
 */
void pid_stop(void){
	PID_F = PID_STOP;
}

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
		p->err = err;
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
