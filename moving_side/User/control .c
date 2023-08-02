#include "control.h"

/************************** LYF Start ************************/
//����岹
void drawline(float x0,float y0,float xe,float ye){
	
}



/***** PID�ײ� *****/
/**
 * @brief  ����PID����
 * @param  *p PID�ṹ��
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
 * @brief  λ��ʽPIDʵ�ֺ���
 * @param  *p PID�ṹ��
 * @param  err ���ֵ��err = current - target
 * @param  outMax ������ֵ
 * @param  outMin �����Сֵ
 * @param  i_Max ��������
 * @retval p->output����p->output�ڸú������Ѿ�����ֵ��
 */
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max){
//  if(a<1&&a>0)
//		p->err = first_order_filter(err,p->err_k1,a);
//	else if(a==1){
//		p->err = err;
//	}
	
    // �����ֱ���
    if (p->output_last > outMax || p->output_last < outMin)
    {
        if (p->output_last * p->err < 0) // errʹ���������ֵ��С
            p->i += p->err;
        else
            p->i = p->i;
    }
    else
        p->i += p->err;
    // �����޷�
    if (p->i > i_Max)
        p->i = i_Max;
    else if (p->i < -i_Max)
        p->i = -i_Max;
		p->d = p->err - p->err_k1;
    p->output = p->kp * p->err + p->ki * p->i + p->kd * (p->d);
    p->err_k1 = p->err;
    p->output_last = p->output;

    // ����޷�
    if (p->output > outMax)
        p->output = outMax;
    if (p->output < outMin)
        p->output = outMin;

    return p->output;
}

/**
 * @brief  ����ʽPIDʵ�ֺ���
 * @param  *p PID�ṹ��
 * @param  err ���ֵ��err = current - target
 * @param  outMax ������ֵ
 * @param  outMin �����Сֵ
 * @retval p->output����p->output�ڸú������Ѿ�����ֵ��
 */
float pid_incremental(pid *p, float err, float outMax, float outMin){
//	int i_flag=1;
	p->err = err;
//	if(fabs(err)>80) i_flag = 0;
	// �����ֱ���
  if (p->output_last > outMax || p->output_last < outMin){
		if (p->output_last * p->err < 0) // errʹ���������ֵ��С
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
	
	// ����޷�
  if (p->output > outMax)
		p->output = outMax;
  if (p->output < outMin)
    p->output = outMin;

   return p->output;
}

/**
  * @brief  PID�ṹ�帴λ�����л�����ʱ����
  * @param  *pid_controller PID�ṹ��
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
  * @brief  һ���˲�
  * @param  new_value ��ֵ
	* @param  last_value ��һ�ε�ֵ
  * @param  a �˲�ϵ��
  * @retval �˲����ֵ
  */
float first_order_filter(float new_value,float last_value,float a){
	//a��ȡֵ�������㷨�������ȣ�aԽ���²ɼ���ֵռ��Ȩ��Խ���㷨Խ��������ƽ˳�Բ�
	//�෴��aԽС���²ɼ���ֵռ��Ȩ��ԽС�������Ȳ��ƽ˳�Ժá�
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}


/************************** LYF End **************************/
