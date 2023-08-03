/*        	������      ������
#      �������� �� �������� �� ��
#      ��                 		��
#      ��       ������       	��
#      ��  ���Щ�      ���Щ�  ��
#      ��                 		��
#      ��       ���ة�       	��
#      ��                 		��
#      ����������         ������
#          		��         ��
#         		��         ��
#          		��         ��
#          		��         ��������������������������������
#         		��                        								��
#         		��                        								������
#          		��                        								������
#          		��                        								��
#         		 ������  ��  �����������������Щ�����  ����
#         		   ���� ����       					�� ���� ����
#         		 �������ة�����       			�������ة�����
#                ���ޱ���
#                ������BUG!
*/

#include "stepper.h"
#include "macro.h"
// �ٶ�����ٶȵ��������һЩ���⣬����Ҫ�ٲ��ԸĽ�
uint16_t Accel_Max = 65535;
uint16_t Velocity_Max = 65500;

int midway = 0; // ��������Ҫ���м���

//------------------------------------------------ȫ�ֱ���&����------------------------------------------------------//

const int Freq_t = 170000000 / 170;		// ����Ԥ��Ƶ֮��ʱ����Ԫ�ĵ���
const uint16_t Tim_Period = 65535;		// ��ʱ������
const uint16_t Tim_Prescaler = 169;		// ��ʱ��Ԥ��Ƶϵ��
const uint16_t PulseWide_Negetive = 15; // ������ʱ��
const uint16_t Min_Period = 950;//50			// �������
const uint16_t Max_Period = 1000;		// �����

//------------------------------------------------��ʼ������------------------------------------------------------//
/*******************************************************************************
 * @brief       ���ɲ����������
 * @param       motor_num: ��������ӿ����
 * @retval      ��
 *******************************************************************************/
void Init_Stepper(Stepper *stp, GPIO_TypeDef *stpdir_gpio_port, uint16_t stpdir_gpio_pin,
				  TIM_HandleTypeDef *stps_timer, uint16_t stps_ch, float angpp)
{
	stp->CC_FLAG = 0;
	stp->AcceAng = 0;
	stp->DirPin = stpdir_gpio_pin;
	stp->DirPort = stpdir_gpio_port;
	stp->EnPin = 0; // Ĭ������
	stp->EnPort = 0;
	stp->motor_state = 0;
	stp->period_now = 0;
	stp->period_rest = 0;
	stp->position_ang = 0;
	stp->position_ctnow = 0;
	stp->stepangle = angpp;
	stp->step_counter = 0;
	stp->step_threshold = 0;
	stp->StpChannel = stps_ch;
	stp->StpTim = stps_timer;
	stp->TagAngV = 0;
	stp->position_ctnow = 0;
	stp->CCR_ValSetter = 0;
	stp->accletperiod0 = 0;
	for (int i = 0; i < 6; i++)
	{
		stp->stepff_memory[i] = 0;
	}
	stp->acceldistancebuffer = 0;
}

//------------------------------------------------���ߺ���------------------------------------------------------//
void StepperInOC(Stepper *stp);

/*******************************************************************************
 * @brief       ȡ����ֵ
 * @param       Ҫȡ����ֵ����
 * @retval      ��
 *******************************************************************************/
uint32_t myabsi(int a)
{
	if (a >= 0)
	{
		return a;
	}
	else
	{
		return -a;
	}
}
float myabsf(float a)
{
	if (a >= 0)
	{
		return a;
	}
	else
	{
		return -a;
	}
}

/**
 * @brief       ���ü��������㵱ǰ���꣨�����ʾ��������
 * @param       Stepper* stepper: �������
 * @retval      ��ǰ�Ƕȣ�����λ�ã�
 */
float _Cal_PositionAng(struct Stepper *stepper)
{
	stepper->position_ang = stepper->stepangle * stepper->position_ctnow;
	return stepper->position_ang;
}
/*******************************************************************************
 * @brief       ���㵱ǰ���ٶȣ������ʾ��
 * @param       Stepper* stepper: �������
 * @retval      ������ٶ�
 *******************************************************************************/
float _Cal_AngV(struct Stepper *stepper)
{
	if ((IFMOVING(stepper->motor_state)))
	{
		if (IfCW((stepper->motor_state))) // CW
		{
			return (stepper->stepangle * Freq_t) / (stepper->period_now);
		}
		else // CCW
		{
			return -(stepper->stepangle * Freq_t) / (stepper->period_now);
		}
	}
	else
	{
		return 0;
	}
}

/*******************************************************************************
 * @brief       �رղ������
 * @param       motor_num: ��������ӿ����
 * @retval      ��
 *******************************************************************************/
void Stepper_Stop(struct Stepper *stepper)
{
	HAL_TIM_OC_Stop_IT(stepper->StpTim, stepper->StpChannel);
	//		stepper->AcceAng					= 0;
	stepper->motor_state = 0;
	stepper->period_now = 0;
	stepper->period_rest = 0;
	stepper->step_counter = 0;
}
/*******************************************************************************
 * @brief       ��ͣ�������
 * @param       motor_num: ��������ӿ����
 * @retval      ��
 *******************************************************************************/
void Stepper_Pause(struct Stepper *stepper)
{
	HAL_TIM_OC_Stop_IT(stepper->StpTim, stepper->StpChannel);
}
/*******************************************************************************
 * @brief       �������ӳ��ٶȼ��ٵ�Ŀ���ٶ���Ҫ�Ľ�λ��,���ص������岽��
 * @param				struct Stepper* stepper ��Ӧ�Ĳ������
 * @retval      int���ͣ�Ŀ��λ��
 *******************************************************************************/
uint32_t Calculator_AngleAcc(struct Stepper *stepper, float startv, float tagv) // �������ӳ��ٶȼ��ٵ�Ŀ���ٶ���Ҫ�Ľ�λ��
{
	static float mid = 0;
	mid = (tagv - startv);
	return myabsi(mid * (startv + 0.5f * mid)) / (stepper->AcceAng * stepper->stepangle);
}

/*******************************************************************************
 * @brief       ���������ٶȹ�������Ҫ������
								��һ�����岻���������
* @param				struct Stepper* stepper ��Ӧ�Ĳ������
								steps ��ǰ�˶����ܲ���
 * @retval      none
 *******************************************************************************/
void CalCulator_NextCount_Accel(struct Stepper *stepper) // ���������ٶȹ�������Ҫ������
{
	static uint32_t rest = 0;

	rest = ((2 * stepper->period_now) + stepper->period_rest) % (4 * stepper->step_counter + 1); // ���㵱ǰ������

	stepper->period_now = (stepper->period_now - (2 * stepper->period_now + stepper->period_rest) / (4 * stepper->step_counter + 1)); // �����µ����ڣ�ʹ�õ�����һ�ε�����

	stepper->period_rest = rest; // ������ֵ

	if (stepper->period_now <= Min_Period)
	{ // �����޷�

		stepper->period_now = Min_Period;
	}
}

/*******************************************************************************
 * @brief       ���������ٶȹ�������Ҫ������
								��һ�����岻���������
 * @param				struct Stepper* stepper ��Ӧ�Ĳ������
								steps ��ǰ�˶����ܲ���
 * @retval      none
 *******************************************************************************/
void CalCulator_NextCount_Deccel(struct Stepper *stepper) // ���������ٶȹ�������Ҫ������
{
	static uint32_t rest = 0;

	rest = ((/*2 **/ stepper->period_now) + stepper->period_rest) % (4 * stepper->step_counter - 1); // ���㵱ǰ������

	stepper->period_now = (stepper->period_now + (2 * stepper->period_now + stepper->period_rest) / (4 * stepper->step_counter - 1)); // �����µ����ڣ�ʹ�õ�����һ�ε�����

	stepper->period_rest = rest; // ������ֵ

	if (stepper->period_now >= Max_Period)
	{ // �����޷�

		stepper->period_now = Max_Period;
	}
}

/*******************************************************************************
 * @brief       ��ʼ��Ҫ�˶��Ĳ�������˶�����
								���ڲ���ǣ����߲��ᶯ
 * @param       Stepper* stepper ����ṹ��
 * @retval      ��
 *******************************************************************************/
void StpDistanceSetNonBlocking(struct Stepper *stepper, float angdistance, float accel, float tagv)
{
	if (myabsf(angdistance) > (stepper->stepangle)) // ���ڲ���ǣ����߲��ᶯ
	{
		if (!(IFMOVING(stepper->motor_state))) // �����´�ʱ�����ֹ
		{
			if (angdistance >= 0)
			{
				DIRTOCW(&stepper->motor_state) // ˳ʱ��
			}
			else
			{
				DIRTOCCW(&stepper->motor_state) // ��ʱ��
			}
			if (accel != stepper->AcceAng) // ���ٶȲ���ͬ������
			{
				stepper->AcceAng = accel;
				stepper->TagAngV = tagv;
				stepper->stepff_memory[0] = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
				stepper->acceldistancebuffer = stepper->stepff_memory[0];
				stepper->period_now = 0.676 * Freq_t * sqrt(2 * stepper->stepangle / stepper->AcceAng);
				if (stepper->period_now <= Min_Period)
				{ // �����޷�
					stepper->period_now = Min_Period;
				}
				else if (stepper->period_now >= Max_Period)
				{
					stepper->period_now = Max_Period;
				}
				stepper->accletperiod0 = stepper->period_now;
			}
			else if (tagv != stepper->TagAngV) // ���ٶ���ͬ���ٶȲ�ͬ
			{
				stepper->TagAngV = tagv;
				stepper->stepff_memory[0] = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
				stepper->acceldistancebuffer = stepper->stepff_memory[0];
				stepper->period_now = stepper->accletperiod0;
			}
			else // ���ٶ� �ٶ� ����ͬ
			{
				stepper->stepff_memory[0] = stepper->acceldistancebuffer;
				stepper->period_now = stepper->accletperiod0;
			}

			stepper->stepff_memory[1] = myabsi((int)angdistance / (stepper->stepangle));

			if (stepper->stepff_memory[0] < (stepper->stepff_memory[1] / 2))
			{ // ���Լӵ�����ٶ�
				stepper->stepff_memory[1] = stepper->stepff_memory[1] - 2 * stepper->stepff_memory[0];
				stepper->stepff_memory[2] = stepper->stepff_memory[0];
				StateToAcc(&stepper->motor_state);
				stepper->motor_state &= (~TYPEFLAG);
				stepper->step_threshold = stepper->stepff_memory[0];
			}
			else
			{ // �е����
				stepper->stepff_memory[0] = stepper->stepff_memory[1] / 2 + (stepper->stepff_memory[1]) % 2;
				stepper->stepff_memory[1] = stepper->stepff_memory[1] / 2;
				stepper->step_threshold = stepper->stepff_memory[0];
				stepper->motor_state &= (~TYPEFLAG);
				stepper->motor_state |= MIDPOINT; // ����˶�
				StateToAcc(&stepper->motor_state);
			}
			MoveSetBit(&stepper->motor_state); // ��λ���˶�״̬
			if ((GETDIRECT(stepper->motor_state)) == CW)
			{ // �˶�����
				HAL_GPIO_WritePin(stepper->DirPort, stepper->DirPin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(stepper->DirPort, stepper->DirPin, GPIO_PIN_RESET);
			}
		}
		else // �����´�ʱ������˶�
		{
			Stepper_Pause(stepper);
			if (((IfCW(stepper->motor_state)) && (angdistance >= 0)) || // �����ӵ��˶���ԭ�е��˶�ͬ����
				((!IfCW(stepper->motor_state)) && (angdistance < 0)))	// �������������ֵ
			{
				uint8_t state = GETRUNSTEP(stepper->motor_state);
				if (IFDIRCHANGE(stepper->motor_state)) // �����ǰ�˶�����Ҫ�����
				{
					DIRCHANGECANCEL(&stepper->motor_state) // ȡ����Ϊͬ��
				}
				if (state == ACCELING) // ���ڼ���
				{
					if (GETTYPE(stepper->motor_state) == FULLSTEP)
					{
						stepper->stepff_memory[1] += myabsi(angdistance) / stepper->stepangle; // ֱ�ӼӽǶ�
					}
					else if (GETTYPE(stepper->motor_state) == MIDPOINT)
					{
						uint32_t steps0 = myabsf(angdistance) / stepper->stepangle;
						uint32_t steps1 = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
						if (2 * steps1 > (stepper->stepff_memory[0] + stepper->stepff_memory[1] + steps1))
						{ // ���Ǽ��ٲ���
							stepper->stepff_memory[0] += (steps0 / 2 + steps0 % 2);
							stepper->stepff_memory[1] += (steps0 / 2);
						}
						else // �л���ƽ��
						{	 // �㹻���ٵ���
							STATETOFULLSTEP((&stepper->motor_state))
							stepper->stepff_memory[2] = steps1; // ��Ϊ�����ļӼ���
							stepper->stepff_memory[1] = steps0 - (steps1 - stepper->stepff_memory[0]);
							stepper->stepff_memory[0] = steps1;
						}
						stepper->step_threshold = stepper->stepff_memory[0]; // ���µĲ�������
					}
				}
				else if (state == RUNNING) // ��������
				{
					stepper->stepff_memory[1] += myabsf(angdistance) / stepper->stepangle; // ֱ�ӼӽǶ�
					stepper->step_threshold = stepper->stepff_memory[1];				   // ���µĲ�������
				}
				else if ((state == DECELING) || (state == STOPING)) // �ڼ��ٻ�Ҫֹͣ��
				{
					uint32_t steps0 = myabsf(angdistance) / stepper->stepangle;
					if (steps0 < stepper->step_counter)
					{
						stepper->step_counter = steps0;
					}
					else
					{
						uint32_t steps1 = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
						uint32_t steps2 = stepper->step_counter + steps0;
						if (2 * steps1 > steps2)
						{ // ���ٲ���
							STATETOMIDPOINT((&stepper->motor_state));
							stepper->stepff_memory[0] = (steps2 / 2 + steps2 % 2);
							stepper->stepff_memory[1] += (steps2 / 2);
						}
						else // �л���ƽ��
						{	 // �㹻���ٵ���
							STATETOFULLSTEP((&stepper->motor_state))
							stepper->stepff_memory[2] = steps1; // ��Ϊ�����ļӼ���
							stepper->stepff_memory[1] = steps2 - 2 * steps1 + stepper->step_counter;
							stepper->stepff_memory[0] = steps1;
						}
						stepper->step_threshold = stepper->stepff_memory[0]; // ���µĲ�������
						StateToAcc((&stepper->motor_state));				 // ���½�����ٶ�
					}
				}
			}
			else // �����ӵ��˶���ԭ�е��˶���ͬ����
			{
				Stepper_Pause(stepper);
				// �ܲ������ܴ��ڶ��߻�����һ��������
				DIRCHANGENEEDED((&stepper->motor_state)); // ��Ҫ����,�ڵ�ǰ�ļ��ٽ�����λΪ����Ҫ�����״̬
				StateToDec((&stepper->motor_state));	  // ��Ϊ����״̬
				uint32_t steps0 = myabsf(angdistance) / stepper->stepangle;
				uint32_t steps1 = 0;							   // ��¼�ڵ�ǰ������ٵ�0��Ҫ�Ĳ����������˶�ʱ����
				if ((GETRUNSTEP(stepper->motor_state)) == RUNNING) // ������
				{
					// stepper->step_threshold=stepper->stepff_memory[0];//���µĲ�������
					steps1 = stepper->stepff_memory[0];
					stepper->step_counter = steps1 = stepper->stepff_memory[0];
				}
				else if (((GETRUNSTEP(stepper->motor_state)) == ACCELING)) // ���ڼ���
				{
					steps1 = stepper->step_counter;
				}
				else if ((GETRUNSTEP(stepper->motor_state)) == DECELING) // �ڼ��ٻ�Ҫֹͣ��
				{
					steps1 = stepper->step_counter;
				}
				steps0 += steps1; // ���ϼ���Ҫ���Ĳ���,���������˶����ܲ���

				stepper->stepff_memory[0] = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
				stepper->stepff_memory[1] = steps0;
				if (stepper->stepff_memory[0] < (stepper->stepff_memory[1] / 2))
				{ // ���Լӵ�����ٶ�
					stepper->stepff_memory[1] = stepper->stepff_memory[1] - 2 * stepper->stepff_memory[0];
					stepper->stepff_memory[2] = stepper->stepff_memory[0];
					stepper->motor_state &= (~TYPEFLAG);
					stepper->step_threshold = stepper->stepff_memory[0];
				}
				else
				{ // �е����
					stepper->stepff_memory[0] = stepper->stepff_memory[1] / 2 + (stepper->stepff_memory[1]) % 2;
					stepper->stepff_memory[1] = stepper->stepff_memory[1] / 2;
					stepper->step_threshold = stepper->stepff_memory[0];
					stepper->motor_state &= (~TYPEFLAG);
					stepper->motor_state |= MIDPOINT; // ����˶�
				}
				stepper->period_buffer = 0.676 * Freq_t * sqrt(2 * stepper->AcceAng / stepper->TagAngV);
				if (stepper->period_buffer <= Min_Period)
				{ // �����޷�
					stepper->period_buffer = Min_Period;
				}
				else if (stepper->period_buffer >= Max_Period)
				{
					stepper->period_buffer = Max_Period;
				}
				if (stepper->step_counter == 0)
				{
					stepper->step_counter = 1;
				}
			}
		}
		// ��ȡ��ǰͨ������Ϣ,��ô��ʾ

		StepperInOC(stepper);
		//__HAL_TIM_SET_COMPARE(stepper->StpTim , stepper->StpChannel  , PulseWide_Negetive);
		HAL_TIM_OC_Start_IT(stepper->StpTim, stepper->StpChannel);
	}
}

/*******************************************************************************
 * @brief       ��ʼ��Ҫ�˶��Ĳ�������˶�����
								���ڲ���ǣ����߲��ᶯ
 * @param       Stepper* stepper ����ṹ��
 * @retval      ��
*******************************************************************************/
void StpDistanceSetBlocking(struct Stepper *stepper, float angdistance, float accel, float tagv)
{
	if (myabsf(angdistance) > (stepper->stepangle)) // ���ڲ���ǣ����߲��ᶯ
	{
		if (!(IFMOVING(stepper->motor_state))) // �����´�ʱ�����ֹ
		{
			if (angdistance >= 0)
			{
				DIRTOCW(&stepper->motor_state) // ˳ʱ��
			}
			else
			{
				DIRTOCCW(&stepper->motor_state) // ��ʱ��
			}
			if (accel != stepper->AcceAng) // ���ٶȲ���ͬ������
			{
				stepper->AcceAng = accel;
				stepper->TagAngV = tagv;
				stepper->stepff_memory[0] = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
				if (stepper->stepff_memory[0] < 1)
				{
					stepper->stepff_memory[0] = 1;
				}
				stepper->acceldistancebuffer = stepper->stepff_memory[0];
				stepper->period_now = 0.676 * Freq_t * sqrt(2 * stepper->stepangle / stepper->AcceAng);
				if (stepper->period_now <= Min_Period)
				{ // �����޷�
					stepper->period_now = Min_Period;
				}
				else if (stepper->period_now >= Max_Period)
				{
					stepper->period_now = Max_Period;
				}
				stepper->accletperiod0 = stepper->period_now;
			}
			else if (tagv != stepper->TagAngV) // ���ٶ���ͬ���ٶȲ�ͬ
			{
				stepper->TagAngV = tagv;
				stepper->stepff_memory[0] = Calculator_AngleAcc(stepper, 0, stepper->TagAngV);
				if (stepper->stepff_memory[0] < 1)
				{
					stepper->stepff_memory[0] = 1;
				}
				stepper->acceldistancebuffer = stepper->stepff_memory[0];
				stepper->period_now = stepper->accletperiod0;
			}
			else // ���ٶ� �ٶ� ����ͬ
			{
				stepper->stepff_memory[0] = stepper->acceldistancebuffer;
				stepper->period_now = stepper->accletperiod0;
			}
			stepper->stepff_memory[1] = myabsi((int)angdistance / (stepper->stepangle));
			if (stepper->stepff_memory[0] < (stepper->stepff_memory[1] / 2))
			{ // ���Լӵ�����ٶ�
				stepper->stepff_memory[1] = stepper->stepff_memory[1] - 2 * stepper->stepff_memory[0];
				stepper->stepff_memory[2] = stepper->stepff_memory[0];
				StateToAcc(&stepper->motor_state);
				stepper->motor_state &= (~TYPEFLAG);
				stepper->step_threshold = stepper->stepff_memory[0];
			}
			else
			{ // �е����
				stepper->stepff_memory[0] = stepper->stepff_memory[1] / 2 + (stepper->stepff_memory[1]) % 2;
				stepper->stepff_memory[1] = stepper->stepff_memory[1] / 2;
				stepper->step_threshold = stepper->stepff_memory[0];
				stepper->motor_state &= (~TYPEFLAG);
				stepper->motor_state |= MIDPOINT; // ����˶�
				StateToAcc(&stepper->motor_state);
			}
			MoveSetBit(&stepper->motor_state); // ��λ���˶�״̬
			if ((GETDIRECT(stepper->motor_state)) == CW)
			{ // �˶�����
				HAL_GPIO_WritePin(stepper->DirPort, stepper->DirPin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(stepper->DirPort, stepper->DirPin, GPIO_PIN_RESET);
			}
		}
		else // �����´�ʱ������˶�
		{
			if ((stepper->AcceAng == accel) && (stepper->TagAngV == tagv)) // ͬ���ٶ�ͬ�����ٶ�
			{
				Stepper_Pause(stepper);										// ����״̬���쳣�䶯
				if (((IfCW(stepper->motor_state)) && (angdistance >= 0)) || // �����ӵ��˶���ԭ�е��˶�ͬ����
					((!IfCW(stepper->motor_state)) && (angdistance < 0)))	// �������������ֵ
				{
					if (IFDIRCHANGE(stepper->motor_state)) // �����ǰ�˶�����Ҫ�����
					{
						DIRCHANGECANCEL(&stepper->motor_state) // ȡ����Ϊͬ��
					}
					uint8_t state = GETRUNSTEP(stepper->motor_state); // ��ȡ�˶��Ľ���
					if (state == ACCELING)							  // ���ڼ���
					{
						uint32_t steps0 = myabsf(angdistance) / stepper->stepangle; // �����ӵĲ���
						// uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );//��ȫ������Ҫ�Ĳ���
						uint32_t steps1 = stepper->acceldistancebuffer;	  // ��ȫ������Ҫ�Ĳ���
						uint32_t steps2 = steps0 + stepper->step_counter; // �����˶��Ĳ���
						if (steps2 >= (2 * steps1))
						{
							STATETOFULLSTEP((&stepper->motor_state));
							stepper->stepff_memory[0] = steps1;
							stepper->stepff_memory[1] = steps2 - 2 * steps1; // steps0-steps1-(steps1-stepper->step_counter) ;//ֱ�ӼӽǶ�
							stepper->stepff_memory[2] = steps1;
							stepper->step_threshold = stepper->stepff_memory[0]; // ���µĲ�������
						}
						else if (steps0 <= steps1) // ����Ҫ������
						{
							stepper->step_counter = steps0; // �µ��˶�����Ϊ����
							StateToDec((&stepper->motor_state));
						}
						else
						{
							STATETOMIDPOINT((&stepper->motor_state));
							stepper->stepff_memory[0] = steps2 / 2 + steps2 % 2;
							stepper->stepff_memory[1] = steps2 / 2;
							stepper->step_threshold = stepper->stepff_memory[0]; // ���µĲ�������
						}
					}
					else if (state == RUNNING) // ��������
					{
						uint32_t steps0 = myabsf(angdistance) / stepper->stepangle; // �����ӵĲ���
						//					uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );//��ȫ������Ҫ�Ĳ���
						uint32_t steps1 = stepper->acceldistancebuffer; // ��ȫ������Ҫ�Ĳ���
						if (steps0 >= steps1)
						{
							stepper->step_threshold = steps0 - steps1; // ���µĲ�������
						}
						else
						{
							stepper->step_counter = steps0;
							StateToDec((&stepper->motor_state));
						}
					}
					else if ((state == DECELING) || (state == STOPING)) // �ڼ��ٻ�Ҫֹͣ��
					{
						uint32_t steps0 = myabsf(angdistance) / stepper->stepangle; // �����ӵĲ���
						//						uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );//��ȫ������Ҫ�Ĳ���
						if (steps0 < stepper->step_counter) // �¼ӵĲ����ڼ����н���
						{
							stepper->step_counter = steps0;
						}
						else
						{
							uint32_t steps1 = stepper->acceldistancebuffer;	  // ��ȫ������Ҫ�Ĳ���
							uint32_t steps2 = steps0 + stepper->step_counter; // �����˶��Ĳ��� stepper->step_counter��Ϊ�²���
							if (steps0 <= stepper->step_counter)			  // ����Ҫ������
							{
								stepper->step_counter = steps0;
							}
							else if (steps2 >= (2 * steps1))
							{
								STATETOFULLSTEP((&stepper->motor_state));
								StateToAcc((&stepper->motor_state));
								stepper->stepff_memory[0] = steps1;
								stepper->stepff_memory[1] = steps2 - 2 * steps1; // steps0-steps1-(steps1-stepper->step_counter) ;//ֱ�ӼӽǶ�
								stepper->stepff_memory[2] = steps1;
								stepper->step_threshold = stepper->stepff_memory[0]; // ���µĲ�������
								if (stepper->step_threshold == stepper->step_counter)
								{
									StateToRun((&stepper->motor_state));
									stepper->step_counter = 0;
									stepper->step_threshold = stepper->stepff_memory[1];
								}
							}
							else
							{
								STATETOMIDPOINT((&stepper->motor_state));
								StateToAcc((&stepper->motor_state));
								stepper->stepff_memory[0] = steps2 / 2 + steps2 % 2;
								stepper->stepff_memory[1] = steps2 / 2;
								stepper->step_threshold = stepper->stepff_memory[0]; // ���µĲ�������
							}
						}
					}
				}
				else // �����ӵ��˶���ԭ�е��˶���ͬ����
				{
					Stepper_Pause(stepper);
					// �ܲ������ܴ��ڶ��߻�����һ��������
					DIRCHANGENEEDED((&stepper->motor_state)); // ��Ҫ����,�ڵ�ǰ�ļ��ٽ�����λΪ����Ҫ�����״̬
					StateToDec((&stepper->motor_state));	  // ��Ϊ����״̬
					uint32_t steps0 = myabsf(angdistance) / stepper->stepangle;
					uint32_t steps1 = 0;							   // ��¼�ڵ�ǰ������ٵ�0��Ҫ�Ĳ����������˶�ʱ����
					if ((GETRUNSTEP(stepper->motor_state)) == RUNNING) // ������
					{
						// stepper->step_threshold=stepper->stepff_memory[0];//���µĲ�������
						steps1 = stepper->stepff_memory[0];
						stepper->step_counter = steps1 = stepper->stepff_memory[0];
					}
					else if (((GETRUNSTEP(stepper->motor_state)) == ACCELING)) // ���ڼ���
					{
						steps1 = stepper->step_counter;
					}
					else if ((GETRUNSTEP(stepper->motor_state)) == DECELING) // �ڼ��ٻ�Ҫֹͣ��
					{
						steps1 = stepper->step_counter;
					}
					steps0 += steps1; // ���ϼ���Ҫ���Ĳ���,���������˶����ܲ���

					//				stepper->stepff_memory[0]= Calculator_AngleAcc(stepper , 0 , stepper->TagAngV);
					stepper->stepff_memory[0] = stepper->acceldistancebuffer;
					stepper->stepff_memory[1] = steps0;
					if (stepper->stepff_memory[0] < (stepper->stepff_memory[1] / 2))
					{ // ���Լӵ�����ٶ�
						stepper->stepff_memory[1] = stepper->stepff_memory[1] - 2 * stepper->stepff_memory[0];
						stepper->stepff_memory[2] = stepper->stepff_memory[0];
						stepper->motor_state &= (~TYPEFLAG);
						stepper->step_threshold = stepper->stepff_memory[0];
					}
					else
					{ // �е����
						stepper->stepff_memory[0] = stepper->stepff_memory[1] / 2 + (stepper->stepff_memory[1]) % 2;
						stepper->stepff_memory[1] = stepper->stepff_memory[1] / 2;
						stepper->step_threshold = stepper->stepff_memory[0];
						stepper->motor_state &= (~TYPEFLAG);
						stepper->motor_state |= MIDPOINT; // ����˶�
					}
					stepper->period_buffer = 0.676 * Freq_t * sqrt(2 * stepper->AcceAng / stepper->TagAngV);
					if (stepper->period_buffer <= Min_Period)
					{ // �����޷�
						stepper->period_buffer = Min_Period;
					}
					else if (stepper->period_buffer >= Max_Period)
					{
						stepper->period_buffer = Max_Period;
					}
					if (stepper->step_counter == 0)
					{
						stepper->step_counter = 1;
					}
				}
			}
		}
		// ��ȡ��ǰͨ������Ϣ,��ô��ʾ

		StepperInOC(stepper);
		//__HAL_TIM_SET_COMPARE(stepper->StpTim , stepper->StpChannel  , PulseWide_Negetive);
		HAL_TIM_OC_Start_IT(stepper->StpTim, stepper->StpChannel);
	}
	else
	{
		Stepper_Stop(stepper);
	}
}

/*******************************************************************************
 * @brief       �ڶ�ʱ���ж����޸Ĳ���
								���˶�����״̬���в�ѯ�˶�����
								��״̬����ʱ����
								ֻ�����޸�״̬
 * @param       Stepper* stepper ����ṹ��
 * @retval      ��
 *******************************************************************************/
void Stepper_Handler(struct Stepper *stepper)
{

	switch ((GETRUNSTEP(stepper->motor_state)))
	{
	case ACCELING: // ���ٶ�
	{
		stepper->step_counter++;
		if (GETDIRECT(stepper->motor_state))
		{
			stepper->position_ctnow += 1;
		}
		else
		{
			stepper->position_ctnow -= 1;
		}
		if (stepper->step_counter < stepper->step_threshold)
		{
			//CalCulator_NextCount_Accel(stepper);
		}
		else if (stepper->step_counter >= stepper->step_threshold)
		{ // ״̬��������������
			//CalCulator_NextCount_Accel(stepper);
			if ((GETTYPE(stepper->motor_state)) == FULLSTEP)
			{ // ƽ��
				StateToRun(&stepper->motor_state);
				// stepper->period_now = stepper->period_buffer ;
				stepper->step_threshold = stepper->stepff_memory[1];
				stepper->step_counter = 0;
			}
			else if ((GETTYPE(stepper->motor_state)) == MIDPOINT)
			{									   // �ⶥ
				StateToDec(&stepper->motor_state); // ״̬��������
				stepper->step_counter = stepper->stepff_memory[1];
			}
		}
		break;
	}
	case RUNNING: // ���ٶ�
	{
		stepper->step_counter++;
		if (GETDIRECT(stepper->motor_state))
		{
			stepper->position_ctnow += 1;
		}
		else
		{
			stepper->position_ctnow -= 1;
		}
		if (stepper->step_counter >= stepper->step_threshold)
		{ // ״̬��������������
			if ((GETTYPE(stepper->motor_state)) == FULLSTEP)
			{									   // ƽ����ⶥ
				StateToDec(&stepper->motor_state); // ״̬��������
				stepper->step_counter = stepper->stepff_memory[2];
			}
		}
		break;
	}
	case DECELING: // ���ٶ�
	{

		if (GETDIRECT(stepper->motor_state))
		{
			stepper->position_ctnow += 1;
		}
		else
		{
			stepper->position_ctnow -= 1;
		}
		//CalCulator_NextCount_Deccel(stepper);
		if (stepper->step_counter <= 1)
		{
			if ((GETTYPE(stepper->motor_state)) == FULLSTEP || (GETTYPE(stepper->motor_state)) == MIDPOINT)
			{									   // ƽ����ⶥ
				//StateToStp(&stepper->motor_state); // ״̬����ֹͣ
				Stepper_Stop(stepper);
			}
		}
		stepper->step_counter--;
		break;
	}
//	case STOPING: // ֹͣ��
//	{
//		if (GETDIRECT(stepper->motor_state))
//		{
//			stepper->position_ctnow += 1;
//		}
//		else
//		{
//			stepper->position_ctnow -= 1;
//		}
//		if (IFDIRCHANGE(stepper->motor_state)) // Ҫ���򣬴�ͷ��ʼ
//		{
//			DIRCHANGECANCEL((&stepper->motor_state))
//			if (IfCW(stepper->motor_state))
//			{
//				DIRTOCCW((&stepper->motor_state));
//			}
//			else
//			{
//				DIRTOCW((&stepper->motor_state));
//			}
//			StateToAcc((&stepper->motor_state)) // ����
//				stepper->step_threshold = stepper->stepff_memory[0];
//			HAL_GPIO_TogglePin(stepper->DirPort, stepper->DirPin);
//			stepper->period_now = stepper->period_buffer;
//		}
//		else
//		{
//			Stepper_Stop(stepper);
//		}
//		break;
//	}
	default:
	{
		break;
	}
	}
}

/*******************************************************************************
 * @brief       �ڶ�ʱ���ж����޸Ĳ���
								���˶�����״̬���в�ѯ�˶�����
								��״̬����ʱ����
								ֻ�����޸�״̬
 * @param       Stepper* stepper ����ṹ��
 * @retval      ��
 *******************************************************************************/
void StepperInOC(Stepper *stp)
{
	uint16_t Comparenow = __HAL_TIM_GET_COMPARE(stp->StpTim, stp->StpChannel);
	if (stp->CC_FLAG)
	{
		Stepper_Handler(stp);
		stp->CCR_ValSetter = (Tim_Period - stp->TagAngV);//stp->period_now + PulseWide_Negetive);
		if (Comparenow < stp->CCR_ValSetter)
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, Comparenow + stp->TagAngV);//(stp->period_now - PulseWide_Negetive));
		}
		else
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, (stp->period_now - stp->CCR_ValSetter));
		}
	}
	else
	{
		if (Comparenow < stp->CCR_ValSetter)
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, (Comparenow + PulseWide_Negetive));
		}
		else
		{
			__HAL_TIM_SET_COMPARE(stp->StpTim, stp->StpChannel, (PulseWide_Negetive - stp->CCR_ValSetter));
		}
	}
	stp->CC_FLAG = !stp->CC_FLAG;
}

/*******************************************************************************
 * @brief       ��ʱ���Ƚ��жϻص�
 * @param       ��ʱ��ͨ��
 * @retval      ��
 *******************************************************************************/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		StepperInOC(&stepper1);
	}
	if (htim == &htim2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		StepperInOC(&stepper2);
	}
}
