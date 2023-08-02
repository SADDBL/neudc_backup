#include "connect.h"
#include "usart.h"
#include <stdio.h>
cv_data cv_ins = {0,LINE_MID_DATA,OUT_RANGE,OUT_RANGE};

/**
  * @brief  �򴮿��������ַ���
  * @param  *buf1 �ַ���
  * @retval None
  */
void HMISends(char *buf1){
	int len = strlen(buf1);
	HAL_UART_Transmit(&HMIUart,(uint8_t*)buf1,len,0xffff);
	HMISendb(0xff);
}

/**
  * @brief  ����3����ͬ�ֽ�
  * @param  k �ַ�
  * @retval None
  */
void HMISendb(uint8_t k){
	uint8_t str[3];
	str[0] = k;
	str[1] = k;
	str[2] = k;
	HAL_UART_Transmit(&HMIUart,str,3,0xffff);
}

UartBuff UartBuff_Ins;
void UartDateHandler(UartBuff *UartBuff1)
{
    if ((UartBuff1->Flag != 0x80) && (UartBuff1->Flag != 0xee))
    {
        if (UartBuff1->Flag == 0x40)
        {
            /***** ���������ݴ��� *****/
            if(UartBuff1->Pointer<13){
							UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
						}
						if(UartBuff1->Pointer==13){
							if(UartBuff1->Uart_Data[5]=='x'&&UartBuff1->Uart_Data[9]=='y'){
								UartBuff1->Flag = 0x80;
							}
							else UartBuff1->Flag=0xee;
						}
            /*****  *****/

        }
        // �ȴ���������ͷ
        else
        {
            // ����ͷ�жϣ�ȷ��������������
            if (UartBuff1->Rxbuf == 'f')
            {
                UartBuff1->Flag = 0x40;
                UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
            }
            else
            {
                UartBuff1->Flag = 0xee;
            }
        }
    }
}

UartBuff UartBuff_Ins1;
int mission_select;
// ���ڻص�����ģ��
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART2){
			if(UartBuff_Ins1.Flag==0x01){
				mission_select = UartBuff_Ins1.Rxbuf-'0';
				UartBuff_Ins1.Flag=0x00;
			}
			else if(UartBuff_Ins1.Flag==0x02){
				if(UartBuff_Ins1.Pointer<4){
					UartBuff_Ins1.Uart_Data[UartBuff_Ins1.Pointer++] = UartBuff_Ins1.Rxbuf;
				}
				if(UartBuff_Ins1.Pointer==4){
					
				}
			}
			else {
				if(UartBuff_Ins1.Rxbuf == 'm'){
					UartBuff_Ins1.Flag = 0x01;
				}
				else if(UartBuff_Ins1.Rxbuf == 's'){
					UartBuff_Ins1.Flag = 0x02;
				}
			}
			HAL_UART_Receive_IT(&huart2, &(UartBuff_Ins1.Rxbuf), 1);
		}
}

//DMA��������������
//void USART3_IRQHandler(void)
//{
//	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))//idle��־����λ
//	{ 
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//�����־λ
//		HAL_UART_DMAStop(&huart3); //  ֹͣDMA���䣬��ֹ
//	}
//	HAL_UART_Receive_DMA(&huart3,jy901_ins.DataBuf,BufLen);//���´�DMA����
//  HAL_UART_IRQHandler(&huart3);
//}

void get_yaw_az(JY901 *j){
	j->yaw = ((short)(j->DataBuf[40]<<8|j->DataBuf[39]))*180/32768.0;
	j->wz = ((short)(j->DataBuf[29]<<8|j->DataBuf[28]))*2000/32768.0;
//	M.beta = (j->wz-j->wz_last)/CONTROL_CYCLE*1000*PI/180;
//	M.Speed = j->wz;
	j->wz_last = j->wz;
}

