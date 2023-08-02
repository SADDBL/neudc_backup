#include "connect.h"
#include "usart.h"
#include <stdio.h>
cv_data cv_ins = {0,LINE_MID_DATA,OUT_RANGE,OUT_RANGE};

/**
  * @brief  向串口屏发送字符串
  * @param  *buf1 字符串
  * @retval None
  */
void HMISends(char *buf1){
	int len = strlen(buf1);
	HAL_UART_Transmit(&HMIUart,(uint8_t*)buf1,len,0xffff);
	HMISendb(0xff);
}

/**
  * @brief  发送3个相同字节
  * @param  k 字符
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
            /***** 定长度数据处理 *****/
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
        // 等待接收数据头
        else
        {
            // 数据头判断，确定接收数据类型
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
// 串口回调函数模板
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

//DMA接收陀螺仪数据
//void USART3_IRQHandler(void)
//{
//	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))//idle标志被置位
//	{ 
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除标志位
//		HAL_UART_DMAStop(&huart3); //  停止DMA传输，防止
//	}
//	HAL_UART_Receive_DMA(&huart3,jy901_ins.DataBuf,BufLen);//重新打开DMA接收
//  HAL_UART_IRQHandler(&huart3);
//}

void get_yaw_az(JY901 *j){
	j->yaw = ((short)(j->DataBuf[40]<<8|j->DataBuf[39]))*180/32768.0;
	j->wz = ((short)(j->DataBuf[29]<<8|j->DataBuf[28]))*2000/32768.0;
//	M.beta = (j->wz-j->wz_last)/CONTROL_CYCLE*1000*PI/180;
//	M.Speed = j->wz;
	j->wz_last = j->wz;
}

