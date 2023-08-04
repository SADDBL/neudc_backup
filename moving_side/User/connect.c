#include "connect.h"
#include "usart.h"
#include <stdio.h>
#include "macro.h"


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

void HMIDateHandler(UartBuff *HMIIns){
	if ((HMIIns->Flag != 0x81)&&(HMIIns->Flag != 0x80) && (HMIIns->Flag != 0xee))
   {
        if (HMIIns->Flag == 0x03)
        {
            /***** 定长度数据处理 *****/
            if(HMIIns->Pointer<2){
							HMIIns->Uart_Data[HMIIns->Pointer++] = HMIIns->Rxbuf;
						}
						if(HMIIns->Pointer==2){
							if(HMIIns->Uart_Data[0]=='m'){
								HMIIns->Flag = 0x80;
							}
							else HMIIns->Flag=0xee;
						}
            /*****  *****/

        }
				else if (HMIIns->Flag == 0x04)
        {
            /***** 定长度数据处理 *****/
            if(HMIIns->Pointer<2){
							HMIIns->Uart_Data[HMIIns->Pointer++] = HMIIns->Rxbuf;
						}
						if(HMIIns->Pointer==2){
							if(HMIIns->Uart_Data[0]=='c'){
								HMIIns->Flag = 0x81;
							}
							else HMIIns->Flag=0xee;
						}
            /*****  *****/

        }
        // 等待接收数据头
        else
        {
            // 数据头判断，确定接收数据类型
            if (HMIIns->Rxbuf == 'p')
            {
                HMIIns->Flag = 0x01;
                //HMIIns->Uart_Data[HMIIns->Pointer++] = HMIIns->Rxbuf;
            }
						else if (HMIIns->Rxbuf == 's')
            {
                HMIIns->Flag = 0x02;
                //HMIIns->Uart_Data[HMIIns->Pointer++] = HMIIns->Rxbuf;
            }
						else if (HMIIns->Rxbuf == 'm')
            {
                HMIIns->Flag = 0x03;
                HMIIns->Uart_Data[HMIIns->Pointer++] = HMIIns->Rxbuf;
            }
						else if(HMIIns->Rxbuf == 'c'){
							 HMIIns->Flag = 0x04;
               HMIIns->Uart_Data[HMIIns->Pointer++] = HMIIns->Rxbuf;
						}
            else
            {
                HMIIns->Flag = 0xee;
            }
        }
				//接收暂停指令
				if(HMIIns->Flag == 0x01){
					hmi_data_ins.stop_F = PAUSE;
					HMIIns->Flag = 0x0e;
					HMIIns->Pointer = 0;
				}
				//接收暂停指令
				else if(HMIIns->Flag == 0x02){
					hmi_data_ins.stop_F = RESTART;
					HMIIns->Flag = 0x0e;
					HMIIns->Pointer = 0;
				}
    }
		//数据处理
		if(HMIIns->Flag==0x80){
			HMIIns->Flag = 0x0e;
			HMIIns->Pointer = 0;
			hmi_data_ins.mission_select = HMIIns->Uart_Data[1]-'0';
		}
		else if(HMIIns->Flag==0x81){
			HMIIns->Flag = 0x0e;
			HMIIns->Pointer = 0;
			hmi_data_ins.set_pos = HMIIns->Uart_Data[1]-'0';
		}
		//故障处理
		if(HMIIns->Flag==0xee){
			HMIIns->Flag = 0x0e;
			HMIIns->Pointer = 0;
		}
}

void UartDateHandler(UartBuff *UartBuff1)
{
    if ((UartBuff1->Flag != 0x80) && (UartBuff1->Flag != 0xee))
    {
        if (UartBuff1->Flag == 0x40)
        {
            /***** 定长度数据处理 *****/
            if(UartBuff1->Pointer<7){
							UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
						}
						if(UartBuff1->Pointer==7){
							if(UartBuff1->Uart_Data[0]=='j'){
								UartBuff1->Flag = 0x80;
							}
							else UartBuff1->Flag=0xee;
						}
            /*****  *****/

        }
				else if(UartBuff1->Flag == 0x41){
					if(UartBuff1->Pointer<25){
							UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
						}
						if(UartBuff1->Pointer==25){
							if(UartBuff1->Uart_Data[0]=='r'){
								UartBuff1->Flag = 0x81;
							}
							else UartBuff1->Flag=0xee;
						}
				}
        // 等待接收数据头
        else
        {
            // 数据头判断，确定接收数据类型
            if (UartBuff1->Rxbuf == 'j')
            {
                UartBuff1->Flag = 0x40;
                UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
            }
						else if (UartBuff1->Rxbuf == 'r')
            {
                UartBuff1->Flag = 0x41;
                UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
            }
            else
            {
                UartBuff1->Flag = 0xee;
            }
        }
    }
		//数据处理
		if(UartBuff1->Flag==0x80){
			UartBuff1->Flag = 0x0e;
			UartBuff1->Pointer = 0;
			sscanf((char*)UartBuff1->Uart_Data,"j%3d%3d",&cv_ins.laser_axis[0],&(cv_ins.laser_axis[1]));
			laser_ins.x_cur = cv_ins.laser_axis[0];
			laser_ins.y_cur = cv_ins.laser_axis[1];
		}
		else if(UartBuff1->Flag==0x81){
			UartBuff1->Flag = 0x0e;
			UartBuff1->Pointer = 0;
			cv_ins.rectangular_det_F = 1;
			sscanf((char*)UartBuff1->Uart_Data,"r%3d%3d%3d%3d%3d%3d%3d%3d",&(cv_ins.rectangular_axis[0]),&cv_ins.rectangular_axis[1],
																				 &cv_ins.rectangular_axis[2],&cv_ins.rectangular_axis[3],
																				 &cv_ins.rectangular_axis[4],&cv_ins.rectangular_axis[5],
																			   &cv_ins.rectangular_axis[6],&cv_ins.rectangular_axis[7]);
//			]=UartBuff1->Uart_Data[1];
//			cv_ins.rectangular_axis[1]=UartBuff1->Uart_Data[2];
//			cv_ins.rectangular_axis[2]=UartBuff1->Uart_Data[3];
//			cv_ins.rectangular_axis[3]=UartBuff1->Uart_Data[4];
//			cv_ins.rectangular_axis[4]=UartBuff1->Uart_Data[5];
//			cv_ins.rectangular_axis[5]=UartBuff1->Uart_Data[6];
//			cv_ins.rectangular_axis[6]=UartBuff1->Uart_Data[7];
//			cv_ins.rectangular_axis[7]=UartBuff1->Uart_Data[8];
			//sscanf((char*)UartBuff1->Uart_Data,"j%3d%3d",&cv_ins.laser_axis[0],&(cv_ins.laser_axis[1]));
		}
		//故障处理
		if(UartBuff1->Flag==0xee){
			UartBuff1->Flag = 0x0e;
			UartBuff1->Pointer = 0;
		}
}




// 串口回调函数模板
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1){
			UartDateHandler(&uart_ins1);
			HAL_UART_Receive_IT(&huart1, &(uart_ins1.Rxbuf), 1);
		}
		else if(huart->Instance == USART2){
			HMIDateHandler(&uart_ins2);
			HAL_UART_Receive_IT(&huart2, &(uart_ins2.Rxbuf), 1);
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

