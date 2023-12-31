/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stepper.h"
#include "macro.h"
#include "control.h"
#include "connect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_2);
	HAL_UART_Receive_IT(&huart1,&(uart_ins1.Rxbuf),1);
	HAL_UART_Receive_IT(&huart2,&(uart_ins2.Rxbuf),1);
	
	Init_Stepper(&stepper1,dir1_GPIO_Port,dir1_Pin,&htim2,TIM_CHANNEL_1,0.05625f);
	Init_Stepper(&stepper2,dir2_GPIO_Port,dir2_Pin,&htim2,TIM_CHANNEL_2,0.05625f/2);
	
	laser_init(&laser_ins,0,0);
	
	float kp_stp1=0.1,ki_stp1=0.00,kd_stp1=0;
	float kp_stp2=0.1,ki_stp2=0.00,kd_stp2=0;
	Laser_On;
	pid_init(&pid_stp1,kp_stp1,ki_stp1,kd_stp1);
	pid_init(&pid_stp2,kp_stp2,ki_stp2,kd_stp2);
	pid_stop();
	
	HAL_Delay(3000);
	
//	move_derectly(mission2_point_list[0],mission2_point_list[1],500);
//	drawline(0,0,500,500,30);
//	StpDistanceSetBlocking(&stepper2,3,1000,100);
//	HAL_Delay(3000);
//	StpDistanceSetBlocking(&stepper2,-10,1000,500);
//	drawline(0,0,0,200,500);
//	HAL_Delay(3000);
//	drawline(200,0,200,200);
//	HAL_Delay(1000);
//	drawline(-200,200,-200,0);
//	HAL_Delay(1000);
//	drawline(-200,0,0,0);
  /* USER CODE END 2 */
//	laser_set_target(&laser_ins,cv_ins.rectangular_axis[0],cv_ins.rectangular_axis[1]);
//	laser_set_target(&laser_ins,320,240);
	//pid_start(1);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	mission3();
  while (1)
  {
    /* USER CODE END WHILE */
//		HAL_UART_Transmit(&huart1,(uint8_t*)"b",1,10);
//		mission2();
			
//		HAL_Delay(1000);
//		motor_reset2origin();
//		HAL_Delay(1000);
//		break;
	//	cal_axis();
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM4){
		if(PID_F==PID_START){
//			pid_incremental(&pid_stp2,laser_ins.x_target-laser_ins.x_cur,3,-3);
//			pid_incremental(&pid_stp1,laser_ins.y_target-laser_ins.y_cur,3,-3);
//			if(!IFMOVING(stepper1.motor_state))
//				StpDistanceSetBlocking(&stepper1,-pid_stp1.output,200,500);
//			if(!IFMOVING(stepper2.motor_state))
//				StpDistanceSetBlocking(&stepper2,-pid_stp2.output,200,500);
			pid_position(&pid_stp2,laser_ins.x_target-laser_ins.x_cur,10,-10,3000);
			pid_position(&pid_stp1,laser_ins.y_target-laser_ins.y_cur,15,-15,3000);
		//	if(!IFMOVING(stepper1.motor_state))
		//		StpDistanceSetBlocking(&stepper1,stepper1.position_ctnow*stepper1.stepangle-pid_stp1.output,200,500);
			if(!IFMOVING(stepper2.motor_state))
				StpDistanceSetBlocking(&stepper2,-stepper2.position_ctnow*stepper2.stepangle+pid_stp2.output,200,500);
		}
	}
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
