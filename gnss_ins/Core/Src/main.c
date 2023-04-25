/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_defines.h"
#include "imu.h"
#include "gnss.h"
#include "ins.h"
#include "moving_average_filter.h"
#include "logging.h"
#include "unit.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void acc_init();
void acc_handle();

void gnss_init();
void gnss_handle();

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
  MX_I2C3_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_UART7_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

#ifdef UNIT
  unit_test();
#endif

  HAL_TIM_Base_Start_IT(&htim3);//period 10ms
  HAL_TIM_Base_Start_IT(&htim4);//period 1s

  acc_init();
  gnss_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  gnss_handle();
	  acc_handle();

    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//  if (huart->Instance == UART4)
//  {
//	  HAL_UART_AbortReceive(&huart4);
//	  g_flag.uart4_rx = 1;
//  }
//}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		g.counter.uart4_error++;
		HAL_UART_AbortReceive(&huart4);
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)g.buffer.uart4_rx, UART4_RX_SIZE);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_UART_AbortReceive(&huart4);
  g.counter.pps++;
  g.flag.uart4_rx_interrupt = TRUE;
//  if(0x0400 != GPIO_Pin)
//  {
//	  uint8_t breakpoint = 0;
//  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
	  g.flag.tim3_interrupt = TRUE;
	  g.counter.tim3++;
  }
  else if (htim->Instance == TIM4)
  {
	  g.flag.tim4_interrupt = TRUE;
	  g.counter.tim4++;
  }
}

void acc_init()
{
	//wait for 5s before calibration
	while(g.counter.tim4 < 4)
	{
		uint8_t i = 0;
	}

	imu_init_acc();

	moving_average_filter_reset(&g.filter_acc_x, FILTER_SIZE_CALIBRATION);
	moving_average_filter_reset(&g.filter_acc_y, FILTER_SIZE_CALIBRATION);
	moving_average_filter_reset(&g.filter_acc_z, FILTER_SIZE_CALIBRATION);

	moving_average_filter_reset(&g.filter_gyro_x, FILTER_SIZE_CALIBRATION);
	moving_average_filter_reset(&g.filter_gyro_y, FILTER_SIZE_CALIBRATION);
	moving_average_filter_reset(&g.filter_gyro_z, FILTER_SIZE_CALIBRATION);
}

void acc_handle()
{

  if(TRUE == g.flag.tim3_interrupt)
  {
	  imu_read_acc(&g.acc_x, &g.acc_y, &g.acc_z);
	  imu_read_gyro(&g.gyro_x, &g.gyro_y, &g.gyro_z);

	  moving_average_filter_update_float(&g.filter_acc_x, g.acc_x.value);
	  moving_average_filter_update_float(&g.filter_acc_y, g.acc_y.value);
	  moving_average_filter_update_float(&g.filter_acc_z, g.acc_z.value);

	  moving_average_filter_update_float(&g.filter_gyro_x, g.gyro_x.value);
	  moving_average_filter_update_float(&g.filter_gyro_y, g.gyro_y.value);
	  moving_average_filter_update_float(&g.filter_gyro_z, g.gyro_z.value);

	  g.acc_x.value_filtered = g.filter_acc_x.value;
	  g.acc_y.value_filtered = g.filter_acc_y.value;
	  g.acc_z.value_filtered = g.filter_acc_z.value;

	  g.gyro_x.value_filtered = g.filter_gyro_x.value;
	  g.gyro_y.value_filtered = g.filter_gyro_y.value;
	  g.gyro_z.value_filtered = g.filter_gyro_z.value;

	  if(g.filter_acc_x.buf_size >= FILTER_SIZE_CALIBRATION || g.flag.acc_calibrated == 1)
	  {
		  if(g.flag.acc_calibrated == 0)
		  {
			  imu_calibrate_acc(&g.acc_x, &g.acc_y, &g.acc_z);
			  imu_calibrate_gyro(&g.gyro_x, &g.gyro_y, &g.gyro_z);

			  moving_average_filter_reset(&g.filter_acc_x, FILTER_SIZE);
			  moving_average_filter_reset(&g.filter_acc_y, FILTER_SIZE);
			  moving_average_filter_reset(&g.filter_acc_z, FILTER_SIZE);

			  imu_read_acc(&g.acc_x, &g.acc_y, &g.acc_z);
			  imu_read_gyro(&g.gyro_x, &g.gyro_y, &g.gyro_z);

			  g.flag.acc_calibrated = 1;

			  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
			  HAL_Delay(250);
			  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
			  HAL_Delay(250);
			  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
			  HAL_Delay(250);
			  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
		  }

		  g.ins.acceleration_2d = ins_calculate_2d_acceleration(g.acc_x.value_filtered, g.acc_y.value_filtered);
		  g.ins.acceleration_3d = ins_calculate_3d_acceleration(g.acc_x.value_filtered, g.acc_y.value_filtered, g.acc_z.value_filtered);

		  g.ins.velocity_m_s = ins_calculate_velocity(&g.ins.velocity_initial, g.ins.acceleration_3d, ACC_SAMPLE_PERIOD);
		  g.ins.velocity_km_h = g.ins.velocity_m_s * 3600 / 1000;

		  g.counter.acc_read++;
	  }

	  g.flag.tim3_interrupt = FALSE;
  }
}

void gnss_init()
{
		//Pradetas daryti baudrate pakeitimas i 115200
//	strncpy(uart4_tx_buffer, SET_NMEA_BAUDRATE_115200, 15);
//	strcpy(uart4_tx_buffer[15], "*");
//
//	L76X_Send_Command((uint8_t *)uart4_tx_buffer);
//
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)g.buffer.uart4_rx, UART4_RX_SIZE);

	#ifdef GNSS_TEST
	  L76X_parse_GNRMC(buff_t, BUFFSIZE);
	#endif
}

void gnss_handle()
{
  if(TRUE == g.flag.uart4_rx_interrupt)
  {
//		  GNRMC gnss_result;
//		  gnss_result = L76X_Gat_GNRMC();

	#ifndef GNSS_TEST
	  gnss_parse_GNRMC(g.buffer.uart4_rx, UART4_RX_SIZE);
	#endif

//	  g.flag.gnss_initiated = TRUE;
	  g.counter.gnss_parse++;

	  if(g.counter.gnss_parse % 10 == 0 && g.counter.test_ins == 0)
	  {
		  g.counter.test_ins = 5;
	  }

	  if(g.counter.test_ins > 0)
	  {
		  g.flag.gnss_active = FALSE;
		  g.counter.test_ins--;
	  }
	  else
	  {
		  g.flag.gnss_active = TRUE;
	  }

	  joint_result_calculate_bearing(&g.joint_result, &g.gnss, &g.acc_x, &g.gyro_z);

	  g.gnss.no_packet_counter = 0;

	  if(TRUE == g.flag.acc_calibrated && TRUE == g.flag.gnss_initiated && g.gnss.lat < 90)
	  {
		  logging();
	  }

	  g.gnss.last_timestamp = 0;

	  HAL_GPIO_TogglePin(GPIOA, LD3_Pin);

	  g.flag.uart4_rx_interrupt = FALSE;
	  HAL_UART_Receive_DMA(&huart4, (uint8_t *)g.buffer.uart4_rx, UART4_RX_SIZE);
  }
  //Detection if there was lost gnss packets
//  else if(g.flag.tim4_interrupt > 1)
//  {
//	  if(g.gnss.last_timestamp > 0)
//	  {
//		  g.gnss.no_packet_counter++;
//		  g.flag.gnss_active = FALSE;
//	  }
//	  else
//	  {
//		  g.gnss.last_timestamp++;
//	  }
//
//	  g.flag.tim4_interrupt = 0;
//  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
