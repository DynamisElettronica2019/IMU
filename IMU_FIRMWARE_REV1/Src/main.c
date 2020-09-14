/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "id_can.h"
#include "pirelli_sim.h"
#include <stdlib.h> 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MEAS_ID 0x641
#define STATUS_ID 0x640
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

advertiseMeasTypedef AdvertiseMeasFL;
advertiseSensorStatusTypedef AdvertiseSensorStatusFL;

advertiseMeasTypedef AdvertiseMeasFR;
advertiseSensorStatusTypedef AdvertiseSensorStatusFR;

advertiseMeasTypedef AdvertiseMeasRL;
advertiseSensorStatusTypedef AdvertiseSensorStatusRL;

advertiseMeasTypedef AdvertiseMeasRR;
advertiseSensorStatusTypedef AdvertiseSensorStatusRR;

uint8_t sendBuffer [8];
uint8_t status=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_blink(void);
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
	
	//Note: inserire il valore che si intende inviare, gli offset vengono applicati dalla funzione di conversione da struct ad array
	
	/*advertiseMeasTypedef:							//Range accettabili:

	uint32_t UID;												//Da 0 a 0xFFFFFFFF
	uint16_t Pressure;									//Da 0 a 2047
	statusTypedef Measurement_Status;		//Vedi typedef status
	int16_t Temperature;								//Da -78 a 177
	uint8_t Service_Counter;						//Da 0 a 255 ciclico

	advertiseSensorStatusTypedef:				//Range accettabili:

	uint32_t UID;												//Da 0 a 0xFFFFFFFF
	locationTypedef Location;						//Vedi typedef Location
	axleTypedef Axle;										//Vedi typedef axle
	typeTypedef Type;										//Vedi typedef type
	int8_t RSSI;												//Da -127 a 0
	muteTypedef Mute;										//Vedi typedef mute
	uint16_t Battery_Voltage;						//Da 100 a 355		
	uint8_t Service_Counter;						//Da 0 a 255 ciclico*/
	
	//Inizializzazione front left
	
	AdvertiseMeasFL.UID = 0xCACCA000;
	AdvertiseMeasFL.Pressure = 100;
	AdvertiseMeasFL.Measurement_Status = IN_RANGE;
	AdvertiseMeasFL.Temperature = 85;
	AdvertiseMeasFL.Service_Counter = 0;
	
	AdvertiseSensorStatusFL.UID = 0xCACCA000;
	AdvertiseSensorStatusFL.Location = LEFT;
	AdvertiseSensorStatusFL.Axle = FRONT;
	AdvertiseSensorStatusFL.Type = TYPE_UNDEFINED;
	AdvertiseSensorStatusFL.RSSI = -15;
	AdvertiseSensorStatusFL.Mute = PSN_ADVERTISING;
	AdvertiseSensorStatusFL.Battery_Voltage = 255;
	AdvertiseSensorStatusFL.Service_Counter = 0;
	
	//Inizializzazione front right
	AdvertiseMeasFR.UID = 0xCACCA001;
	AdvertiseMeasFR.Pressure = 120;
	AdvertiseMeasFR.Measurement_Status = IN_RANGE;
	AdvertiseMeasFR.Temperature = 70;
	AdvertiseMeasFR.Service_Counter = 0;
	
	AdvertiseSensorStatusFR.UID = 0xCACCA001;
	AdvertiseSensorStatusFR.Location = RIGHT;
	AdvertiseSensorStatusFR.Axle = FRONT;
	AdvertiseSensorStatusFR.Type = TYPE_UNDEFINED;
	AdvertiseSensorStatusFR.RSSI = -16;
	AdvertiseSensorStatusFR.Mute = PSN_ADVERTISING;
	AdvertiseSensorStatusFR.Battery_Voltage = 250;
	AdvertiseSensorStatusFR.Service_Counter = 0;
	
	//Inizializzazione rear left
	
	AdvertiseMeasRL.UID = 0xCACCA002;
	AdvertiseMeasRL.Pressure = 90;
	AdvertiseMeasRL.Measurement_Status = IN_RANGE;
	AdvertiseMeasRL.Temperature = 120;
	AdvertiseMeasRL.Service_Counter = 0;
	
	AdvertiseSensorStatusRL.UID = 0xCACCA002;
	AdvertiseSensorStatusRL.Location = LEFT;
	AdvertiseSensorStatusRL.Axle = REAR;
	AdvertiseSensorStatusRL.Type = TYPE_UNDEFINED;
	AdvertiseSensorStatusRL.RSSI = -17;
	AdvertiseSensorStatusRL.Mute = PSN_ADVERTISING;
	AdvertiseSensorStatusRL.Battery_Voltage = 200;
	AdvertiseSensorStatusRL.Service_Counter = 0;
	
	//Inizializzazione rear right
	
	AdvertiseMeasRR.UID = 0xCACCA003;
	AdvertiseMeasRR.Pressure = 90;
	AdvertiseMeasRR.Measurement_Status = IN_RANGE;
	AdvertiseMeasRR.Temperature = 120;
	AdvertiseMeasRR.Service_Counter = 0;
	
	AdvertiseSensorStatusRR.UID = 0xCACCA003;
	AdvertiseSensorStatusRR.Location = RIGHT;
	AdvertiseSensorStatusRR.Axle = REAR;
	AdvertiseSensorStatusRR.Type = TYPE_UNDEFINED;
	AdvertiseSensorStatusRR.RSSI = -18;
	AdvertiseSensorStatusRR.Mute = PSN_ADVERTISING;
	AdvertiseSensorStatusRR.Battery_Voltage = 200;
	AdvertiseSensorStatusRR.Service_Counter = 0;
	
		
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
  MX_I2C1_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	LED_blink();
	canStart();
	HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM6)
	{
		switch (status)
		{
			case 0:
				AdvertiseMeasFL.Service_Counter++;
				AdvertiseMeasToArray(AdvertiseMeasFL,sendBuffer);
				CAN_send_intel(sendBuffer, MEAS_ID, 8);
				AdvertiseMeasFL.Pressure++;
				AdvertiseMeasFL.Temperature++;
				status = 1;
			break;
			case 1:
				AdvertiseSensorStatusFL.Service_Counter++;
				AdvertiseSensorStatusToArray(AdvertiseSensorStatusFL,sendBuffer);
				CAN_send_intel(sendBuffer, STATUS_ID, 8);
				status = 2;
			break;
			case 2:
				AdvertiseMeasFR.Service_Counter++;
				AdvertiseMeasToArray(AdvertiseMeasFR,sendBuffer);
				CAN_send_intel(sendBuffer, MEAS_ID, 8);
				AdvertiseMeasFR.Pressure++;
				AdvertiseMeasFR.Temperature++;
				status = 3;
			break;
			case 3:
				AdvertiseSensorStatusFR.Service_Counter++;
				AdvertiseSensorStatusToArray(AdvertiseSensorStatusFR,sendBuffer);
				CAN_send_intel(sendBuffer, STATUS_ID, 8);
				status = 4;
			break;
			case 4:
				AdvertiseMeasRL.Service_Counter++;
				AdvertiseMeasToArray(AdvertiseMeasRL,sendBuffer);
				CAN_send_intel(sendBuffer, MEAS_ID, 8);
				AdvertiseMeasRL.Pressure++;
				AdvertiseMeasRL.Temperature++;
				status = 5;
			break;
			case 5:
				AdvertiseSensorStatusRL.Service_Counter++;
				AdvertiseSensorStatusToArray(AdvertiseSensorStatusRL,sendBuffer);
				CAN_send_intel(sendBuffer, STATUS_ID, 8);
				status = 6;
			break;
			case 6:
				AdvertiseMeasRR.Service_Counter++;
				AdvertiseMeasToArray(AdvertiseMeasRR,sendBuffer);
				CAN_send_intel(sendBuffer, MEAS_ID, 8);
				AdvertiseMeasRR.Pressure++;
				AdvertiseMeasRR.Temperature++;
				status = 7;
			break;
			case 7:
				AdvertiseSensorStatusRR.Service_Counter++;
				AdvertiseSensorStatusToArray(AdvertiseSensorStatusRR,sendBuffer);
				CAN_send_intel(sendBuffer, STATUS_ID, 8);
				status = 0;
			break;
			default:
				
			break;
		}
		HAL_GPIO_TogglePin(GPIOC, LED_DEBUG_1_Pin);
	}
}

void LED_blink(void)
{
	uint8_t count=0;
	HAL_GPIO_WritePin(GPIOC, LED_DEBUG_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_DEBUG_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_DEBUG_3_Pin, GPIO_PIN_RESET);
	
	while (count<5)
	{
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOC, LED_DEBUG_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LED_DEBUG_2_Pin, GPIO_PIN_SET);	
			HAL_GPIO_WritePin(GPIOB, LED_DEBUG_3_Pin, GPIO_PIN_SET);	
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOC, LED_DEBUG_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_DEBUG_2_Pin, GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOB, LED_DEBUG_3_Pin, GPIO_PIN_RESET);	
			count++;	
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
