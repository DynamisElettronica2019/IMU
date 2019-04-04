/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*Quaternion type. Used for conversion*/
typedef struct
{
	float qw;
	float qi;
	float qj;
	float qk;
}quaternionType;

/*Euler angle type. Units: Degrees*/
typedef struct
{
	float roll;
	float pitch;
	float yaw;
}eulerType;

/*Struct used to store raw data from the sensor*/
typedef struct
{
	/*Accelerometer uncompensated data. Units m/s^2. Q point = 8*/
	int16_t accel_X;
	int16_t accel_Y;
	int16_t accel_Z;
	
	/*Accelerometer data compensated against gravity. Units m/s^2. Q point = 8*/
	int16_t linear_X;
	int16_t linear_Y;
	int16_t linear_Z;
	
	/*Gyroscope angular rate data. Units rad/s. Q point = 9*/
	int16_t angular_X;
	int16_t angular_Y;
	int16_t angular_Z;
	
	/*Quaternion rotation data. Order: i - j - k - real. Q point = 12 + accuracy, Q point 12 (units??).*/
	int16_t quat_i;
	int16_t quat_j;
	int16_t quat_k;
	int16_t quat_r;
	int16_t accuracy;
	
}sensorRawReadType;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*THRESOLD for handling singularities in quaternion conversion*/
#define THRESHOLD 0.499
#define M_PI 3.14159265358979323846
#define SOFTWARE_VERSION 1
#define IMU_ADDRESS (0x28 << 1) /*Alternative 0x29 (IMU PIN 17 HIGH), one left bit shift required for hal function*/ 
#define IMU_TIMEOUT_BLK 6
#define IMU_UNKNOWN_BLK 4


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

eulerType quaternionToEuler (quaternionType quaternion);
float radToDeg(float angle);

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
	uint16_t led_cycles;
	HAL_StatusTypeDef IMU_checksum;
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	/*LED_DEBUG_1 turns on: microcontroller powered and initialized successfully.*/
	led_cycles=0;
	while(led_cycles<SOFTWARE_VERSION)
	{
	HAL_GPIO_TogglePin(GPIOC, LED_DEBUG_1_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, LED_DEBUG_1_Pin);
	HAL_Delay(100);
	led_cycles++;
	}
	HAL_GPIO_TogglePin(GPIOC, LED_DEBUG_1_Pin);
	
	/*IMU Initialization.*/
	HAL_GPIO_TogglePin(GPIOC, nBOOT_IMU_Pin); /*Sets nBOOT pin. HIGH -> normal operation / LOW -> Bootloader*/
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, RESET_IMU_Pin); /*Drives HIGH the imu reset pin (ACTIVE LOW). Needs 800 ms delay after reset (BNO055 datasheet)*/
	HAL_Delay(1000);
	
	/*IMU Checksum. LED_DEBUG_2 state: ON -> success / BLINK THREE TIMES -> timeout error / BLINK TWO TIMES -> unknown error*/
	IMU_checksum = HAL_I2C_IsDeviceReady(&hi2c1,IMU_ADDRESS, 2, 10);
	if (IMU_checksum==HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOB, LED_DEBUG_2_Pin);
	}
	else if (IMU_checksum==HAL_TIMEOUT)
	{
		for (led_cycles=0;led_cycles<IMU_TIMEOUT_BLK;led_cycles++)
		{
			HAL_GPIO_TogglePin(GPIOB, LED_DEBUG_2_Pin);
			HAL_Delay(500);
		}
	}
	else
	{
		for (led_cycles=0;led_cycles<IMU_UNKNOWN_BLK;led_cycles++)
		{
			HAL_GPIO_TogglePin(GPIOB, LED_DEBUG_2_Pin);
			HAL_Delay(500);
		}
	}
	
	
			
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_DEBUG_3_Pin|LED_DEBUG_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_DEBUG_1_Pin|nBOOT_IMU_Pin|RESET_IMU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_DEBUG_3_Pin LED_DEBUG_2_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_3_Pin|LED_DEBUG_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DEBUG_1_Pin nBOOT_IMU_Pin RESET_IMU_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_1_Pin|nBOOT_IMU_Pin|RESET_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

eulerType quaternionToEuler (quaternionType quaternion)
{
	eulerType output;
	float sqi,sqj,sqk;
	float test;

	test = quaternion.qi*quaternion.qj + quaternion.qk*quaternion.qw;

	if (test > THRESHOLD) 
	{
		output.yaw = 2 * atan2(quaternion.qi,quaternion.qw);
		output.pitch = M_PI/2;
		output.roll = 0;
		output.yaw = radToDeg(output.yaw);
		output.pitch = radToDeg(output.pitch);
		output.roll = radToDeg(output.roll);
		return output;
	}

	if (test < -THRESHOLD)
	{
		output.yaw = -2 * atan2(quaternion.qi,quaternion.qw);
		output.pitch = - M_PI/2;
		output.roll = 0;
		output.yaw = radToDeg(output.yaw);
		output.pitch = radToDeg(output.pitch);
		output.roll = radToDeg(output.roll);
		return output;
	}

    sqi = quaternion.qi*quaternion.qi;
    sqj = quaternion.qj*quaternion.qj;
    sqk = quaternion.qk*quaternion.qk;

    output.yaw = atan2(2*quaternion.qj*quaternion.qw-2*quaternion.qi*quaternion.qk , 1 - 2*sqj - 2*sqk);
	output.pitch = asin(2*test);
	output.roll = atan2(2*quaternion.qi*quaternion.qw-2*quaternion.qj*quaternion.qk , 1 - 2*sqi - 2*sqk);
	output.yaw = radToDeg(output.yaw);
	output.pitch = radToDeg(output.pitch);
	output.roll = radToDeg(output.roll);
	

	return output;
}

float radToDeg(float angle)
{
	return ((angle/M_PI)*180.00);
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
