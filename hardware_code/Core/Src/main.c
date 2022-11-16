/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static uint8_t uart_Rx_buff;
static HAL_StatusTypeDef uart_Rx_busy_flag = HAL_OK;
static HAL_StatusTypeDef uart_Tx_busy_flag = HAL_OK;

// Define calibration (replace with actual calibration data)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//const FusionVector hardIronOffset = {-.2384536685f, .1476907337f, .092882198187f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};


/**
 * Fusion objects used by the Ahrs IMU sensor fusion algorithm
 */
static FusionOffset offset;
static FusionAhrs ahrs;

///**
// * indx indicating the data buffer to read into
// */
//static uint8_t word_indx = 0;

/**
 * variable to get the return flag of i2c calling {HAL_OK or HAL_ERROR}
 */
HAL_StatusTypeDef ret;

/**
 * timer objects for keeping time for calibrating the gyroscope throughout the fusion algorithm
 */
static uint32_t prev_time_stamp;

/**
 * delta time for timing interval between 2 gyro readings
 */
static float delta_time;

/**
 * Fusion algorithm results stored in global static variable, better for monitoring in gdb
 */
static FusionEuler euler;
static FusionVector earth;
/**
 * buffer for printing and UART to high-level computer
 */
char word_buffer[300];

/**
 * structures to maintain global sensor variables
 */
struct SENSOR_Driver slave_xl;
struct SENSOR_Driver slave_gr;
struct SENSOR_Driver slave_mg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  Sensor_Unit_Init(&slave_xl, XL_SENSOR);
  Sensor_Unit_Init(&slave_gr, GR_SENSOR);
  Sensor_Unit_Init(&slave_mg, MG_SENSOR);

//  check the status of sensors first
  ret  = Sensor_Check_Ready(&slave_xl, hi2c1);
  ret  = Sensor_Check_Ready(&slave_gr, hi2c1);
  ret = Sensor_Check_Ready(&slave_mg, hi2c1);


//  set configuration for the accelerometer
  ret = Sensor_Set_Config(&slave_xl, hi2c1, LSM6DSOX_CTRL1_XL_REG, 0b01110010);
//  set configuration for the gyroscope
  ret = Sensor_Set_Config(&slave_gr, hi2c1, LSM6DSOX_CTRL2_GR_REG, 0b01110010);
//  set configuration for the magnetic field sensor
  ret = Sensor_Set_Config(&slave_mg, hi2c1, LIS3MDL_CTRL_REG1, 0b00111110);
  ret = Sensor_Set_Config(&slave_mg, hi2c1, LIS3MDL_CTRL_REG2, 0b00001100);
  ret = Sensor_Set_Config(&slave_mg, hi2c1, LIS3MDL_CTRL_REG3, 0b00000000);
  ret = Sensor_Set_Config(&slave_mg, hi2c1, LIS3MDL_CTRL_REG4, 0b00000100);
//
//  initialize objects for FusionAlgorithm usages
//  FusionOffsetInitialise(&offset, SAMPLE_FREQ);
  FusionAhrsInitialise(&ahrs);

  const FusionAhrsSettings settings = {
	  .gain = 0.5f,
	  .accelerationRejection = 10.0f,
	  .magneticRejection = 20.0f,
	  .rejectionTimeout = 5 * SAMPLE_FREQ, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);


  uart_Rx_busy_flag = HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_Rx_buff, sizeof(uart_Rx_buff));

//  call HAL_GetTick  to get a current time according to system clk
  prev_time_stamp = HAL_GetTick();

//  if using timer fashion callback to prescribe the sampling frequency
//  start the timer 2 in the interrupt mode, counter starting counting from 0 to ARR value and raise IT
//  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		load value from the xl/gr/mg all 3 axes {LOW, HIGH} into data buffers
		ret =   Sensor_Read_Data(&slave_xl, hi2c1);
		ret =   Sensor_Read_Data(&slave_gr, hi2c1);
		ret =   Sensor_Read_Data(&slave_mg, hi2c1);

//		concatenate two 8-bit into one 16-bit 2's complement word
		Sensor_Data_Process(&slave_xl, LSM6DSOX_XL_2G_COEFF);
		Sensor_Data_Process(&slave_gr, LSM6DSOX_GR_125_COEFF);
		Sensor_Data_Process(&slave_mg, LIS3MDL_MG_4_COEFF);

//		advanced calibration methodologies including sensitivities, mis-alignment, and manual offsets
		slave_xl.reading_float =    FusionCalibrationInertial(slave_xl.reading_float, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		slave_gr.reading_float =   FusionCalibrationInertial(slave_gr.reading_float, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		slave_mg.reading_float = FusionCalibrationMagnetic(slave_mg.reading_float, softIronMatrix, hardIronOffset);


//		 Update gyroscope offset correction algorithm
		slave_gr.reading_float = FusionOffsetUpdate(&offset, slave_gr.reading_float);

//		get a timer tag for the current moment {to approx gyro error for next round}
//		return of HAL_GetTick is unsigned integer 32 bit, already in mili-sec
//		0xffff'ffff / (1000 * 60 * 60 * 24) = 49.7 which means sysclk will not overflow until 49 days running
		uint32_t cur_time_stamp = HAL_GetTick();
		delta_time = (float) (cur_time_stamp - prev_time_stamp) * MILISEC_TO_SEC;
		prev_time_stamp = cur_time_stamp;

//		 Update gyroscope AHRS algorithm
		FusionAhrsUpdate(&ahrs, slave_gr.reading_float, slave_xl.reading_float, slave_mg.reading_float, delta_time);

//		retrieve the orientation in Euler Angles and position relative to Earth
		euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		earth = FusionAhrsGetEarthAcceleration(&ahrs);

//		load all info into the word buffer, ready to transmit
//		uart to mac to monitor the output from MCU
	  experiment_log(word_buffer, &euler, &earth, &slave_xl, &slave_gr, &slave_mg);

//	  reset a on-duty interrupt receiving request
	  	if (uart_Rx_busy_flag == HAL_OK)
	  	{
	  		//transmit data
			Sensor_Uart(word_buffer, huart1);
			//restart listening on com port for further data reading commands
	  		uart_Rx_busy_flag = HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_Rx_buff, sizeof(uart_Rx_buff));
	  	}
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.Timing = 0x00100413;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim_int)
//{
//	/*
//	 * TODO each round of data reading in
//	 */
////	  check whether Timer 2 is causing the callback
//	  if (htim_int->Instance != TIM2) return;
//
////	  use UART now, later move onto BLE transmission calls here
//	  Sensor_Uart(word_buffer, huart1);
//}



void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
	/**
	 * TODO former interrupt handle finished, allowed to go to the next one
	 */
	uart_Rx_busy_flag = HAL_OK;
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
	  __NOP();
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
