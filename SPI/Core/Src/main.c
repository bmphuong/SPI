/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define TURN 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t PSX_RX[8];
uint8_t PSX_TX[2] = {
		0x01, 0x42
};
enum state
{
	FORWARD,
	FOR_LEFT,
	FOR_RIGHT,
	BACKWARD,
	BAC_LEFT,
	BAC_RIGHT,
	LEFT_ROUND,
	RIGHT_ROUND,
	ZERO
};
enum state state_check(uint8_t arr[8]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void forward();
void left();
void right();
void backward();
void forward_left();
void forward_right();
void backward_left();
void backward_right();
void stop_case();
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 HAL_Delay(5);
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 HAL_SPI_TransmitReceive(&hspi1, PSX_TX, PSX_RX, 8, 10);
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 switch (state_check(PSX_RX))
	 {
		 case FORWARD :
			 forward();
			 break;

		 case FOR_LEFT :
			forward_left();
			break;

		 case FOR_RIGHT :
			forward_right();
			break;

		 case BACKWARD :
			backward();
			break;

		 case BAC_LEFT :
			backward_left();
			break;

		 case BAC_RIGHT :
			backward_right();
			break;

		 case RIGHT_ROUND :
		 			right();
		 			break;

		 case LEFT_ROUND :
		 			left();
		 			break;

		 case ZERO:
			stop_case();
			break;

		 default:
			 break;
	 }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
enum state state_check(uint8_t arr[8])
{
	uint8_t forward_flag = 0;
	uint8_t backward_flag = 0;
	uint8_t left_flag = 0;
	uint8_t right_flag = 0;
	if (arr[3] == 127 || arr[3] == 111 || arr[3] == 63) left_flag = 1;
	if (arr[3] == 223 || arr[3] == 207 || arr[3] == 159) right_flag = 1;
	if (arr[4] == 239 || arr[4] == 111 || arr[4] == 207) forward_flag = 1;
	if (arr[4] == 159 || arr[4] == 191 || arr[4] == 63) backward_flag = 1;
	if (forward_flag && left_flag) return FOR_LEFT;
	else
		if (forward_flag && right_flag) return FOR_RIGHT;
		else if (forward_flag) return FORWARD;
		else if (backward_flag && left_flag) return BAC_LEFT;
		else if (backward_flag && right_flag) return BAC_RIGHT;
		else if (backward_flag) return BACKWARD;
		else if (left_flag) return LEFT_ROUND;
		else if (right_flag) return RIGHT_ROUND;
		else return ZERO;
}
void forward()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 100);
}
void left()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 100);
}
void right()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
}
void backward()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
}
void forward_left()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100 - TURN);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 100);
}
void forward_right()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, TURN);
}
void backward_left()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, TURN);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
}
void backward_right()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 100 - TURN);
}
void stop_case()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
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
