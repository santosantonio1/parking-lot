/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRIG_0		(GPIOA->BRR  = 1 << 1)
#define TRIG_1		(GPIOA->BSRR = 1 << 1)

#define LED0_0		(GPIOC->BRR  = 1 << 10)
#define LED0_1		(GPIOC->BSRR = 1 << 10)
#define LED1_0		(GPIOC->BRR  = 1 << 12)
#define LED1_1		(GPIOC->BSRR = 1 << 12)
#define LED2_0		(GPIOD->BRR  = 1 << 0)
#define LED2_1		(GPIOD->BSRR = 1 << 0)
#define LED3_0		(GPIOD->BRR  = 1 << 3)
#define LED3_1		(GPIOD->BSRR = 1 << 3)

#define SLAVE0_0	(GPIOC->BRR  = 1 << 2)
#define SLAVE0_1 	(GPIOC->BSRR = 1 << 2)
#define SLAVE1_0	(GPIOC->BRR  = 1 << 3)
#define SLAVE1_1 	(GPIOC->BSRR = 1 << 3)

#define SLAVE_CTRL	1

#define LOCK_DELAY	5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	if (ch != '\n') {
//		LCD_WRCHAR(ch);
		HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 2);
	}
	return ch;
}

volatile uint32_t echo_start = 0;
volatile uint32_t echo_end0   = 0;
volatile uint32_t echo_end1   = 0;
volatile uint32_t echo_end2   = 0;
volatile uint32_t echo_end3   = 0;
volatile uint8_t  echo_done0  = 0;
volatile uint8_t  echo_done1  = 0;
volatile uint8_t  echo_done2  = 0;
volatile uint8_t  echo_done3  = 0;

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_PIN) {
	if(GPIO_PIN == GPIO_PIN_4) {
		__HAL_TIM_SET_COUNTER(&htim1, 0); // reinicia contagem
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_1) {
		__HAL_TIM_SET_COUNTER(&htim1, 0); // reinicia contagem
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_11) {
		__HAL_TIM_SET_COUNTER(&htim1, 0); // reinicia contagem
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_12) {
		__HAL_TIM_SET_COUNTER(&htim1, 0); // reinicia contagem
		echo_start = 0;
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_PIN) {
	if(GPIO_PIN == GPIO_PIN_4) {
		echo_end0 = __HAL_TIM_GET_COUNTER(&htim1);
		echo_done0 = 1;
	}

	if(GPIO_PIN == GPIO_PIN_1) {
		echo_end1 = __HAL_TIM_GET_COUNTER(&htim1);
		echo_done1 = 1;
	}

	if(GPIO_PIN == GPIO_PIN_11) {
		echo_end2 = __HAL_TIM_GET_COUNTER(&htim1);
		echo_done2 = 1;
	}

	if(GPIO_PIN == GPIO_PIN_12) {
		echo_end3 = __HAL_TIM_GET_COUNTER(&htim1);
		echo_done3 = 1;
	}
}

void udelay(void) {
	int t = 7;
	while(t--);
}

void delayus(int t) {
	while(t--) udelay();
}

void trigger_sensor(){
	TRIG_0;
	delayus(2);
	TRIG_1;
	delayus(10);
	TRIG_0;
}

void transmit_ch(char *ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *) ch, 1, 2);
}

void transmit_str(const char *str) {
	char *ptr = str;
	while (*ptr) transmit_ch(ptr++);
}

void unlock_entrace(int slave_ctrl) {
	while(slave_ctrl--) {
		SLAVE0_1;
		HAL_Delay(1);
		SLAVE0_0;
		HAL_Delay(19);
	}
}

void lock_entrace(int slave_ctrl) {
	while(slave_ctrl--) {
		SLAVE0_1;
		HAL_Delay(2);
		SLAVE0_0;
		HAL_Delay(18);
	}
}

void unlock_exit(int slave_ctrl) {
	while(slave_ctrl--) {
		SLAVE1_1;
		HAL_Delay(1);
		delayus(500);
		SLAVE1_0;
		HAL_Delay(18);
		delayus(500);
	}
}

void lock_exit(int slave_ctrl) {
	while(slave_ctrl--) {
		SLAVE1_1;
		HAL_Delay(2);
		SLAVE1_0;
		HAL_Delay(18);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t pulse0, pulse1, pulse2, pulse3;
	float dist0, dist1, dist2, dist3;

	char nl = '\n';

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  HAL_UART_Init(&huart2);
  printf("\rInicia\n");
  transmit_ch(&nl);

  lock_exit(SLAVE_CTRL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lock_exit(SLAVE_CTRL);
	  HAL_Delay(LOCK_DELAY/2);
	  unlock_exit(SLAVE_CTRL);
	  HAL_Delay(LOCK_DELAY/2);

	  continue;


	  trigger_sensor();
	  if(echo_done0) {
		  pulse0 = echo_end0 - echo_start; // pulse width em us
	  	  dist0 = (pulse0 * 0.0343f)/2;

	  	  if (dist0 < 100.0f) {
	  		  printf("\rDist0 = %05.2f cm\n", dist0);
	  		  transmit_ch(&nl);
	  	  }

	  	  if (dist0 < 6.0f) LED0_0;
	  	  else LED0_1;

	  	  echo_done0 = 0;
	  }

	  if(echo_done1) {
		  pulse1 = echo_end1 - echo_start; // pulse width em us
		  dist1 = (pulse1 * 0.0343f)/2;

		  if (dist1 < 100.0f) {
			  printf("\rDist1 = %05.2f cm\n", dist1);
			  transmit_ch(&nl);
		  }

		  if (dist1 < 6.0f) LED1_0;
		  else LED1_1;

		  echo_done1 = 0;
	  }

	  if(echo_done2) {
		  pulse2 = echo_end2 - echo_start; // pulse width em us
		  dist2 = (pulse2 * 0.0343f)/2;

		  if (dist2 < 100.0f) {
			  printf("\rDist2 = %05.2f cm\n", dist2);
			  transmit_ch(&nl);
		  }

		  if (dist2 < 6.0f) LED2_0;
		  else LED2_1;

		  echo_done2 = 0;
	  }

	  if(echo_done3) {
		  pulse3 = echo_end3 - echo_start; // pulse width em us
		  dist3 = (pulse3 * 0.0343f)/2;

		  if (dist3 < 100.0f) {
			  printf("\rDist3 = %05.2f cm\n", dist3);
			  transmit_ch(&nl);
		  }

		  if (dist3 < 6.0f) LED3_0;
		  else LED3_1;

		  echo_done3 = 0;
	  }

	  HAL_Delay(100);



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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC12 PC2 PC3 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

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
