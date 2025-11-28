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

#define scl_0		(GPIOC->BRR  = 1 << 6)
#define scl_1		(GPIOC->BSRR = 1 << 6)
#define sda_0		(GPIOC->BRR  = 1 << 8)
#define sda_1 		(GPIOC->BSRR = 1 << 8)
#define sda_in 		(GPIOC->IDR & (1 << 8))
#define i2c_ack 0
#define i2c_nak 255

#define sck_0 		(GPIOA->BRR  = 1 << 11)
#define sck_1 		(GPIOA->BSRR = 1 << 11)
#define data_0 		(GPIOA->BRR  = 1 << 12)
#define data_1 		(GPIOA->BSRR = 1 << 12)
#define data_in 	(GPIOA->IDR & (1 << 12))

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

#define READ_BTN	(GPIOC->IDR & (1 << 3))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void udelay(void);
void delayus(int t);

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t dado);
uint8_t i2c_read(uint8_t ack_state);
void i2c_wrmem(uint8_t slave_addr, uint8_t addr, uint8_t addr_size, uint8_t data);
uint8_t i2c_rdmem(uint8_t slave_addr, uint8_t addr, uint8_t addr_size);

void sht_init(void);
void sht_pulse(void);
void sht_start(void);
void sht_stop(void);
void sht_wrCmd(uint8_t data);
uint8_t sht_rdData(uint8_t confirm);
float sht_rdTemp(void);
uint16_t sht_rdTempRAW(void);
float sht_rdHumid(void);
uint16_t sht_rdHumidRAW(void);
void sht_reSynch(void);

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

volatile uint8_t ctrl_entrace = 0;
volatile uint8_t ctrl_exit = 0;

void trigger_sensor(){
	TRIG_0;
	delayus(2);
	TRIG_1;
	delayus(10);
	TRIG_0;
}

void transmit_int(uint32_t n) {
	char str[12];
	sprintf(str, "%d", n);
	transmit_str(&str);
}

void transmit_ch(char *ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *) ch, 1, 2);
}

void transmit_str(const char *str) {
	char *ptr = str;
	while (*ptr) transmit_ch(ptr++);
}

TIM_OC_InitTypeDef sConfig_entrace = {0}, sConfig_exit = {0};

void unlock_entrace() {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	sConfig_entrace.Pulse = 20 - 1; //TH = 8ms, TL = 2ms
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig_entrace, TIM_CHANNEL_1);
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void lock_entrace() {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	sConfig_entrace.Pulse = 12 - 1; //TH = 8ms, TL = 2ms
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig_entrace, TIM_CHANNEL_1);
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void lock_exit() {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	sConfig_exit.Pulse = 14-1; //TH = 8ms, TL = 2ms
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig_exit, TIM_CHANNEL_2);
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void unlock_exit() {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	sConfig_exit.Pulse = 24-1; //TH = 8ms, TL = 2ms
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig_exit, TIM_CHANNEL_2);
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
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

	uint8_t entrace_cnt, exit_cnt;

	float temp;

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_PWM_Init(&htim3);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  HAL_UART_Init(&huart2);

  i2c_init();
  sht_init();

  sConfig_entrace.OCMode = TIM_OCMODE_PWM1;
  sConfig_entrace.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig_entrace.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig_entrace.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig_entrace.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig_entrace.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig_exit.OCMode = TIM_OCMODE_PWM1;
  sConfig_exit.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig_exit.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig_exit.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig_exit.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig_exit.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig_exit.Pulse = 15 - 1;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig_exit, TIM_CHANNEL_2);
  HAL_TIM_PWM_Init(&htim3);



  lock_entrace();
  lock_exit();

  entrace_cnt = 50;
  exit_cnt = 50;

  printf("\rInicia\n");
  transmit_ch(&nl);

  temp = 3.14f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  temp = sht_rdTemp();
	  sht_reSynch();
	  printf("\r%3.2f\n", temp);
	  nl = '\n';
	  transmit_ch(&nl);

	  HAL_Delay(200);

	  continue;

	  if (ctrl_entrace) {
	      if (entrace_cnt == 50) {
	          unlock_entrace();
	      }

	      if (entrace_cnt > 0) {
	          entrace_cnt--;
	      }

	      if (entrace_cnt == 0) {
	          lock_entrace();
	          ctrl_entrace = 0;
	      }

	  } else {
	      entrace_cnt = 50;
	  }

	  if (ctrl_exit) {
		  if (exit_cnt == 50) {
			  unlock_exit();
		  }

		  if (exit_cnt > 0) {
			  exit_cnt--;
		  }

		  if (exit_cnt == 0) {
			  lock_exit();
			  ctrl_exit = 0;
		  }

	  } else {
		  exit_cnt = 50;
	  }

	  transmit_int(entrace_cnt);
	  nl = '\r';
	  transmit_ch(&nl);
	  nl = '\n';
	  transmit_ch(&nl);

	  transmit_int(ctrl_entrace);
	  nl = '\r';
	  transmit_ch(&nl);
	  nl = '\n';
	  transmit_ch(&nl);

	  trigger_sensor();
	  if(echo_done0) {
		  pulse0 = echo_end0 - echo_start; // pulse width em us
	  	  dist0 = (pulse0 * 0.0343f)/2;

//	  	  if (dist0 < 100.0f) {
//	  		  printf("\rDist0 = %05.2f cm\n", dist0);
//	  		  transmit_ch(&nl);
//	  	  }

	  	  if (dist0 < 6.0f) LED0_0;
	  	  else LED0_1;

	  	  echo_done0 = 0;
	  }

	  if(echo_done1) {
		  pulse1 = echo_end1 - echo_start; // pulse width em us
		  dist1 = (pulse1 * 0.0343f)/2;

//		  if (dist1 < 100.0f) {
//			  printf("\rDist1 = %05.2f cm\n", dist1);
//			  transmit_ch(&nl);
//		  }

		  if (dist1 < 6.0f) LED1_0;
		  else LED1_1;

		  echo_done1 = 0;
	  }

	  if(echo_done2) {
		  pulse2 = echo_end2 - echo_start; // pulse width em us
		  dist2 = (pulse2 * 0.0343f)/2;

//		  if (dist2 < 100.0f) {
//			  printf("\rDist2 = %05.2f cm\n", dist2);
//			  transmit_ch(&nl);
//		  }

		  if (dist2 < 6.0f) LED2_0;
		  else LED2_1;

		  echo_done2 = 0;
	  }

	  if(echo_done3) {
		  pulse3 = echo_end3 - echo_start; // pulse width em us
		  dist3 = (pulse3 * 0.0343f)/2;

//		  if (dist3 < 100.0f) {
//			  printf("\rDist3 = %05.2f cm\n", dist3);
//			  transmit_ch(&nl);
//		  }

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 200-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC12 PC6 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11;
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

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void udelay(void) {
	int t = 7;
	while(t--);
}

void delayus(int t) {
	while(t--) udelay();
}

//--------------------------
//			I2C
//--------------------------
void i2c_init(void)
{
	sda_1;
	scl_1;
	delayus(50);
}

void i2c_start(void)
{
	sda_0;
	delayus(5);
	scl_0;
	delayus(1);
}

void i2c_stop(void)
{
	sda_0;
	delayus(1);
	scl_1;
	delayus(5);
	sda_1;
	delayus(5);
}

uint8_t i2c_write(uint8_t dado)
{
	int8_t i;
	uint8_t resp;
	for (i = 7; i >= 0; i--)
	{
		if ((dado & (1 << i)) != 0) sda_1; else sda_0;
		delayus(1);
		scl_1;
		delayus(5);
		scl_0;
		delayus(5);
	}
	sda_1;
	delayus(1);
	scl_1;
	if (sda_in != 0) resp = i2c_nak; else resp = i2c_ack;
	delayus(5);
	scl_0;
	delayus(5);
	sda_0;
	delayus(1);
	return resp;
}

uint8_t i2c_read(uint8_t ack_state)
{
	int8_t i;
	uint8_t v = 0;
	sda_1;
	delayus(1);
	for (i = 7; i >= 0; i--)
	{
		v <<= 1;
		scl_1;
		if (sda_in != 0) v++;
		delayus(5);
		scl_0;
		delayus(5);
	}
	if (ack_state) sda_1; else sda_0;
	scl_1;
	delayus(5);
	scl_0;
	delayus(5);
	return v;
}

void i2c_wrmem(uint8_t slave_addr, uint8_t addr, uint8_t addr_size, uint8_t data)
{
	uint8_t hi_addr, lo_addr;

	hi_addr = addr/256;
	lo_addr = addr%256;

	i2c_start();

	i2c_write((slave_addr & 0xfe)); // Garante que op seja 0 (wr)

	if(addr_size == 2) i2c_write(hi_addr);
	i2c_write(lo_addr);

	i2c_write(data);

	i2c_stop();
}

uint8_t i2c_rdmem(uint8_t slave_addr, uint8_t addr, uint8_t addr_size)
{
	uint8_t resp;
	uint8_t hi_addr, lo_addr;

	hi_addr = addr/256;
	lo_addr = addr%256;

	i2c_start();

	i2c_write((slave_addr & 0xfe)); // Garante que op seja 0 (wr)

	if(addr_size == 2) i2c_write(hi_addr);
	i2c_write(lo_addr);

	i2c_stop();
	i2c_start();

	i2c_write((slave_addr & 0xfe) + 1); // Garante que op seja 1 (rd)
	resp = i2c_read(i2c_nak);

	i2c_stop();

	return resp;
}

//--------------------------
//			SHT15
//--------------------------

void sht_init(void)
{
	data_0;
	sck_0;
	data_1;
	delayus(10);
}

void sht_pulse(void)
{
	delayus(1);
	sck_1;
	delayus(2);
	sck_0;
	delayus(1);
}

void sht_start(void)
{
	sck_1;
	delayus(1);
	data_0;
	delayus(1);
	sck_0;
	delayus(1);
	sck_1;
	delayus(1);
	data_1;
	delayus(1);
	sck_0;
	delayus(1);
}

void sht_stop(void)
{
	data_1;
	sht_pulse();
}

void sht_wrCmd(uint8_t data)
{
	uint8_t n;
	for(n=0; n<8; n++)
	{
		if((data & (1<<(7-n))) == 0) data_0; else data_1;
		sht_pulse();
	}
	data_1; // Libera a porta para leitura...
	sht_pulse(); // Ack do device...
}

uint8_t sht_rdData(uint8_t confirm)
{
	uint8_t n, val=0;
	data_1;
	for(n=0; n<8; n++)
	{
		val <<= 1;
		delayus(1);
		sck_1;
		delayus(1);

		if(data_in != 0) val++;
		delayus(1);
		sck_0;
		delayus(1);
	}
	if(confirm == 0) data_0; else data_1;
	sht_pulse();
	data_1;
	return val;
}

float sht_rdTemp(void)
{
	uint16_t temp = 0;
	uint8_t Hi_data, Lo_data;
	float linear_temp;
	sht_start();
	sht_wrCmd(0x03);
	while(data_in != 0) HAL_Delay(1); // Aguarda data_in = 0
	Hi_data = sht_rdData(0); // ACK
	Lo_data = sht_rdData(0xff); // NAK e Stop...
	temp = (Hi_data<<8)+Lo_data;
	linear_temp = 0.0102*temp-40.988;
	return linear_temp;
}

uint16_t sht_rdTempRAW(void)
{
	uint16_t temp = 0;
	uint8_t Hi_data, Lo_data;
	sht_start();
	sht_wrCmd(0x03);
	while(data_in != 0) HAL_Delay(1); // Aguarda data_in = 0...
	Hi_data = sht_rdData(0); // ACK
	Lo_data = sht_rdData(0xff); // NAK e Stop...
	temp = (Hi_data<<8)+Lo_data;
	return temp;
}

float sht_rdHumid(void)
{
	uint16_t humid = 0;
	uint8_t Hi_data, Lo_data;
	float linear_humid;
	sht_start();
	sht_wrCmd(0x05);
	while(data_in != 0) HAL_Delay(1); // Aguarda data_in = 0...
	Hi_data = sht_rdData(0); // ACK
	Lo_data = sht_rdData(0xff); // NAK e Stop...
	humid = (Hi_data<<8)+Lo_data;
	linear_humid = (4e-10)*humid*humid*humid - 0.000005*humid*humid + 0.0434*humid -7.3845;
	return linear_humid;
}

uint16_t sht_rdHumidRAW(void)
{
	uint16_t humid = 0;
	uint8_t Hi_data, Lo_data;
	sht_start();
	sht_wrCmd(0x05);
	while(data_in != 0) HAL_Delay(1); // Aguarda data_in = 0...
	Hi_data = sht_rdData(0); // ACK
	Lo_data = sht_rdData(0xff); // NAK e Stop...
	humid = (Hi_data<<8)+Lo_data;
	return humid;
}

void sht_reSynch(void)
{
	uint8_t n;
	data_1;
	for(n=0; n<10; n++)
	{
		sht_pulse();
	}
}
//--------------------------

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_PIN) {
	if(GPIO_PIN == GPIO_PIN_4) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_1) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_11) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_12) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		echo_start = 0;
	}

	if(GPIO_PIN == GPIO_PIN_2) {
		ctrl_exit = 1;
	}

	if(GPIO_PIN == GPIO_PIN_3) {
		ctrl_entrace = 1;
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
