/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "servo.h"
#include "crsf.h"
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_RING_SIZE 256
#define ADC_DMA_BUFFER_SIZE 3

#define VREF_HISTORY_LEN 20
#define CH_HISTORY_LEN 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t rxByte;    //
static volatile uint8_t rxRing[RX_RING_SIZE]; //
static volatile uint16_t rxHead; //Pointer
static volatile uint16_t rxTail;   //Pointer

// Voltage monitoring variables

static volatile uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE] = {0};

static uint16_t vref_history[VREF_HISTORY_LEN];
static uint16_t ch1_history[CH_HISTORY_LEN];
static uint16_t ch5_history[CH_HISTORY_LEN];
static uint8_t vref_idx = 0, ch1_idx = 0, ch5_idx = 0;
static bool vref_filled = false, ch1_filled = false, ch5_filled = false;

static uint32_t last_voltage_check_ms = 0;
static uint16_t ch1_warn_count = 0;
static uint16_t ch5_warn_count = 0;
static uint16_t ch1_recover_count = 0, ch5_recover_count = 0;
static uint16_t vdda_fail_count = 0, vdda_recover_count = 0;

typedef enum {
    VOLT_STATE_NORMAL = 0,
    VOLT_STATE_WARN,
    VOLT_STATE_CRITICAL,
    VOLT_STATE_VDDA_FAIL
} VoltState;

static VoltState state_6V = VOLT_STATE_NORMAL;
static VoltState state_BEC = VOLT_STATE_NORMAL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void BLUE_LED_ON();
void BLUE_LED_OFF();
void GREEN_LED_ON();
void GREEN_LED_OFF();
void RED_LED_ON();
void RED_LED_OFF();
void BUZZER_ON();
void BUZZER_OFF();
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
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  led_set(false);
  servo_init();
  crsf_init();
  HAL_UART_Receive_IT(&huart2,&rxByte,1);
  HAL_ADCEx_Calibration_Start(&hadc1);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }


  //RED_LED_ON();
  //GREEN_LED_ON();
  //BLUE_LED_ON();
  //BUZZER_ON();
  //HAL_Delay(10);
  //BUZZER_OFF();
  //RED_LED_OFF();
  //GREEN_LED_OFF();
  //BLUE_LED_OFF();

  //float Motor_Voltage = 0;
  //float Cell_Box_Voltage = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_ADC_Start(&hadc2);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUFFER_SIZE)
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_DMA_Buffer, ADC_DMA_BUFFER_SIZE);
  while (1)
  {
	  app_status_tick();// Must run every loop to handle blinking, handles all LEDs and Buzzer automatically

      if (HAL_GetTick() - last_voltage_check_ms >= 100) {
          last_voltage_check_ms = HAL_GetTick();

          vref_history[vref_idx++] = adc_dma_buffer[2];
          if (vref_idx >= VREF_HISTORY_LEN) { vref_idx = 0; vref_filled = true; }
          ch1_history[ch1_idx++] = adc_dma_buffer[1];
          if (ch1_idx >= CH_HISTORY_LEN) { ch1_idx = 0; ch1_filled = true; }
          ch5_history[ch5_idx++] = adc_dma_buffer[0];
          if (ch5_idx >= CH_HISTORY_LEN) { ch5_idx = 0; ch5_filled = true; }

          uint32_t vref_sum = 0; int vref_n = vref_filled ? VREF_HISTORY_LEN : vref_idx;
          for (int i = 0; i < vref_n; i++) vref_sum += vref_history[i];
          uint32_t raw_vref_avg = vref_n ? vref_sum / vref_n : adc_dma_buffer[2];

          uint32_t ch1_sum = 0; int ch1_n = ch1_filled ? CH_HISTORY_LEN : ch1_idx;
          for (int i = 0; i < ch1_n; i++) ch1_sum += ch1_history[i];
          uint32_t raw_ch1_avg = ch1_n ? ch1_sum / ch1_n : adc_dma_buffer[1];

          uint32_t ch5_sum = 0; int ch5_n = ch5_filled ? CH_HISTORY_LEN : ch5_idx;
          for (int i = 0; i < ch5_n; i++) ch5_sum += ch5_history[i];
          uint32_t raw_ch5_avg = ch5_n ? ch5_sum / ch5_n : adc_dma_buffer[0];

          float Vdda = 3.3f;
          if (raw_vref_avg > 0) Vdda = (1.20f * 4095.0f) / (float)raw_vref_avg;

          if (Vdda < 3.0f) {
              vdda_fail_count++; vdda_recover_count = 0;
              if (vdda_fail_count >= 5) {
                  state_6V = VOLT_STATE_VDDA_FAIL;
                  state_BEC = VOLT_STATE_VDDA_FAIL;
              }
          } else {
              vdda_fail_count = 0;
              if (state_6V == VOLT_STATE_VDDA_FAIL) {
                  if (++vdda_recover_count >= 10) {
                      state_6V = VOLT_STATE_NORMAL; state_BEC = VOLT_STATE_NORMAL;
                      ch1_warn_count = ch5_warn_count = 0;
                      ch1_recover_count = ch5_recover_count = vdda_recover_count = 0;
                  }
              } else {
                  float V_6V  = (float)raw_ch1_avg * Vdda / 4095.0f * 2.0f;
                  float V_BEC = (float)raw_ch5_avg * Vdda / 4095.0f * 1.833f;

                  /* CH1 (6V) */
                  if (state_6V == VOLT_STATE_CRITICAL) {
                      if (V_6V >= 5.0f) {
                          if (++ch1_recover_count >= 10) { state_6V = VOLT_STATE_NORMAL; ch1_warn_count = 0; ch1_recover_count = 0; }
                      } else ch1_recover_count = 0;
                  } else if (V_6V < 4.8f) {
                      if (++ch1_warn_count >= 5) { state_6V = VOLT_STATE_CRITICAL; ch1_recover_count = 0; }
                  } else if (V_6V < 5.0f) {
                      if (++ch1_warn_count >= 10) state_6V = VOLT_STATE_WARN;
                  } else { ch1_warn_count = 0; state_6V = VOLT_STATE_NORMAL; }

                  /* CH5 (BEC) */
                  if (state_BEC == VOLT_STATE_CRITICAL) {
                      if (V_BEC >= 4.8f) {
                          if (++ch5_recover_count >= 5) { state_BEC = VOLT_STATE_NORMAL; ch5_warn_count = 0; ch5_recover_count = 0; }
                      } else ch5_recover_count = 0;
                  } else if (V_BEC < 4.6f) {
                      if (++ch5_warn_count >= 5) { state_BEC = VOLT_STATE_CRITICAL; ch5_recover_count = 0; }
                  } else if (V_BEC < 4.8f) {
                      if (++ch5_warn_count >= 5) state_BEC = VOLT_STATE_WARN;
                  } else { ch5_warn_count = 0; state_BEC = VOLT_STATE_NORMAL; }
              }
          }

          /* Hand over the states to the app manager for LED/Buzzer updates */
          app_set_voltage_state(state_6V, state_BEC);

          /* Failsafe Servo Actions */
          if (state_6V == VOLT_STATE_CRITICAL || state_BEC == VOLT_STATE_CRITICAL || state_6V == VOLT_STATE_VDDA_FAIL) {
              app_set_volt_failsafe(true);
              failsafe_apply(); 
          } else {
              app_set_volt_failsafe(false);
          }
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while(rxTail != rxHead)//If not full
	  {
		  uint8_t b = rxRing[rxTail];
		  rxTail = (uint16_t)((rxTail + 1) % RX_RING_SIZE);
		  crsf_rx_byte(b); //call crsf
	  }
	  crsf_check_link(); //失控檢查
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  huart2.Init.BaudRate = 420000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint16_t next = (uint16_t)((rxHead + 1) % RX_RING_SIZE);
    if (next != rxTail)
    {
      rxRing[rxHead] = rxByte;
      rxHead = next;
    }
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);   /* overflow */
    __HAL_UART_CLEAR_NEFLAG(huart);    /* noise */
    __HAL_UART_CLEAR_FEFLAG(huart);    /* data wrong */
    __HAL_UART_CLEAR_PEFLAG(huart);    /* check again*/
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);   /* wrong from receive */
  }
}

void BLUE_LED_ON()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,GPIO_PIN_SET);
}

void BLUE_LED_OFF()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,GPIO_PIN_RESET);
}

void GREEN_LED_ON()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);
}

void GREEN_LED_OFF()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_RESET);
}

void RED_LED_ON()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
}

void RED_LED_OFF()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
}

void BUZZER_ON()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);//Buzzer low voltage ON
}

void BUZZER_OFF()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);//Buzzer high voltage OFF
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
