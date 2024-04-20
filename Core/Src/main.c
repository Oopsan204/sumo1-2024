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

#include "stm32f1xx.h"
#include "tim_pwm.h"
#include "AML_LaserSensor.h"
#include "AML_DebugDevice.h"
#include "parameter.h"
#include <stdbool.h>

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t u8_fr = 1; // gpio_ext3
uint8_t u8_fl = 1; // gpio_ext0
uint8_t u8_br = 1; // gpio_ext4
uint8_t u8_bl = 1; // gpio_ext1
uint16_t button[Array_Size_Button];
uint16_t currentIndex = 1;
// co interrupt
bool flagInterrupt_fl = false;
bool flagInterrupt_bl = false;
bool flagInterrupt_br = false;
bool flagInterrupt_fr = false;
int32_t timer1 = 0;
uint8_t i = 0;

uint16_t target = 0;
void (*func)(uint16_t);
void (*plan)(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AML_motor_stop()
{
  PWM_Start(&htim2, LPWM1);
  PWM_Start(&htim2, LPWM2);
  PWM_Start(&htim2, RPWM1);
  PWM_Start(&htim2, RPWM2);
  PWM_Write(&htim2, LPWM1, 0);
  PWM_Write(&htim2, LPWM2, 0);
  PWM_Write(&htim2, RPWM1, 0);
  PWM_Write(&htim2, RPWM2, 0);
}
void AML_motor_forward(uint8_t speed)
{
  PWM_Start(&htim2, RPWM1);
  PWM_Start(&htim2, RPWM2);
  PWM_Write(&htim2, RPWM1, speed);
  PWM_Write(&htim2, RPWM2, speed);
}
void AML_motor_lef(uint8_t speed_L, uint8_t speed_R)
{
  PWM_Start(&htim2, RPWM2);
  PWM_Start(&htim2, LPWM1);
  PWM_Write(&htim2, RPWM2, speed_L - 50);
  PWM_Write(&htim2, LPWM1, speed_R);
}
void AML_motor_right(uint8_t speed_L, uint8_t speed_R)
{
  PWM_Start(&htim2, LPWM2);
  PWM_Start(&htim2, RPWM1);
  PWM_Write(&htim2, LPWM2, speed_L);
  PWM_Write(&htim2, RPWM1, speed_R - 50);
}
void AML_motor_back(uint8_t speed)
{
  PWM_Start(&htim2, LPWM1);
  PWM_Start(&htim2, LPWM2);
  PWM_Write(&htim2, LPWM1, speed);
  PWM_Write(&htim2, LPWM2, speed);
}
void AML_motor_rote(uint8_t speed_L, uint8_t speed_R)
{
  PWM_Start(&htim2, LPWM1);
  PWM_Start(&htim2, RPWM2);
  PWM_Write(&htim2, LPWM1, speed_L);
  PWM_Write(&htim2, RPWM2, speed_R);
}
void delay_timer_3(uint16_t ms)
{
  for (uint16_t i = 0; i < ms; i++)
  {
    TIM3->CNT = 0;
    while (TIM3->CNT < 1000)
      ;
  }
}
void AML_IRSensor_standby()
{
  const uint32_t time1 = 1000;

  while (flagInterrupt_fl && HAL_GetTick() - timer1 < time1)
    ;
  while (flagInterrupt_bl && HAL_GetTick() - timer1 < time1)
    ;
  while (flagInterrupt_br && HAL_GetTick() - timer1 < time1)
    ;
  while (flagInterrupt_fr && HAL_GetTick() - timer1 < time1)
    ;
  // dat lai co
  flagInterrupt_bl = false;
  flagInterrupt_br = false;
  flagInterrupt_fl = false;
  flagInterrupt_fr = false;
  AML_motor_stop();
}
void search()
{
  target = searchNearest();
  if (target == 0)
  {
    /*
    khong tim thay doi thu
    xoay tron tim doi thu
    */
    // AML_motor_rote(PWM_speed_L, PWM_speed_R);
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(delay1);
  }
  if (target == 2)
  {
    // FF dam dau
    if (AML_LaserSensor_ReadSingleWithFillter(FF) < 100)
    {
      AML_motor_forward(PWM_speed);
    }
    else
    {
      AML_motor_forward(PWM_speed - 40);
    }
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(delay1);
  }
  if (target == 1)
  {
    // FL re trai
    if (AML_LaserSensor_ReadSingleWithFillter(FL) < 100)
    {
      AML_motor_lef(PWM_speed_L - 50, PWM_speed_R);
      
    }

    else
    {
      AML_motor_lef(PWM_speed_L - 50, PWM_speed_R);
    }
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(delay1);
  }
  if (target == 3)
  {
    if (AML_LaserSensor_ReadSingleWithFillter(FR) < 100)
    {
      AML_motor_right(PWM_speed_L, PWM_speed_R - 50);
    }
    else
    {
      AML_motor_right(PWM_speed_L, PWM_speed_R - 50);
    }
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(delay1);
  }
  if (target == 6)
  {
    // R quay sang phai
    if (AML_LaserSensor_ReadSingleWithFillter(R) < 100)
    {
      AML_motor_right(PWM_speed_L, PWM_speed_R - 50);
    }
    else
    {
      AML_motor_right(PWM_speed_L - 50, PWM_speed_R - 50);
    }
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(100);
  }
  if (target == 7)
  {
    // L quay sang trai
    if (AML_LaserSensor_ReadSingleWithFillter(L) < 100)
    {
      AML_motor_lef(PWM_speed_L - 50, PWM_speed_R);
    }
    else
    {
      AML_motor_lef(PWM_speed_L - 50, PWM_speed_R - 50);
    }
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(100);
  }
  if (target == 4 || target == 5)
  {
    if (AML_LaserSensor_ReadSingleWithFillter(BL) < 40 || AML_LaserSensor_ReadSingleWithFillter(BR) < 40)
    {
      AML_motor_back(150);
    }
    else
    {
      AML_motor_back(100);
    }
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    HAL_Delay(delay1);
  }
}
void plan_begin()
{
  AML_motor_forward(150);
  // HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
  // HAL_Delay(900);
  // AML_motor_right(150, 100);
  // HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
  // HAL_Delay(500);
}
void plan_stop()
{
  AML_motor_stop();
}

// void readADCStore()
// {
//   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)button, 1);
//   uint16_t adcvalue = button[1];
//   if (currentIndex < Array_Size_Button)
//   {
//     button[currentIndex] = adcvalue;
//     currentIndex++;
//   }
//   else
//   {
//     currentIndex = 2;
//   }
// }

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // HAL_Delay(3000); // delay 3s
  AML_LaserSensor_Setup();
  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
  plan_begin();
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)button, 1);
  // switch (button[0])   
  // {
  // case 0:
  //   plan = &plan_begin;
  //   break;
  // case 127:
  //   plan = &plan_stop;
  // default:
  //   break;
  // }
  // plan();
  // button tatic

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    AML_LaserSensor_ReadAll();
    print_sensorvalue();
    search();
    AML_IRSensor_standby();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XSHUT_FL_Pin|XSHUT_FF_Pin|XSHUT_FR_Pin|XSHUT_BR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, XSHUT_BL_Pin|XSHUT_R_Pin|XSHUT_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT_FL_Pin XSHUT_FF_Pin XSHUT_FR_Pin XSHUT_BR_Pin */
  GPIO_InitStruct.Pin = XSHUT_FL_Pin|XSHUT_FF_Pin|XSHUT_FR_Pin|XSHUT_BR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzz_Pin */
  GPIO_InitStruct.Pin = Buzz_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(Buzz_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT_BL_Pin XSHUT_R_Pin XSHUT_L_Pin */
  GPIO_InitStruct.Pin = XSHUT_BL_Pin|XSHUT_R_Pin|XSHUT_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void interrupt_bl()
{
  PWM_Start(&htim2, RPWM1);
  PWM_Start(&htim2, RPWM2);
  PWM_Write(&htim2, RPWM1, PWM_Driver_Left_B);
  PWM_Write(&htim2, RPWM2, PWM_Driver_Right_B);

  flagInterrupt_bl = true; // bat co
  timer1 = HAL_GetTick();  // moc thoi gian bat dau thoat hiem
  flagInterrupt_br = false;
  flagInterrupt_fl = false;
  flagInterrupt_fr = false;
}

void interrupt_br()
{
  PWM_Start(&htim2, RPWM1);
  PWM_Start(&htim2, RPWM2);
  PWM_Write(&htim2, RPWM1, PWM_Driver_Left_F);
  PWM_Write(&htim2, RPWM2, PWM_Driver_Right_F);

  flagInterrupt_br = true; // bat co
  timer1 = HAL_GetTick();  // moc thoi gian bat dau thoat hiem
  flagInterrupt_bl = false;
  flagInterrupt_fl = false;
  flagInterrupt_fr = false;
}

void interrupt_fl()
{
  PWM_Start(&htim2, LPWM1);
  PWM_Start(&htim2, LPWM2);
  PWM_Write(&htim2, LPWM1, PWM_Driver_Left_B);
  PWM_Write(&htim2, LPWM2, PWM_Driver_Right_B);

  flagInterrupt_fl = true; // bat co
  timer1 = HAL_GetTick();  // moc thoi gian bat dau thoat hiem
  flagInterrupt_br = false;
  flagInterrupt_bl = false;
  flagInterrupt_fr = false;
}
void interrupt_fr()
{
  PWM_Start(&htim2, LPWM1);
  PWM_Start(&htim2, LPWM2);
  PWM_Write(&htim2, LPWM1, PWM_Driver_Left_F);
  PWM_Write(&htim2, LPWM2, PWM_Driver_Right_F);

  flagInterrupt_fr = true; // bat co
  timer1 = HAL_GetTick();  // moc thoi gian bat dau thoat hiem
  flagInterrupt_br = false;
  flagInterrupt_fl = false;
  flagInterrupt_bl = false;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  if (GPIO_Pin == GPIO_PIN_0)
  {
    u8_fl = 0; // ir4
    interrupt_fl();
  }
  if (GPIO_Pin == GPIO_PIN_1)
  {
    u8_bl = 0; // ir2
    interrupt_bl();
  }
  if (GPIO_Pin == GPIO_PIN_4)
  {
    u8_br = 0; // ir1
    interrupt_br();
  }
  if (GPIO_Pin == GPIO_PIN_3)
  {
    u8_fr = 0; // ir3
    interrupt_fr();
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
