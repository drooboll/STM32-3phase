/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /*PA7   ------> TIM1_CH1N
    PB0   ------> TIM1_CH2N
    PB1   ------> TIM1_CH3N
    PA8   ------> TIM1_CH1
    PA9   ------> TIM1_CH2
    PA10   ------> TIM1_CH3*/

  GPIOA->CRL |= 0b10 << GPIO_CRL_CNF7_Pos | 0b11 << GPIO_CRL_MODE7_Pos;
  GPIOA->CRH |= 0b10 << GPIO_CRH_CNF8_Pos | 0b11 << GPIO_CRH_MODE8_Pos | 0b10 << GPIO_CRH_CNF9_Pos | 0b11 << GPIO_CRH_MODE9_Pos | 0b10 << GPIO_CRH_CNF9_Pos | 0b11 << GPIO_CRH_MODE10_Pos;
  GPIOB->CRL |= 0b10 << GPIO_CRL_CNF0_Pos | 0b11 << GPIO_CRL_MODE0_Pos | 0b10 << GPIO_CRL_CNF1_Pos | 0b11 << GPIO_CRL_MODE1_Pos;
  GPIOA->CRL |= 0b1 << GPIO_CRL_CNF0_Pos | 0b11 << GPIO_CRL_MODE0_Pos;
  //TIM1->CCR1 = pwmHigh;
  TIM1->CR1 |= TIM_CR1_CEN;
  //TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->EGR = TIM_EGR_UG;
  //LL_mDelay(5000);
  uint32_t state = 0;

  TIM1->CCR1 = 0;
  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);

  TIM1->CCR2 = 0;
  TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);

  TIM1->CCR3 = pwmHigh;
  TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

  TIM1->EGR = TIM_EGR_UG;

  LL_mDelay(300);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t rot = 0;
  const uint32_t topValue = 900;
  const uint32_t startValue = 10;
  const uint32_t delayDefault = 8;
  const uint32_t targetValue = 1;

  while (1)
  {
	  //continue;
	if (rot < topValue)
		rot++;

	if (rot > startValue){
		LL_mDelay(delayDefault - (rot - startValue) * (delayDefault - targetValue) / (topValue - startValue));
	} else {
		LL_mDelay(delayDefault);
	}
	GPIOA->ODR ^= GPIO_ODR_ODR0;
	TIM1->CNT = 0;

	switch(state){
	case 0:
		TIM1->CCR1 = pwmHigh;
		TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

		TIM1->CCR2 = 0; // ??
		TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);

		TIM1->CCR3 = 0;
		TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);

		TIM1->EGR = TIM_EGR_UG;

		break;
	case 1:
		TIM1->CCR1 = pwmHigh;
		TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

		TIM1->CCR2 = 0; // ??
		TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);

		TIM1->CCR3 = 0;
		TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

		TIM1->EGR = TIM_EGR_UG;

		break;
	case 2:
		TIM1->CCR1 = 0;
		TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);

		TIM1->CCR2 = pwmHigh;
		TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);

		TIM1->CCR3 = 0;
		TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

		TIM1->EGR = TIM_EGR_UG;

		break;
	case 3:
		TIM1->CCR1 = 0;
		TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

		TIM1->CCR2 = pwmHigh;
		TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);

		TIM1->CCR3 = 0;
		TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);

		TIM1->EGR = TIM_EGR_UG;

		break;
	case 4:
		TIM1->CCR1 = 0;
		TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

		TIM1->CCR2 = 0;
		TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);

		TIM1->CCR3 = pwmHigh;
		TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

		TIM1->EGR = TIM_EGR_UG;

		break;
	case 5:
		TIM1->CCR1 = 0;
		TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);

		TIM1->CCR2 = 0;
		TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);

		TIM1->CCR3 = pwmHigh;
		TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

		TIM1->EGR = TIM_EGR_UG;

		break;
	}

	state++;
	state = state % 6;

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSE_EnableCSS();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = timerHigh;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 20;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM1 GPIO Configuration
  PA7   ------> TIM1_CH1N
  PB0   ------> TIM1_CH2N
  PB1   ------> TIM1_CH3N
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  PA10   ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_RemapPartial_TIM1();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

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
