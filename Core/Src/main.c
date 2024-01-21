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
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// define states
#define Pgo 0
#define Wgo 1
#define Sgo 2
#define Wwait 3
#define Swait 4
#define Warn1 5
#define Off1 6
#define Warn2 7
#define Off2 8
#define Warn3 9
#define AllStop 10

//define colors
#define GREEN 1
#define YELLOW 2
#define WARN 3
#define ALLRED 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const uint32_t greenDelay = 10000;
const uint32_t yellowDelay = 5000;
const uint32_t warnDelay = 1000;
const uint32_t redDelay = 180403;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void TimerDelayMs(uint32_t time);
static void sendRemaningTime(uint8_t color, uint32_t time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Runs on STM32F103C8T6
// Use a table implementation of a Moore finite state machine to operate
// a traffic light.
// Truong Giang Nguyen
// Jan 10th, 2024
//
// walk button sensor connected to PB5 (1=button pressed)
// north facing car detector connected to PB4 (1=car present)
// east facing car detector connected to PB3 (1=car present)
//
// east facing red light connected to PA8
// east facing yellow light connected to PA7
// east facing green light connected to PA6
// north facing red light connected to PA5
// north facing yellow light connected to PA4
// north facing green light connected to PA3
// walk blue light connected to PA2
// walk green light connected to PA1
// walk red light connected to PA0

struct State {
	uint32_t out;
	unsigned long wait;
	uint32_t next[8]; //Index of the state
};
typedef const struct State Stype;

Stype fsm[11] = {
		// Pgo
		{0x127, greenDelay, {Warn1,	Warn1,	Warn1,	Warn1,	Pgo,	Warn1,	Warn1,	Warn1}},
		// Wgo
		{0x61, greenDelay, {Wwait,	Wgo,	Wwait,	Wwait,	Wwait,	Wwait,	Wwait,	Wwait}},
		// Sgo
		{0x109, greenDelay, {Swait,	Swait,	Sgo,	Swait,	Swait,	Swait,	Swait,	Swait}},
		// Wwait
		{0xA1, yellowDelay, {AllStop,	Wgo,	Sgo,	Sgo,	Pgo,	Pgo,	Pgo,	Pgo}},
		// Swait
		{0x111, yellowDelay, {AllStop,	Wgo,	Sgo,	Wgo,	Pgo,	Wgo,	Pgo,	Wgo}},
		// Warn1
		{0x127, warnDelay, {Off1,	Off1,	Off1,	Off1,	Off1,	Off1,	Off1,	Off1}},
		// Off1
		{0x121, warnDelay, {Warn2,	Warn2,	Warn2,	Warn2,	Warn2,	Warn2,	Warn2,	Warn2}},
		// Warn2
		{0x120, warnDelay, {Off2,	Off2,	Off2,	Off2,	Off2,	Off2,	Off2,	Off2}},
		// Off2
		{0x121, warnDelay, {Warn3,	Warn3,	Warn3,	Warn3,	Warn3,	Warn3,	Warn3,	Warn3}},
		// Warn3
		{0x120, warnDelay, {AllStop,	Wgo,	Sgo,	Sgo,	Pgo,	Wgo,	Sgo,	Sgo}},
		// AllStop
		{0x121, redDelay, {AllStop,	Wgo,	Sgo,	Sgo,	Pgo,	Wgo,	Sgo,	Sgo}}
};

uint16_t S;
uint32_t Input = 0xFF;
bool checkWalk = 0;
bool checkSouth = 0;
bool checkWest = 0;
bool checkGPIO = 0;
uint8_t inputValue = 0;
uint32_t greenEnd = 0;
uint32_t yellowEnd = 0;
uint32_t warnEnd = 0;
uint32_t greenEnds = 0;
uint32_t yellowEnds = 0;
uint32_t warnEnds = 0;
uint32_t count1 = 0;
uint32_t count2 = 0;
uint16_t greenCNT = 0;
uint16_t yellowCNT = 0;
uint16_t warnCNT = 0;
char lcdCNT[50];

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
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HD44780_Init(2);
  HD44780_Clear();
  S = AllStop;
  while(1) {
	  // set output
	  GPIOA->ODR = (fsm[S].out)|((fsm[S].out & 0x100)<<1);
	  // delay
	  TimerDelayMs(fsm[S].wait);
	  //read input
	  Input = inputValue;
	  //S = next state
	  S = fsm[S].next[Input];
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void sendRemaningTime(uint8_t color, uint32_t time) {
	time /= 1000;
	time ++;
	uint32_t warnTime = 1;
	switch(color)
	{
	case 1: // green
		HD44780_Clear();
		HD44780_SetCursor(5,0);
		sprintf(lcdCNT,"%d", time);
		HD44780_PrintStr(lcdCNT);
		break;
	case 2: // yellow
		HD44780_Clear();
		HD44780_SetCursor(5,0);
		sprintf(lcdCNT,"%d", time);
		HD44780_PrintStr(lcdCNT);
		break;
	case 3: // warn
		HD44780_Clear();
		HD44780_SetCursor(5,0);
		if(warnTime == 1) {
			sprintf(lcdCNT,"%d", warnTime);
			HD44780_PrintStr(lcdCNT);
			warnTime ++;
			break;
		} else if (warnTime == 2) {
			sprintf(lcdCNT,"%d", warnTime);
			HD44780_PrintStr(lcdCNT);
			warnTime ++;
			break;
		} else if (warnTime == 3) {
			sprintf(lcdCNT,"%d", warnTime);
			HD44780_PrintStr(lcdCNT);
			warnTime ++;
			break;
		} else if (warnTime == 4) {
			sprintf(lcdCNT,"%d", warnTime);
			HD44780_PrintStr(lcdCNT);
			warnTime ++;
			break;
		} else if (warnTime == 5) {
			sprintf(lcdCNT,"%d", warnTime);
			HD44780_PrintStr(lcdCNT);
			warnTime ++;
			break;
		}
		break;
	case 1: // green
		HD44780_Clear();
		HD44780_SetCursor(5,0);
		sprintf(lcdCNT,"%d", time);
		HD44780_PrintStr(lcdCNT);
	}
}

static void TimerDelayMs(uint32_t time) {
	bool jumpToMain = 0;
	switch (time)
	{
	case 10000: // green
		if(greenEnd == 0) {
			HAL_TIM_Base_Start_IT(&htim2);
			if(greenEnds == 0) {
				greenEnds ++;
				greenEnd = 0;
			}
			while(1) {
				greenCNT = TIM2->CNT;
//				greenCNT /= 1000;
//				sprintf(lcdCNT,"%d",greenCNT);
//				HD44780_SetCursor(0,0);
//				HD44780_PrintStr(lcdCNT);
				if(greenEnd == 1) {
					greenCNT = TIM2->CNT;
					count1++;
					jumpToMain = 1;
					HAL_TIM_Base_Stop_IT(&htim2);
					break;
				}
			}
		}
		if((greenEnd >= 1 ) && (jumpToMain == 0)) {
			greenCNT = TIM2->CNT;
//			count2++;
			HAL_TIM_Base_Start_IT(&htim2);
			uint32_t nextGreenEnd = greenEnd + 1;
			uint8_t inputCompare = inputValue;
			while(1) {
				if((greenEnd == nextGreenEnd) || (inputCompare != inputValue)) {
					HAL_TIM_Base_Stop_IT(&htim2);
					break;
				}
			}
		}
		break;
	case 180403: // all red
		greenEnds = 0;
		yellowEnds = 0;
		warnEnds = 0;
		greenEnd = 0;
		yellowEnd = 0;
		warnEnd = 0;
		while(1) {
			if(checkGPIO == 1) {
				break;
			}
		}
	break;
	case 5000: // yellow
		yellowEnd = 0;
		greenEnd = 0;
		greenEnds = 0;
		warnEnd = 0;
		HAL_TIM_Base_Start_IT(&htim3);
		while(1) {
			yellowCNT = TIM3->CNT;
			if(yellowEnd == 1) {
				HAL_TIM_Base_Stop_IT(&htim3);
				break;
			}
		}
	case 1000: // warn
		warnEnd = 0;
		greenEnd = 0;
		greenEnds = 0;
		HAL_TIM_Base_Start_IT(&htim4);
		while(1) {
			warnCNT = TIM4->CNT;
			if(warnEnd == 1) {
				HAL_TIM_Base_Stop_IT(&htim4);
				break;
			}
		}
		break;
	default:
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        greenEnd += 1;
    }
    if (htim->Instance == TIM3) {
    	yellowEnd += 1;
    }
    if (htim->Instance == TIM4) {
    	warnEnd += 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_5) { // Walk button
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) {
            // Rising edge (button released)
            checkWalk = 1;
        } else {
            // Falling edge (button pressed)
            checkWalk = 0;
        }
    }
    if (GPIO_Pin == GPIO_PIN_4) { // Walk button
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) {
            // Rising edge (button released)
            checkSouth = 1;
        } else {
            // Falling edge (button pressed)
            checkSouth = 0;
        }
    }
    if (GPIO_Pin == GPIO_PIN_3) { // Walk button
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) {
            // Rising edge (button released)
            checkWest = 1;
        } else {
            // Falling edge (button pressed)
            checkWest = 0;
        }
    }
    inputValue = (checkWalk << 2) | (checkSouth << 1) | (checkWest << 0);
    checkGPIO = checkWalk | checkSouth |  checkWest;
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
