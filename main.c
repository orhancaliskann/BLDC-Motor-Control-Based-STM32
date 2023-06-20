/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Q1H TIM_CHANNEL_1
#define Q2H TIM_CHANNEL_2
#define Q3H TIM_CHANNEL_3

#define PHASE_I     hall_sensor1 == 1 && hall_sensor2 == 0 && hall_sensor3 == 1
#define PHASE_II    hall_sensor1 == 1 && hall_sensor2 == 0 && hall_sensor3 == 0
#define PHASE_III   hall_sensor1 == 1 && hall_sensor2 == 1 && hall_sensor3 == 0
#define PHASE_IV    hall_sensor1 == 0 && hall_sensor2 == 1 && hall_sensor3 == 0
#define PHASE_V     hall_sensor1 == 0 && hall_sensor2 == 1 && hall_sensor3 == 1
#define PHASE_VI    hall_sensor1 == 0 && hall_sensor2 == 0 && hall_sensor3 == 1

#define RIGHT   1
#define LEFT    0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//float adc = 0, current_duty, current, motor_voltage, motor_voltage_duty;
//float raw_voltage;

uint8_t v_dat;
uint16_t v_adc;
uint16_t current_dat;
uint16_t current_adc;

uint32_t adc_read[2];

uint8_t hall_sensor1, hall_sensor2, hall_sensor3;

uint8_t right_push = 1;
uint8_t left_push = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void motor_movement_fw(void);
void motor_movement_bw(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    if (GPIO_Pin == HALL1_Pin) {
        if (HAL_GPIO_ReadPin(HALL1_GPIO_Port, HALL1_Pin)) {
            hall_sensor1 = 1;
        }

        if (!HAL_GPIO_ReadPin(HALL1_GPIO_Port, HALL1_Pin)) {
            hall_sensor1 = 0;
        }
    }

    if (GPIO_Pin == HALL2_Pin) {
        if (HAL_GPIO_ReadPin(HALL2_GPIO_Port, HALL2_Pin)) {
            hall_sensor2 = 1;
        }

        if (!HAL_GPIO_ReadPin(HALL2_GPIO_Port, HALL2_Pin)) {
            hall_sensor2 = 0;
        }
    }

    if (GPIO_Pin == HALL3_Pin) {
        if (HAL_GPIO_ReadPin(HALL3_GPIO_Port, HALL3_Pin)) {
            hall_sensor3 = 1;
        }

        if (!HAL_GPIO_ReadPin(HALL3_GPIO_Port, HALL3_Pin)) {
            hall_sensor3 = 0;
        }
    }
    if (GPIO_Pin == RIGHT_Pin)
    {
    	right_push=1;
    	left_push=0;
    }
    if (GPIO_Pin == LEFT_Pin)
    {
    	right_push=0;
    	left_push=1;
    }
}

void motor_movement_fw(void) {
    if (PHASE_I) {
        HAL_GPIO_WritePin(Q1L_GPIO_Port, Q1L_Pin, 0);
        HAL_TIM_PWM_Stop(&htim1, Q1H);
        HAL_GPIO_WritePin(Q2L_GPIO_Port, Q2L_Pin, 1);
        HAL_TIM_PWM_Stop(&htim2, Q2H);
        HAL_GPIO_WritePin(Q3L_GPIO_Port, Q3L_Pin, 0);
        HAL_TIM_PWM_Start(&htim1, Q3H);
    } else if (PHASE_II) {
        HAL_GPIO_WritePin(Q1L_GPIO_Port, Q1L_Pin, 1);
        HAL_TIM_PWM_Stop(&htim1, Q1H);
        HAL_GPIO_WritePin(Q2L_GPIO_Port, Q2L_Pin, 0);
        HAL_TIM_PWM_Stop(&htim2, Q2H);
        HAL_GPIO_WritePin(Q3L_GPIO_Port, Q3L_Pin, 0);
        HAL_TIM_PWM_Start(&htim1, Q3H);
    } else if (PHASE_III) {
        HAL_GPIO_WritePin(Q1L_GPIO_Port, Q1L_Pin, 1);
        HAL_TIM_PWM_Stop(&htim1, Q1H);
        HAL_GPIO_WritePin(Q2L_GPIO_Port, Q2L_Pin, 0);
        HAL_TIM_PWM_Start(&htim2, Q2H);
        HAL_GPIO_WritePin(Q3L_GPIO_Port, Q3L_Pin, 0);
        HAL_TIM_PWM_Stop(&htim1, Q3H);
    } else if (PHASE_IV) {
        HAL_GPIO_WritePin(Q1L_GPIO_Port, Q1L_Pin, 0);
        HAL_TIM_PWM_Stop(&htim1, Q1H);
        HAL_GPIO_WritePin(Q2L_GPIO_Port, Q2L_Pin, 0);
        HAL_TIM_PWM_Start(&htim2, Q2H);
        HAL_GPIO_WritePin(Q3L_GPIO_Port, Q3L_Pin, 1);
        HAL_TIM_PWM_Stop(&htim1, Q3H);
    } else if (PHASE_V) {
        HAL_GPIO_WritePin(Q1L_GPIO_Port, Q1L_Pin, 0);
        HAL_TIM_PWM_Start(&htim1, Q1H);
        HAL_GPIO_WritePin(Q2L_GPIO_Port, Q2L_Pin, 0);
        HAL_TIM_PWM_Stop(&htim2, Q2H);
        HAL_GPIO_WritePin(Q3L_GPIO_Port, Q3L_Pin, 1);
        HAL_TIM_PWM_Stop(&htim1, Q3H);
    } else if (PHASE_VI) {
        HAL_GPIO_WritePin(Q1L_GPIO_Port, Q1L_Pin, 0);
        HAL_TIM_PWM_Start(&htim1, Q1H);
        HAL_GPIO_WritePin(Q2L_GPIO_Port, Q2L_Pin, 1);
        HAL_TIM_PWM_Stop(&htim2, Q2H);
        HAL_GPIO_WritePin(Q3L_GPIO_Port, Q3L_Pin, 0);
        HAL_TIM_PWM_Stop(&htim1, Q3H);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, adc_read, 2);

  hall_sensor1 = HAL_GPIO_ReadPin(HALL1_GPIO_Port, HALL1_Pin);
  hall_sensor2 = HAL_GPIO_ReadPin(HALL2_GPIO_Port, HALL2_Pin);
  hall_sensor3 = HAL_GPIO_ReadPin(HALL3_GPIO_Port, HALL3_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      v_adc = adc_read[0];
      v_dat = v_adc/41;

      current_adc = adc_read[1];
      current_dat =current_adc * 3.3 * 2 / 4095 * 1.035;
     // current = (raw_voltage - 2.5) / 0.1;

      TIM1->CCR1 = v_dat;
      TIM2->CCR2 = v_dat;
      TIM1->CCR3 = v_dat;

      motor_movement_fw();
//      if (right_push == 1)
//      {
//          motor_movement_bw();
//      }
//      else if (left_push == 1)
//      {
//          motor_movement_fw();
//      }

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
