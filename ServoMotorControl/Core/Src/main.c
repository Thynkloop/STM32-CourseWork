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
/* USER CODE BEGIN PFP */
void GPIO_PB6_Init(void);
void TIM4_Init(void);
void Set_Servo_Angle(uint16_t angle);
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
  /* USER CODE BEGIN 2 */
  GPIO_PB6_Init();

  // Initialize TIM3 for PWM generation
  TIM4_Init();
  /* USER CODE END 2 */
  TIM4->CCR1 = 1000; //1 ms
//  TIM4->CCR1 = 210; //2 ms
  HAL_Delay(1000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Set servo angle to 0 degrees (1 ms pulse width)
//	Set_Servo_Angle(0);
//	HAL_Delay(1000);
//
//	// Set servo angle to 90 degrees (1.5 ms pulse width)
//	Set_Servo_Angle(90);
//	HAL_Delay(1000);
//
//	// Set servo angle to 180 degrees (2 ms pulse width)
//	Set_Servo_Angle(180);
//	HAL_Delay(1000);
//	  for(int i = 1000; i <= 2000; i++)
//	  {
//		  TIM4->CCR1 = i;
//		  HAL_Delay(50);
//	  }
//	  for(int i = 2000; i >= 1000; i--)
//	  {
//		  TIM4->CCR1 = i;
//		  HAL_Delay(50);
//	  }

	  TIM4->CCR1 = 1000;
	  HAL_Delay(1000);

	  TIM4->CCR1 = 1500;
	  HAL_Delay(1000);

	  TIM4->CCR1 = 2500;
	  HAL_Delay(1000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */
void GPIO_PB6_Init(void) {
    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Set PB6 as alternate function (PWM output)
    GPIOB->MODER &= ~GPIO_MODER_MODE6;
    GPIOB->MODER |= GPIO_MODER_MODE6_1;  // Alternate function mode
    GPIOB->AFR[0] |= (0x2 << (6 * 4));   // AF2 (TIM4_CH1)
}
void TIM4_Init(void) {
    // Enable TIM4 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Set prescaler and auto-reload for 50 Hz frequency (20 ms period)
    TIM4->PSC = 84;  // Prescaler to divide 84 MHz by 840 (100 kHz timer clock)
    TIM4->ARR = 20000-1; // Auto-reload value for 50 Hz (100 kHz / 50 Hz - 1)

    // Set pulse width for 0 degrees (1 ms)
    TIM4->CCR1 = 99;  // 1 ms pulse width at 50 Hz

    // Configure PWM mode for OC1 (PWM Mode 1)
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M; // Clear previous settings
    TIM4->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // Set PWM Mode 1 for OC1

    // Enable PWM output on TIM4_CH1
    TIM4->CCER |= TIM_CCER_CC1E;          // Enable output

    // Start the timer
    TIM4->CR1 |= TIM_CR1_CEN;
}

void Set_Servo_Angle(uint16_t angle) {
    // Calculate pulse width based on angle (1 ms for 0°, 1.5 ms for 90°, 2 ms for 180°)
    uint16_t pulseWidth = (angle * 1000 / 180) + 100; // Pulse width = (angle / 180) * 1 ms + 1 ms

    // Set the compare register for TIM4
    TIM4->CCR1 = pulseWidth;
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
