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
uint16_t adc_value1, adc_value2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_Init(void) {
// 1. Enable clock for GPIOA
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

// 2. Configure PA0 and PA1 in analog mode
GPIOA->MODER |= GPIO_MODER_MODER0; // Set PA0 to analog mode (11)
GPIOA->MODER |= GPIO_MODER_MODER1; // Set PA1 to analog mode (11)

// 3. Enable ADC1 clock
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

// 4. Configure ADC1
// Disable ADC first
ADC1->CR2 &= ~ADC_CR2_ADON;

// Reset registers
ADC1->CR1 = 0x00000000;
ADC1->CR2 = 0x00000000;

// Configure CR1
// 12-bit resolution (default)
// Scan mode enabled for multiple channels
ADC1->CR1 |= ADC_CR1_SCAN;

// Configure CR2
// Right alignment (default)
// Enable continuous conversion
ADC1->CR2 |= ADC_CR2_CONT;
// External trigger disabled (software trigger)

// Configure number of conversions in regular sequence
ADC1->SQR1 &= ~ADC_SQR1_L; // Clear L[3:0] bits
ADC1->SQR1 |= ((2-1) << 20); // Set L[3:0] to 1 (2 conversions)

// Configure channel sequence
ADC1->SQR3 &= ~(ADC_SQR3_SQ1 | ADC_SQR3_SQ2); // Clear SQ1 and SQ2 bits
ADC1->SQR3 |= (0 << 0); // SQ1 = Channel 0 (PA0)
ADC1->SQR3 |= (1 << 5); // SQ2 = Channel 1 (PA1)

// Configure sampling time for both channels (set to 84 cycles)
ADC1->SMPR2 |= (0x4 << 0); // Channel 0
ADC1->SMPR2 |= (0x4 << 3); // Channel 1

// Enable ADC
ADC1->CR2 |= ADC_CR2_ADON;

// Short delay for ADC to stabilize
for(volatile int i = 0; i < 10000; i++);
}

// Function to start ADC conversion
void ADC_Start(void) {
// Start regular conversion
ADC1->CR2 |= ADC_CR2_SWSTART;
}

// Function to read ADC value
uint16_t ADC_Read(void) {
// Wait for conversion to complete
while(!(ADC1->SR & ADC_SR_EOC));

// Return converted value
return ADC1->DR;
}

//uint32_t value1 = 0;
//uint32_t value2 = 0;
//
//void ADCInit(void) {
//	RCC->APB2ENR |= (1U << 8);
//
//	RCC->AHB1ENR |= (1U << 0);
//
//	GPIOA->MODER |= (3U << 0);	// PA0
//	GPIOA->MODER |= (3U << 2);	// PA1
//
////	ADC1->CR1 &= (1U << 24);
////	ADC1->CR1 &= (1U << 25);
//	ADC1->CR1 = 0;
//	ADC1->CR1 |= (1U << 8);	//SCAN mode
//
////	ADC1->CR2 &= (1U << 0);
//	ADC1->CR2 = 0;
//	ADC1->CR2 |= (1U << 1);	//CONT
//
//	ADC1->SQR1 = (1U << 20);	//L[3:0] 0001 - 2 conversions
//
//	ADC1->SQR3 = 0 | (1U << 5);	// CH0 and CH1
//
//	ADC->CCR = 0;
//
//	ADC1->CR2 |= (1U << 8);	//EOCS = 1 which enables EOC at each conversion in a sequence
//
//	ADC1->CR2 = (1U << 0);	//ADC ON
//}
//
//uint32_t ADCRead(void) {
//	ADC1->CR2 |= (1U << 30);	// SWSTART
//
//	while (!(ADC1->SR & (1U << 1)))
//		// EOC
//		;
//
//	return (ADC1->DR);
//}

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
  ADC_Init();
//  ADCInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ADC_Start();

	  // Read first channel (PA0)
	  adc_value1 = ADC_Read();

	  // Second value will be automatically converted due to continuous mode
	  adc_value2 = ADC_Read();

	  // Do something with the values...

	  // Add delay if needed
	  for(volatile int i = 0; i < 10000; i++);

//		ADC1->CR2 |= (1U << 30);	// SWSTART
//
//		while (!(ADC1->SR & (1U << 1)))
//			// EOC
//			;
//		value1 = ADC1->DR;
//
//		HAL_Delay(1000);
//
//		while (!(ADC1->SR & (1U << 1)))
//			// EOC
//			;
//		value2 = ADC1->DR;
//
//		HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
