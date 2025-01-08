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
uint16_t pot_value;
float pot_voltage;

uint16_t lm35_value;
float lm35_voltage;
float temperatureC;

uint16_t ADC_VAL[2] = {0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_Disable (void);
uint16_t ADC_GetVal (void);
void ADC_WaitForConv (void);
void ADC_Start (int channel);
void ADC_Enable (void);
void ADC_Multi_Init(void);
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
	  ADC_Multi_Init();

	  ADC_Enable ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	ADC_Start (0);
	ADC_WaitForConv ();
	ADC_VAL[0] = ADC_GetVal();


	ADC_Start (1);
	ADC_WaitForConv ();
	ADC_VAL[1] = ADC_GetVal();

	pot_voltage = ADC_VAL[0] * (3.3/4095.0);
	lm35_voltage = ADC_VAL[1] * (3.3/4095.0);
	temperatureC = (lm35_voltage) * 100 ;

	HAL_Delay(10);



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

void ADC_Enable (void)
{
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	HAL_Delay(1);
}


void ADC_Start (int channel)
{

	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<<0);    // conversion in regular sequence

	ADC1->SR = 0;        		   // clear the status register

	ADC1->CR2 |= (1<<30);    	   // start the conversion
}


void ADC_WaitForConv (void)
{

	while (!(ADC1->SR & (1<<1)));  // wait for EOC flag to set
}

uint16_t ADC_GetVal (void)
{
	return ADC1->DR;  // Read the Data Register
}

void ADC_Disable (void)
{
	ADC1->CR2 &= ~(1<<0);  // Disable ADC
}

void ADC_Multi_Init(void)
{
    //Enable the clock for GPIOA and ADC1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Enable ADC1 clock

    //Configure PA0 as analog mode
    GPIOA->MODER |= (3U << (0 * 2));       // Set PA0 to analog mode (MODER[1:0] = 11)
    GPIOA->PUPDR &= ~(3U << (0 * 2));      // No pull-up, pull-down (PUPDR[1:0] = 00)

    //Configure PA1 as analog mode
    GPIOA->MODER |= (3U << (1 * 2));       // Set PA0 to analog mode (MODER[1:0] = 11)
    GPIOA->PUPDR &= ~(3U << (1 * 2));      // No pull-up, pull-down (PUPDR[1:0] = 00)

    //Configure the ADC1
    ADC1->CR2 = 0;                         // Ensure ADC is disabled before configuration

    ADC1->CR1 = 0;                         // Use default resolution (12-bit), single conversion mode

	ADC1->CR2 |= (1<<10);    			   // EOC after each conversion

    ADC1->SQR1 |= (1 << 20);               // two conversions

    ADC1->SQR3 |= (0 << (0 * 4));          // First conversion in the sequence is Channel 0 (PA0)
    ADC1->SQR3 |= (1 << (1 * 4));          // First conversion in the sequence is Channel 0 (PA0)

	ADC->CCR |= 1<<16;  		           // PCLK2 divide by 4

    //Set ADC1 sample time for Channel 0
    ADC1->SMPR2 |= (7U << (0 * 3));  // Maximum sampling time (480 cycles) - Channel 0 (POT)

    // Configure sample time for Channel 1 (LM35)
    ADC1->SMPR2 |= (7U << (1 * 3));  // Maximum sampling time (480 cycles)  - Channel 1 (LM35)

    //Enable ADC1
    ADC1->CR2 |= ADC_CR2_ADON;             // Enable the ADC
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
