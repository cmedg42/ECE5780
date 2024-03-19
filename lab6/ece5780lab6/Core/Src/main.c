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
void SystemClock_Config(void);

void initleds(){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->OTYPER &= ~(1<<6 | 1<<7 | 1<<8 | 1<<9);
	GPIOC->OSPEEDR &= ~(1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->PUPDR &= ~(1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 | 1<<17 | 1<<18 | 1<<19);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	initleds();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable gpioA
	GPIOA->MODER |= GPIO_MODER_MODER1;//PA1 is now in analog mode
	GPIOA->PUPDR &= ~(1<<3 | 1<<2); //PUPDR for PA1 is none
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;//enable ADC
	
	//configure ADC
	ADC1->CFGR1 |= ADC_CFGR1_CONT | (1<<4); //continuous mode, 8 bit res part 1
	ADC1->CFGR1 &= ~(1<<3);//8 bit res part 2
	ADC1->CFGR1 &= ~(1<<11 | 1<<10);//HARDWARE trigger disabled
	ADC1->CHSELR |= (1<<1); //ADC channel enable 1
	
	//CALIBRATION
	if ((ADC1->CR & ADC_CR_ADEN) != 0) {//check ADEN = 0
		ADC1->CR |= ADC_CR_ADDIS; //clear aden must be done by setting AD DiS
	} 
	while ((ADC1->CR & ADC_CR_ADEN) != 0) {}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; //clear DMAEN
	ADC1->CR |= ADC_CR_ADCAL; //start calibration
	while ((ADC1->CR & ADC_CR_ADCAL) != 0)  {}//wait for adcal = 0
	uint8_t factor = ADC1->DR & (1<<6|1<<5|1<<4|1<<3|1<<2|1<<1|1<<0);
		
		
//ready
		/* (1) Ensure that ADRDY = 0 */ /* (2) Clear ADRDY */ /* (3) Enable the ADC */ /* (4) Wait until ADC ready */ 
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */ {
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */ 
	} 
	ADC1->CR |= ADC_CR_ADEN; /* (3) */ 
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */ {
  /* For robust implementation, add here time-out management */ 
	}
	
	//start conversion
	ADC1->CR |= (1<<2);//ADC conversion start
	
  while (1)
  {
		uint8_t read = ADC1->DR;
		GPIOC->ODR &= ~(1<<6|1<<7|1<<8|1<<9);//turn all off RBOG
		
		//testing values
		if (read > 10){GPIOC->ODR |= (1<<6);}
		if (read > 50) {GPIOC->ODR |= (1<<7);}
		if (read > 100){GPIOC->ODR |= (1<<8);}
		if (read > 200){GPIOC->ODR |= (1<<9);}
  }
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
