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
volatile char usartRecieved = 0;
volatile uint32_t usartFlag = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	//gpioc, alternate functions for usart3
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	GPIOC->AFR[0] |= (1<<16 | 1<<20);
	GPIOC->MODER |= (1<<9 | 1<<11);
	
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->OTYPER &= ~(1<<6 | 1<<7 | 1<<8 | 1<<9);
	GPIOC->OSPEEDR &= ~(1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->PUPDR &= ~(1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 | 1<<17 | 1<<18 | 1<<19);
	GPIOC->ODR |= (1<<9 | 1<<8 | 1<<7 | 1<<6);
	
	//enable usart3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	//set BAUD to 115200 bits/s
	//BRR of 70 gives error of -0.8%
	USART3->BRR = (HAL_RCC_GetHCLKFreq() / 115200)+1;//+1; //69 + 1
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 0);
	
	//USART3->CR1 |= 1<<5;
	//enable transmit/recieve/enable periph
	USART3->CR1 |= 1<<5; //RX NE Interrupt enable;
	USART3->CR1 |= (1<<0 | 1<<1 | 1<<2 | 1<<3 ); //maybe get rid of bit 1
	
	
  while (1){
		//transmitChar('t');
		transmitString("CMD>");
		while (usartFlag == 0){;}//will exit once the interrupt happens. 
		char color = usartRecieved;
		usartFlag = 0;

		transmitChar(color);
		GPIOC->ODR ^= 1<<9;
		while (usartFlag == 0){;}//will exit once the interrupt happens. 
		char number = usartRecieved;
		usartFlag = 0;
		transmitChar(number);
		GPIOC->ODR ^= 1<<9;

		//oh hey look! a nasty switch-inside-switch! this could be done a whole lot better!
		switch(color){
			case 'r': //--------------red
				//
			  switch(number){
					case '0':
						GPIOC->ODR &= ~(1<<6); 
						break;
					case '1':
						GPIOC->ODR |= (1<<6);
						break;
					case '2':
						GPIOC->ODR ^= (1<<6);
						break;
					default: transmitString("<ERROR! Unrecognized command! ");
					break;
				}
				//
				break;
			case 'g': //----------green
				//
				switch(number){
					case '0':
						GPIOC->ODR &= ~(1<<9); 
						break;
					case '1':
						GPIOC->ODR |= (1<<9);
						break;
					case '2':
						GPIOC->ODR ^= (1<<9);
						break;
					default: transmitString("<ERROR! Unrecognized command! ");
					break;
				}
				//
				break;
			case 'b': //-----------blue
				//
				switch(number){
					case '0':
						GPIOC->ODR &= ~(1<<7); 
						break;
					case '1':
						GPIOC->ODR |= (1<<7);
						break;
					case '2':
						GPIOC->ODR ^= (1<<7);
						break;
					default: transmitString("<ERROR! Unrecognized command! ");
					break;
				}
				//
				break;
			case 'o': //-------------orange
				//
				switch(number){
					case '0':
						GPIOC->ODR &= ~(1<<8); 
						break;
					case '1':
						GPIOC->ODR |= (1<<8);
						break;
					case '2':
						GPIOC->ODR ^= (1<<8);
						break;
					default: transmitString("<ERROR! Unrecognized command! ");
					break;
				}
				//
				break;
			default: //-------------------unrecognized command
				transmitString("<ERROR! Unrecognized color! "); 
				break;
		}
			
		//OLD CODE FOR REFERENCE BELOW	
		
		//char myletter = 'w';
		//transmitString("Howdy!>>");
//		while (!((USART3->ISR)  & 1<<5)){;}//recieve reg is empty
//		char c = USART3->RDR;
//		transmitChar(c);
//		transmitChar(' ');
//			switch(c){
//				case 'r':
//				GPIOC->ODR ^= 1<<6;
//				break;
//				case 'g':
//					GPIOC->ODR ^= (1<<9);
//				break;
//				case 'b':
//					GPIOC->ODR ^= (1<<7);
//				break;
//				case 'o':
//					GPIOC->ODR ^= (1<<8);
//				break;
//				default: transmitString("ERROR! ");break;
//			}
				
		//GPIOC->ODR ^= 1<<9;
		//HAL_Delay(400);
	}
}


void USART3_4_IRQHandler(void){
	GPIOC->ODR ^= 1<<9;
	usartRecieved = USART3->RDR;
	usartFlag = 1;
}

void transmitChar(char c){
	//GPIOC->ODR ^= 1<<8;
	while (!((USART3->ISR) & (1<<6))){;}
	USART3->TDR = c;
}

void transmitString(char* s){
	uint32_t i = 0;
	while (s[i] != '\0'){
		transmitChar(s[i]);
		i = i+1;
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
