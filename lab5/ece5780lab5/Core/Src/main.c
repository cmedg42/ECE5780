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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
int doublei2c(char reg, volatile int16_t* read){ //collects two nums
	
	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (1<<16) //numbytes == 1
		| (0x69<<1); //slave address = 0x69
	I2C2->CR2 &= ~(1<<10); //write transfer (bit 10 is 0)
	I2C2->CR2 |= (1<<13);//start bit
	while(1){ //wait for TXIS
		//I2C_ISR_TXIS
		//GPIOC->ODR |= (1<<8);
		if ((I2C2->ISR & (1<<1))){ break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
	}
	
	
	I2C2->TXDR = reg; //addr
	while (1){

		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
	}
	
	//GPIOC->ODR |= (1<<7);//blue

	
	//READ NOW. 
	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (2<<16) //numbytes == 1
		| (0x69<<1); //slave address = 0x6B
	I2C2->CR2 |= (1<<10); //READ transfer (bit 10 is 1)
	I2C2->CR2 |= (1<<13);//start bit set
	
	int8_t h, l;
  int16_t result;
	for(uint32_t i = 0; i < 2; i++){
		while (1){
			if (I2C2->ISR & I2C_ISR_RXNE){break;}
			else if (I2C2->ISR & I2C_ISR_NACKF) {
				//error
				return 1;
			}
		}
		
		if (i == 0){
			l = I2C2->RXDR;
		}else{
			h = I2C2->RXDR;
		}
	}
	
	while (1){
		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
	}
	I2C2->CR2 |= (1<<14);//STOP
	result = (h << 8) | (l);
	*(read) = result;
	return 0;//no error!
}

int i2ctransfer(char reg, char info , volatile char* read){ //slave address = 0x69
	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (2<<16) //numbytes == 1
		| (0x69<<1); //slave address = 0x69
	I2C2->CR2 &= ~(1<<10); //write transfer (bit 10 is 0)
	I2C2->CR2 |= (1<<13);//start bit
	while(1){ //wait for TXIS
		//I2C_ISR_TXIS
		//GPIOC->ODR |= (1<<8);
		if ((I2C2->ISR & (1<<1))){ break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
	}
	
	I2C2->TXDR = reg; //addr
	/*while (1){
		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
	}*/ //COMMENTED OUT
	//TESTESTESTEST
	while(1){ //wait for TXIS
		//I2C_ISR_TXIS
		//GPIOC->ODR |= (1<<8);
		if ((I2C2->ISR & (1<<1))){ break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
	}
	I2C2->TXDR = info;
	while (1){
		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
	}
	//TESTESTESTEST
	//now it is time to read the response 
	
	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (1<<16) //numbytes == 1
		| (0x69<<1); //slave address = 0x6B
	I2C2->CR2 |= (1<<10); //READ transfer (bit 10 is 1)
	I2C2->CR2 |= (1<<13);//start bit set
	while (1){

		if (I2C2->ISR & I2C_ISR_RXNE){break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
	}
	

	while (1){ 
		if (I2C2->ISR & I2C_ISR_TC){break;}
	}
	
	*(read) = I2C2->RXDR;
	I2C2->CR2 |= (1<<14);//STOP
	
	
	
	
	
	return 0;
}

void initleds(){
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->OTYPER &= ~(1<<6 | 1<<7 | 1<<8 | 1<<9);
	GPIOC->OSPEEDR &= ~(1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->PUPDR &= ~(1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 | 1<<17 | 1<<18 | 1<<19);
}

void initgyro(){
	//gyro
	GPIOC->BSRR = (1<<0);
  
}

int main(void)
{
  HAL_Init();
	SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	initleds();
	
	//pin b11
	GPIOB->MODER |= (1<<23 | 0<<22); //alternate function mode for pin B11
	GPIOB->OTYPER |= (1<<11);//open drain output on B11
	GPIOB->AFR[1] |= (1<<12); //set pin B11 to AF 1 (I2C2_SDA)
	
	//pin b13
	GPIOB->MODER |= (1<<27); //alternate function mode for pin b13
	GPIOB->OTYPER |= (1<<13);//open drain B13
	GPIOB->AFR[1] |= (1<<22 | 1<<20);//AF5 for pin b13 (i2c2_SCL)
	
	//pin b14
	GPIOB->MODER |= (1<<28);//output mode
		//pushpull is default
	GPIOB->ODR |= (1<<14);//initialize high
	
	GPIOB->PUPDR |= (1<<22 | 1<<26);
	
	//pin PC0
	GPIOC->MODER |= (1<<0); //PC0 output mode
		//pushpull is default
	GPIOC->ODR |= (1<<0);//initialize high
	
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//i2c2
	I2C2->TIMINGR = (1<< 28)//PRESCALER == 1
		| (0x4 << 20) // SCLDEL == 0x4 
		| (0x2<<16) //SDADEL ==  0x2
		| (0xF<<8)//SCLH == 0xF
		| (0x13<<0);//8SCLL == 0x13
		//| (I2C2->TIMINGR & (0xF << 24)); //reserved bits
	I2C2->CR1 |= (1<<0); //PE bit

	initgyro();
	
	volatile char read, info, reg;

	//i2ctransfer(0x0F, 0x00, &read);
	//if (read == 0xD3){
				//GPIOC->ODR |= (1<<7);//blue

	//} else {
				//GPIOC->ODR |= (1<<9); //green
	//}
	

	reg = 0x20;
	info = 0x0B; //00001011 <- PD ,x+y en
	
	i2ctransfer(reg, info, &read);//reg info read
	
	//////////////////////////////////
	//main loop
	while (1)
  {
		HAL_Delay(100);
		volatile int16_t x, y;
		//collect x
		doublei2c(0xA8, &x); //reg read
		//collect y
		doublei2c(0xAA, &y);
		//decide lights
		const int16_t thresh = 0x01FF;
		if (x < 0-thresh) {GPIOC->ODR |= (1<<8);} else {GPIOC->ODR &= ~(1<<8);} //orange
		if (y < 0-thresh) {GPIOC->ODR |= (1<<7);} else {GPIOC->ODR &= ~(1<<7);} //blue
		if (y > thresh) {GPIOC->ODR |= (1<<6);} else {GPIOC->ODR &= ~(1<<6);} //red
		if (x > thresh) {GPIOC->ODR |= (1<<9);} else {GPIOC->ODR &= ~(1<<9);} //green
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
