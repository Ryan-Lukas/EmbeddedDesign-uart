/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void TDRSend(char);
void isBusy(void);
void getArrayValue(char*);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
	
	
	
	//Q1. putting in string
	//Q2.changing main to be able to transmit data
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= (0x1 << 18);

	
	
	
	//pin pC4
	GPIOC->MODER |= (0x2 << 8);
	GPIOC->AFR[0] |= (0x1 << 16);

	
	//pin PC5
	GPIOC->MODER |= (0x2 << 10);
	GPIOC->AFR[0] |= (0x1 << 20);
	
	//periph led init
	GPIOC->MODER |= (0x1 << 12);//pin6 red
	GPIOC->OTYPER &= ~(0x1 << 6);
	GPIOC->OSPEEDR &= ~(0x3 << 12);
	GPIOC->PUPDR &= ~(0x3 <<12);
	
	GPIOC->MODER |= (0x1 << 14);//pin7 blue
	GPIOC->OTYPER &= ~(0x1 << 7);
	GPIOC->OSPEEDR &= ~(0x3 << 14);
	GPIOC->PUPDR &= ~(0x3 <<14);
	
	
	GPIOC->MODER |= (0x01 <<16); //pc8 orange
	GPIOC->OTYPER &= ~(0x1 <<8);
	GPIOC->OSPEEDR &= ~(0x3 <<16);
	GPIOC->PUPDR &= ~(0x3 << 16);
	
	GPIOC->MODER |= (0x01 <<18); //pc9 yellow
	GPIOC->OTYPER &= ~(0x1 <<9);
	GPIOC->OSPEEDR &= ~(0x3 <<18);
	GPIOC->PUPDR &= ~(0x3 << 18);
	
	
	//setting baud rate
	uint32_t freq = HAL_RCC_GetHCLKFreq();
	int baud = freq/115200;
	USART3->BRR = baud; //69 if not work
	
	//enable RE
 USART3->CR1 |= (0x1 << 2);
 //enable TE
 USART3->CR1 |= (0x1 << 3);
 USART3->CR1 |= (0x1);
 		int counter = 0;
		int inputc = 0;
		char loc= 0;
		char secloc =0;
  while (1)
  {

		if(!(USART3->ISR & (1 << 5))){
			char* cmd = "CMD?\n";
			if(counter == 0){
				getArrayValue(cmd);
				counter ++;
			}
		}else{
			if(inputc == 0){
				loc = USART3->RDR;
				USART3->ISR &= (1 << 5);
				inputc++;
			}
		}
	
		if(inputc == 1){
			if(USART3->ISR & (1 <<5)){
				inputc++;
			}
		}else if(inputc == 2){
			inputc++;
			secloc = USART3->RDR;
		}else if(inputc == 3){
			
			switch(loc){
				case('r'):{
					if(secloc == '0'){
						GPIOC->ODR &= ~(0x1 << 6);
						char* which = "\t INPUT:	r0\n";
						getArrayValue(which);
					}
					else if(secloc == '1'){
							GPIOC->ODR |= (0x1 << 6);
						char* which = "\t INPUT:	r1\n";
						getArrayValue(which);
					}else if(secloc == '2'){
						GPIOC->ODR ^=(0x1 << 6);
						char* which = "\t INPUT:	r2\n";
						getArrayValue(which);
					}
					break;
				}
				
				case('b'):{
					if(secloc == '0'){
						GPIOC->ODR &= ~(0x1 << 7);
						char* which = "\t INPUT:	b0\n";
						getArrayValue(which);
					}
					else if(secloc == '1'){
							GPIOC->ODR |= (0x1 << 7);
							char* which = "\t INPUT:	b1\n";
							getArrayValue(which);
					}else if(secloc == '2'){
						GPIOC->ODR ^=(0x1 << 7);
						char* which = "\t INPUT:	b2\n";
						getArrayValue(which);
					}
					break;
				}
				
					case('o'):{
					if(secloc == '0'){
						GPIOC->ODR &= ~(0x1 << 8);
						char* which = "\t INPUT:	o0\n";
						getArrayValue(which);
					}
					else if(secloc == '1'){
							GPIOC->ODR |= (0x1 << 8);
							char* which = "\t INPUT:	o1\n";
						getArrayValue(which);
					}else if(secloc == '2'){
						GPIOC->ODR ^=(0x1 << 8);
						char* which = "\t INPUT:	o2\n";
						getArrayValue(which);
					}
					break;
				}
				
				case('y'):{
					if(secloc == '0'){
						GPIOC->ODR &= ~(0x1 << 9);
						char* which = "\t INPUT:	y0\n";
						getArrayValue(which);
					}
					else if(secloc == '1'){
							GPIOC->ODR |= (0x1 << 9);
						char* which = "\t INPUT:	y1\n";
						getArrayValue(which);
					}else if(secloc == '2'){
						GPIOC->ODR ^=(0x1 << 9);
						char* which = "\t INPUT:	y2\n";
						getArrayValue(which);
					}
					break;
				}
				
				default:{
					char* error = "ERROR\n";
					getArrayValue(error);	
					break;
				}
			}
			 
			
			inputc = 0;
			counter = 0;
		}
		  
  /* USER CODE END 3 */

	}
}

//loop through array
void getArrayValue(char* location){
	int count = 0;
	while(location[count] != '\0'){
		TDRSend(location[count]);
		count++;
	}
	return;
}



//send bit func
void TDRSend(char send){
	while(!(USART3->ISR & (1 << 7))){
		
	}
	USART3->TDR = send;
	
}




/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


