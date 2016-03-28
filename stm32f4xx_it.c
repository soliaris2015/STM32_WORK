/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h" //!!
#include "stm32f4xx_it.h" //!!

/* USER CODE BEGIN 0 */
extern uint16_t sys_tick_counter;
extern uint8_t red_bool;
extern uint8_t green_bool;
extern uint8_t blue_bool;
extern uint8_t orange_bool;
extern UART_HandleTypeDef huart2;


extern uint16_t adc_val;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  sys_tick_counter = sys_tick_counter + 1;
	if (sys_tick_counter > 100)
	{
//		TIM_OC_InitTypeDef sConfigOC;

//		 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
//		 sConfigOC.OCMode = TIM_OCMODE_PWM1;
//		   sConfigOC.Pulse = (adc_val >> 4)*256;
//		   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//		   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//		   HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
//		 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//
		 uint16_t adc_cor = (adc_val >> 3)* 2.75;
		TIM4->CCR2= 800 + adc_cor;

		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		sys_tick_counter = 0;
		//HAL_UART_Transmit(&huart2, (uint8_t *) "9\n\r", 3,10);

	}
}
  /* USER CODE END SysTick_IRQn 1 */


/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	red_bool = 0;
	green_bool = 0;
	blue_bool = 0;
	orange_bool = 0;

//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_UART_Transmit(&huart2, (uint8_t *) "BUTTON PRESSED\n\r", 16,10);



  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */



  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */


  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
 // adc_val = HAL_ADC_GetValue(&hadc1);
	  //HAL_UART_Transmit(&huart2, (uint16_t *) adc_val, 1,10);
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

	//if (TIM4 != RESET)
	//{

//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//								HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//								HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//								HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);


	//}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
						//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
						//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
						//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
						//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
