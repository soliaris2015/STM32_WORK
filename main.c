/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_tim.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile  uint16_t sys_tick_counter = 0;
volatile uint16_t res;

volatile uint8_t red_bool = 0;
volatile uint8_t green_bool = 0;
volatile uint8_t blue_bool = 0;
volatile uint8_t orange_bool = 0;

volatile uint16_t adc_val = 0;

uint32_t timeOut = 10;
uint8_t rx_buff[1] = {0};
uint8_t command_str[6] = {0,0,0,0,0,0};
uint8_t counter = 0;

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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//
//
//
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//	//HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "READY\n\r", 7);
//	//HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "JOPA\n\r", 6);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_ADC_Start_DMA(&hadc1,&adc_val,1);

	while(1)
	{

	}


	while (1)
	{

		uint32_t adcResult = 0;
		float coeff = 3.0/4095.0;
		char buff[20] = {0};

		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 100);

		adcResult = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);

		res = (uint16_t)(coeff * adcResult * 100.0 + 1);
		uint16_t temp = res/100;
		uint16_t temp2 = res%100;
		//sprintf(buff,"%d.%d\n\r",temp,temp2);
		//sys_tick_counter = sys_tick_counter + res*100000;

		sprintf(buff,"%d\n\r",res);
		HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff),10);

		//if (HAL_OK == HAL_UART_Receive_IT(&huart2, rx_buff, 1))
		//{
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//									HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//									HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//									HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		//}












		//		if (HAL_OK == HAL_UART_Receive_IT(&huart2, rx_buff, 1))
//		{
//			if(rx_buff[0] == '/')
//			{
//				uint8_t i = 0;
//				for(i = 0; i < 6; i++)
//				{
//					command_str[i] = 0;
//				}
//				counter = 0;
//				HAL_UART_Transmit(&huart2,  "\n\r", 2, 2);
//				rx_buff[0] = 0;
//				continue;
//			}
//
//
//			if(counter > 6)
//			{
//				HAL_UART_Transmit(&huart2,  " =LENGHT ERROR=", 15, timeOut);
//				uint8_t i = 0;
//				for(i = 0; i < 6; i++)
//				{
//					command_str[i] = 0;
//				}
//				counter = 0;
//				HAL_UART_Transmit(&huart2,  "\n\r", 2, 2);
//				rx_buff[0] = 0;
//				continue;
//			}
//
//			//=====ÝÕÎ====
//			HAL_UART_Transmit(&huart2,  rx_buff, 1, 2);
//			//============
//
//			command_str[counter] = rx_buff[0];
//			if(command_str[0] == 'R')
//			{
//				if(command_str[1] == 'E')
//				{
//					if(command_str[2] == 'D')
//					{
//						red_bool = '1';
//						HAL_UART_Transmit(&huart2,  " COMPLITE\n\r", 11, timeOut);
//						uint8_t i = 0;
//						for(i = 0; i < 6; i++)
//						{
//							command_str[i] = 0;
//						}
//						counter = 0;
//					}
//					else
//						counter++;
//				}
//				else
//					counter++;
//			}
//			else if(command_str[0] == 'B')
//			{
//				if(command_str[1] == 'L')
//				{
//					if(command_str[2] == 'U')
//					{
//						if(command_str[3] == 'E')
//						{
//							blue_bool = '1';
//							HAL_UART_Transmit(&huart2,  " COMPLITE\n\r", 11, timeOut);
//							uint8_t i = 0;
//							for(i = 0; i < 6; i++)
//							{
//								command_str[i] = 0;
//							}
//							counter = 0;
//						}
//						else
//							counter++;
//					}
//					else
//						counter++;
//				}
//				else
//					counter++;
//			}
//			else if(command_str[0] == 'O')
//			{
//				if(command_str[1] == 'R')
//				{
//					if(command_str[2] == 'A')
//					{
//						if(command_str[3] == 'N')
//						{
//							if(command_str[4] == 'G')
//							{
//								if(command_str[5] == 'E')
//								{
//									orange_bool = '1';
//									HAL_UART_Transmit(&huart2,  " COMPLITE\n\r", 11, timeOut);
//									uint8_t i = 0;
//									for(i = 0; i < 6; i++)
//									{
//										command_str[i] = 0;
//									}
//									counter = 0;
//								}
//								else
//									counter++;
//							}
//							else
//								counter++;
//						}
//						else
//							counter++;
//					}
//					else
//						counter++;
//				}
//				else
//					counter++;
//			}
//			else if(command_str[0] == 'G')
//			{
//				if(command_str[1] == 'R')
//				{
//					if(command_str[2] == 'E')
//					{
//						if(command_str[3] == 'E')
//						{
//							if(command_str[4] == 'N')
//							{
//								green_bool = '1';
//								HAL_UART_Transmit(&huart2,  " COMPLITE\n\r", 11, timeOut);
//								uint8_t i = 0;
//								for(i = 0; i < 6; i++)
//								{
//									command_str[i] = 0;
//								}
//								counter = 0;
//							}
//							else
//								counter++;
//						}
//						else
//							counter++;
//					}
//					else
//						counter++;
//				}
//				else
//					counter++;
//			}
//			else if(command_str[0] == 'C')
//			{
//				if(command_str[1] == 'L')
//				{
//					HAL_UART_Transmit(&huart2,  " COMPLITE\n\r", 11, timeOut);
//					red_bool = '0';
//					green_bool = '0';
//					blue_bool = '0';
//					orange_bool = '0';
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//					uint8_t i = 0;
//					for(i = 0; i < 6; i++)
//					{
//						command_str[i] = 0;
//					}
//					counter = 0;
//				}
//				else
//					counter++;
//			}
//			else
//			{
//				HAL_UART_Transmit(&huart2,(uint8_t *) " COMMAND NOT FOUND\n\r", 20, timeOut);
//
//				counter = 0;
//				uint8_t i = 0;
//				for(i = 0; i < 6; i++)
//				{
//					command_str[i] = 0;
//				}
//			}
		}





//		if (HAL_OK == HAL_UART_Receive_IT(&huart2, rx_buff, 6))
//		{
//			HAL_UART_Transmit(&huart2, (uint8_t *) "O\n\r", 3, timeOut);
//
//			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//			if (0 == strcmp(rx_buff,"GE"))
//			{
//				green_bool = 1;
//				HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//			}
//			else if (0 == strcmp(rx_buff,"RE"))
//			{
//				red_bool = 1;
//				HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//			}
//			else if (0 == strcmp(rx_buff,"BL"))
//			{
//				blue_bool = 1;
//				HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//			}
//			else if (0 == strcmp(rx_buff,"OR"))
//			{
//				orange_bool = 1;
//				HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//			}
//			else if (0 == strcmp(rx_buff,"CL"))
//			{
//				red_bool = 0;
//				green_bool = 0;
//				blue_bool = 0;
//				orange_bool = 0;
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//
//				HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//			}
//			else if (rx_buff[0] == 0 && rx_buff[1] == 0 && rx_buff[2] == 0)
//			{
//				continue;
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//				HAL_UART_Transmit(&huart2, (uint8_t *) "ERROR\n\r", 7, timeOut);
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//			}
//			rx_buff[0] = 0;
//			rx_buff[1] = 0;
//			rx_buff[2] = 0;
//			rx_buff[3] = 0;
//			rx_buff[4] = 0;
//			rx_buff[5] = 0;
//		}
//		HAL_Delay(100);
//		if (red_bool == '1')
//		{
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//		}
//		if (blue_bool == '1')
//		{
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//		}
//		if (green_bool == '1')
//		{
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//		}
//		if (orange_bool == '1')
//		{
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//		}
//
//		HAL_Delay(100);
//	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//
//
//
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//
//		HAL_UART_Receive(&huart2, &rx_buff, 1, timeOut);
//
//		if (rx_buff == 'G')
//		{
//			//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//			green_bool = '1';
//			HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//		}
//		else if (rx_buff == 'R')
//		{
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//			red_bool = '1';
//			HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//		}
//		else if (rx_buff == 'B')
//		{
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//			blue_bool = '1';
//			HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//		}
//		else if (rx_buff == 'O')
//		{
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			orange_bool = '1';
//			HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//		}
//		else if (rx_buff == 'C')
//		{
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//			red_bool = '0';
//			green_bool = '0';
//			blue_bool = '0';
//			orange_bool = '0';
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//
//			HAL_UART_Transmit(&huart2, (uint8_t *) "OK\n\r", 4, timeOut);
//		}
//		else if (rx_buff == 0)
//		{
//			continue;
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart2, (uint8_t *) "ERROR\n\r", 7, timeOut);
//		}
//	}

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OTG_FS_PowerSwitchOn_Pin|DEBUG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin DEBUG_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 I2S3_SCK_Pin PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|I2S3_SCK_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
