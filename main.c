/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

AD5932_InitTypeDef 						hAD5932i;

ADF4360_InitTypeDef 					hADF4360i;
ADF4360_RCounterTypeDef				hADF4360_Ri;
ADF4360_NCounterTypeDef				hADF4360_Ni;
ADF4360_ControlRegTypeDef			hADF4360_CTRLi;

double att_steps[6] = {16, 8, 4, 2, 1, 0.5};

uint8_t 					ser_att = 0;
uint8_t 					*SPI1_RX_BUFF;
uint8_t 					*SPI1_TX_BUFF;
uint8_t 					*SPI2_RX_BUFF;
uint8_t 					*SPI2_TX_BUFF;
uint8_t 					*UART_RX_BUFF;
uint8_t 					*UART_TX_BUFF;
uint8_t						UART_first_byte;
uint8_t 					uart_command;
uint8_t						adc_enable;
uint8_t						tim2_overflow = 0;
uint8_t 					adc_counter = 0;
uint8_t						UART_BIG_TX_BUFF[3000];
uint8_t 					average_count = AV_COUNT;

long_std_union 		adc_result;
std_union					UART_length;

uint16_t 												adc_result_massive[AV_COUNT][1000];
uint16_t 												adc_result_counter = 0;
uint16_t 												adc_global_counter = 0;
uint16_t 												detected_signal = 0;
uint16_t												rand_num = 0;

/******************************* test parameters ***************************************/
double													center_freq = FREQ_OUT;
double													band_freq = BASE_BAND;
uint16_t												steps_freq = SWEEP_LEN;
double													single_step_freq = BASE_FREQ_STEP;

/******************************* working parameters ************************************/
double 													center_freq_a_1[18];
double 													band_freq_a_1[18];
double 													single_step_freq_a_1[18];
uint16_t												steps_freq_a_1[18];
uint8_t 												temp_k = 0;
uint8_t 												antenna_start = 0;
uint8_t													antenna_stop = 0;
uint8_t													tag_start[3];
uint8_t													tag_stop[3];
uint8_t													mode = MODE_TEST;

const float 										RAND_MAX_F = RAND_MAX;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
	SPI1_RX_BUFF = (uint8_t *)malloc(2);
	SPI1_TX_BUFF = (uint8_t *)malloc(2);
	
	SPI2_RX_BUFF = (uint8_t *)malloc(3);
	SPI2_TX_BUFF = (uint8_t *)malloc(3);
	
	UART_RX_BUFF = (uint8_t *)malloc(11);
	UART_TX_BUFF = (uint8_t *)malloc(11);
	strcpy((char*)UART_TX_BUFF, "Hello World");
	
	memset(SPI1_RX_BUFF, 0, sizeof(SPI1_RX_BUFF));
	memset(SPI1_TX_BUFF, 0, sizeof(SPI1_TX_BUFF));
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
	MX_TIM2_Init();
	DWT_Init();
  /* USER CODE BEGIN 2 */
	attenuator_init(ATT_ADDRESS);																														//attenuator TX address select
	adc_result.listd = 0;																																		//adc_result clear
	std_union temp;
	HAL_GPIO_WritePin(RF_POWER_SUP_CTRL_GPIO_Port, RF_POWER_SUP_CTRL_Pin, GPIO_PIN_SET);		//Turn on RF power
	HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);													//Turn on RS485 receiver
	single_step_freq = band_freq / steps_freq;																							//Determine frequency discrete
	antenna_start = 2;
	tag_start[0] = 1;
	tag_start[1] = 0;
	tag_start[2] = 0;
	
	init_transmitter();
	select_path(TRANSMITTER);
	switch_PE_handle(RF2);
	attenuator_set_att(10);
	//select_path(RECEIVER);

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t temp_container = 0;
		uint32_t temp_container_1 = 0;
		uint16_t temp_counter = 0;
		if(adc_enable == ON)
		{
			if(mode == MODE_TEST)
			{
				switch_PE_handle(RF2);
				for(int t = 0; t < average_count; t++)
				{
					for(int i = 0; i < steps_freq; i++)
					{
						start_gen_1(center_freq, band_freq, i);
						select_path(TRANSMITTER);
						HAL_TIM_Base_Start_IT(&htim2);				
						while(tim2_overflow != 1);
						
						select_path(RECEIVER);
						HAL_TIM_Base_Start_IT(&htim2);
						/*while(tim2_overflow != 0)
						{
							HAL_ADC_Start(&hadc1);
							HAL_ADC_PollForConversion(&hadc1, 1);
							temp_container = HAL_ADC_GetValue(&hadc1);
							if(temp_container >= temp_container_1)
							{
								temp_container_1 = temp_container;
							}					
							//temp_counter++;
						}*/
						HAL_ADC_Start(&hadc1);
						HAL_ADC_PollForConversion(&hadc1, 1);
						temp_container = HAL_ADC_GetValue(&hadc1);
						while(tim2_overflow != 0);
						//temp_container /= temp_counter;
						adc_result_massive[t][adc_result_counter] = temp_container;
						/*if(temp_container_1 >= temp_container)
						{
							adc_result_massive[t][adc_result_counter] = temp_container_1;
						}
						else 
						{
							adc_result_massive[t][adc_result_counter] = temp_container;
						}*/
						
						adc_result_counter++;
						temp_counter = 0;
						temp_container = 0;
					}
					if(t != (average_count - 1))
					{
						adc_result_counter = 0;	
					}
				}
				/******************* avareage filter ********************/
				/*for(int i = 0; i < adc_result_counter; i++)
				{
					for(int t = 1; t < average_count; t++)
					{
						adc_result_massive[0][i] += adc_result_massive[t][i];
					}
					adc_result_massive[0][i] /= average_count;
				}*/
				
				/******************* median filter **********************/
				uint16_t min = 0;
				for(int i = 0; i < adc_result_counter; i++)
				{
					for(int t = 0; t < average_count - 1; t++)
					{
						for(int y = t + 1; y < average_count; y++)
						{
							if(adc_result_massive[t][i] > adc_result_massive[y][i])
							{
								min = adc_result_massive[y][i];
								adc_result_massive[y][i] = adc_result_massive[t][i];
								adc_result_massive[t][i] = min;
							}
						}
					}
					adc_result_massive[0][i] = adc_result_massive[(average_count + 1) / 2][i];
				}

				temp.istd = (adc_result_counter*2)+SERVICE_BITS_LEN; //"+2" is for additional account value
				
				UART_BIG_TX_BUFF[0] = SYNCHRO;
				UART_BIG_TX_BUFF[1] = temp.cstd[1];
				UART_BIG_TX_BUFF[2] = temp.cstd[0];
				UART_BIG_TX_BUFF[3] = UART_ADDR;
				UART_BIG_TX_BUFF[4] = ADC_CONT_1SEC;
				for(uint16_t i = 0; i < adc_result_counter; i++)
				{
					temp.istd = adc_result_massive[0][i];
					UART_BIG_TX_BUFF[2*i + 5] = temp.cstd[1];
					UART_BIG_TX_BUFF[2*i + 6] = temp.cstd[0];
				}	
				
				UART_BIG_TX_BUFF[(adc_result_counter*2) + 5] = xor_handler(UART_BIG_TX_BUFF);		
				
				temp.istd = (adc_result_counter*2)+SERVICE_BITS_LEN; 		
				HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart3, UART_BIG_TX_BUFF, temp.istd, 10000);
				HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);	
				adc_result_counter = 0;	
				memset(UART_BIG_TX_BUFF, 0, 3000);	
				adc_enable = 0;
				//while(adc_enable == 0);	
			}
			else if(mode == MODE_WORK)
			{
				for(int a = 0; a < 3; a++)
				{
					if(antenna_start & ((1 << a) & 0x07))
					{
						switch_PE_handle(a);
						for(int y = 0; y < 6; y++)
						{
							if(tag_start[y] & ((1 << y) & 0x3F))
							{
								for(int t = 0; t < average_count; t++)
								{
									for(int i = 0; i < steps_freq; i++)
									{
										attenuator_set_att(31.5);		
										start_gen_1(center_freq, band_freq, i);
										attenuator_set_att(0);	
										select_path(TRANSMITTER);
										HAL_TIM_Base_Start_IT(&htim2);				
										while(tim2_overflow != 1);
										
										select_path(RECEIVER);
										HAL_TIM_Base_Start_IT(&htim2);
										while(tim2_overflow != 0)
										{
											HAL_ADC_Start(&hadc1);
											HAL_ADC_PollForConversion(&hadc1, 1);
											temp_container += HAL_ADC_GetValue(&hadc1);
											temp_counter++;
										}
										temp_container /= temp_counter;
										adc_result_massive[t][adc_result_counter] = temp_container;
										adc_result_counter++;
										temp_counter = 0;
										temp_container = 0;
									}
									if(t != (average_count - 1))
									{
										adc_result_counter = 0;	
									}
								}
									/******************* avareage filter ********************/
									/*for(int i = 0; i < adc_result_counter; i++)
									{
										for(int t = 1; t < average_count; t++)
										{
											adc_result_massive[0][i] += adc_result_massive[t][i];
										}
										adc_result_massive[0][i] /= average_count;
									}*/
									
									/******************* median filter **********************/
									uint16_t min = 0;
									for(int i = 0; i < adc_result_counter; i++)
									{
										for(int t = 0; t < average_count - 1; t++)
										{
											for(int y = t + 1; y < average_count; y++)
											{
												if(adc_result_massive[t][i] > adc_result_massive[y][i])
												{
													min = adc_result_massive[y][i];
													adc_result_massive[y][i] = adc_result_massive[t][i];
													adc_result_massive[t][i] = min;
												}
											}
										}
										adc_result_massive[0][i] = adc_result_massive[(average_count + 1) / 2][i];
									}

									temp.istd = (adc_result_counter*2)+SERVICE_BITS_LEN; //"+2" is for additional account value
									
									UART_BIG_TX_BUFF[0] = SYNCHRO;
									UART_BIG_TX_BUFF[1] = temp.cstd[1];
									UART_BIG_TX_BUFF[2] = temp.cstd[0];
									UART_BIG_TX_BUFF[3] = UART_ADDR;
									UART_BIG_TX_BUFF[4] = ADC_CONT_1SEC;
									for(uint16_t i = 0; i < adc_result_counter; i++)
									{
										temp.istd = adc_result_massive[0][i];
										UART_BIG_TX_BUFF[2*i + 5] = temp.cstd[1];
										UART_BIG_TX_BUFF[2*i + 6] = temp.cstd[0];
									}	
									
									UART_BIG_TX_BUFF[(adc_result_counter*2) + 5] = xor_handler(UART_BIG_TX_BUFF);		
									
									temp.istd = (adc_result_counter*2)+SERVICE_BITS_LEN; 		
									HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_SET);
									HAL_UART_Transmit(&huart3, UART_BIG_TX_BUFF, temp.istd, 10000);
									HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);	
									adc_result_counter = 0;	
									memset(UART_BIG_TX_BUFF, 0, 3000);	
									//adc_enable = 0;
									//while(adc_enable == 0);
								
							}
						}
					}					
				}
			}
			else
			{
				adc_enable = OFF;
			}					
		} 
    /* USER CODE END WHILE */   
  }
	/* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; 
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIME_TX; //252
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	//__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, X14_Pin|X15_Pin|X16_Pin|X10_Pin 
                          |X11_Pin|X12_Pin|X13_Pin|HMC8073_A0_Pin 
                          |HMC8073_A1_Pin|HMC8073_A2_Pin|HMC8073_CLK_Pin|HMC8073_LE_Pin 
                          |HMC8073_SI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PE42430_V1_Pin|PE42430_V2_Pin|PE42430_V3_Pin|AD5932_FSYNC_Pin 
                          |AD5932_CTRL_Pin|RF_CTRL_AMP_TR_Pin|RF_CTRL_AMP_RC_Pin|TEST_LED_Pin 
                          |X9_Pin|RF_POWER_SUP_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AD5932_INTER_Pin|RF_CTRL_2_Pin|ADF4360_CE_Pin|ADF4360_LE_Pin 
                          |X5_Pin|X6_Pin|X7_Pin|X8_Pin 
                          |RE_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_CTRL_0_GPIO_Port, RF_CTRL_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X14_Pin X15_Pin X16_Pin X10_Pin 
                           X11_Pin X12_Pin X14C3_Pin HMC8073_A0_Pin 
                           HMC8073_A1_Pin HMC8073_A2_Pin */
  GPIO_InitStruct.Pin = X14_Pin|X15_Pin|X16_Pin|X10_Pin 
                          |X11_Pin|X12_Pin|X13_Pin|HMC8073_A0_Pin 
                          |HMC8073_A1_Pin|HMC8073_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE42430_V1_Pin PE42430_V2_Pin PE42430_V3_Pin RF_CTRL_AMP_TR_Pin 
                           RF_CTRL_AMP_RC_Pin */
  GPIO_InitStruct.Pin = PE42430_V1_Pin|PE42430_V2_Pin|PE42430_V3_Pin|RF_CTRL_AMP_TR_Pin 
                          |RF_CTRL_AMP_RC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AD5932_FSYNC_Pin AD5932_CTRL_Pin TEST_LED_Pin X9_Pin 
                           RF_POWER_SUP_CTRL_Pin */
  GPIO_InitStruct.Pin = AD5932_FSYNC_Pin|AD5932_CTRL_Pin|TEST_LED_Pin|X9_Pin 
                          |RF_POWER_SUP_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AD5932_SYNCOUT_Pin ADF4360_MUX_Pin */
  GPIO_InitStruct.Pin = AD5932_SYNCOUT_Pin|ADF4360_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AD5932_INTER_Pin ADF4360_CE_Pin X5_Pin X6_Pin 
                           X7_Pin X8_Pin RE_DE_Pin */
  GPIO_InitStruct.Pin = AD5932_INTER_Pin|ADF4360_CE_Pin|X5_Pin|X6_Pin 
                          |X7_Pin|X8_Pin|RE_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AD5932_MSBOUT_MCU_Pin */
  GPIO_InitStruct.Pin = AD5932_MSBOUT_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD5932_MSBOUT_MCU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_CTRL_2_Pin ADF4360_LE_Pin */
  GPIO_InitStruct.Pin = RF_CTRL_2_Pin|ADF4360_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HMC8073_CLK_Pin HMC8073_LE_Pin HMC8073_SI_Pin */
  GPIO_InitStruct.Pin = HMC8073_CLK_Pin|HMC8073_LE_Pin|HMC8073_SI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_CTRL_0_Pin */
  GPIO_InitStruct.Pin = RF_CTRL_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RF_CTRL_0_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
