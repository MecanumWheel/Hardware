/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "float.h"
#include "Periph_BATT.h"
#include "glcd.h"
#include "glcd_graphics.h"
#include "glcd_text.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId UltrasonicTaskHandle;
osThreadId BatteryControlTaskHandle;
osThreadId IndicatorsTaskHandle;

SemaphoreHandle_t Ultrasonic_ISR_BS;

QueueHandle_t Ultrasonic_Queue[4];
QueueHandle_t UltrasonicToPC;

BaseType_t Ultrasonic_HPTW_Q1 = pdFALSE, Ultrasonic_HPTW_Q2 = pdFALSE;

uint8_t Ultrasonic_flags = 0;
uint8_t Battery_flags = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void UltrasonicTaskFunction(void const * argument);
void BatteryControlTaskFunction(void const * argument);
void IndicatorsTaskFunction(void const * argument);

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  Ultrasonic_ISR_BS = xSemaphoreCreateBinary();
	if(!Ultrasonic_ISR_BS)
		Error_Handler();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(UltrasonicTask, UltrasonicTaskFunction, osPriorityNormal, 0, 128);
  UltrasonicTaskHandle = osThreadCreate(osThread(UltrasonicTask), NULL);
	osThreadDef(BatteryControlTask, BatteryControlTaskFunction, osPriorityNormal, 0, 128);
  BatteryControlTaskHandle = osThreadCreate(osThread(BatteryControlTask), NULL);
	osThreadDef(IndicatorsTask, IndicatorsTaskFunction, osPriorityNormal, 0, 128);
  IndicatorsTaskHandle = osThreadCreate(osThread(IndicatorsTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  Ultrasonic_Queue[0] = xQueueCreate(2, sizeof( uint32_t ) );
	Ultrasonic_Queue[1] = xQueueCreate(2, sizeof( uint32_t ) );
	Ultrasonic_Queue[2] = xQueueCreate(2, sizeof( uint32_t ) );
	Ultrasonic_Queue[3] = xQueueCreate(2, sizeof( uint32_t ) );
	UltrasonicToPC = xQueueCreate(2, sizeof( struct Ultrasonic_Distances ) );
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 24999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 12500;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void UltrasonicTaskFunction(void const * argument){
	uint16_t US_time_acq [4][2];							//For every sensor we get two values, rising edge and falling
	float US_distance [4],US_min_dist;				//Array to hold distances in mm and minimum distance
	struct Ultrasonic_Distances US_RxStruct;	//Struct to be sent to the PC, with distances
	uint32_t channel;													//For timer channel selection
	HAL_TIM_IC_Init(&htim1);									//Activates timer 1, to begin acquiring sensor data
	HAL_TIM_PWM_Init(&htim2);									//Activates timer 2, to begin generating pulses
	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
	while(1){
		//MX_TIM1_Init();													//Sets timer and all channel input captures to rising edge
		for (uint8_t i=0; i<4 ; i++){
			channel = i << 2;											//Selects the right timer channel
			Ultrasonic_flags |= ULTRASONIC_SGNEDGE;	//Sets Scan Edge to rising
			HAL_TIM_IC_Start_IT(&htim1,channel);	//Starts interruptions for input capture
			if(xQueueReceive(Ultrasonic_Queue[i],&US_time_acq[i][0],25/portTICK_PERIOD_MS) == pdTRUE){
				if((Ultrasonic_flags & ULTRASONIC_SGNEDGE) != 0){
					HAL_TIM_IC_Stop_IT(&htim1,channel);
					continue;
				}
				if(xQueueReceive(Ultrasonic_Queue[i],&US_time_acq[i][1],25/portTICK_PERIOD_MS) == pdTRUE)
					Ultrasonic_flags |= (ULTRASONIC_CAPTURE_1 << i);
			}
			HAL_TIM_IC_Stop_IT(&htim1,channel);
		}																														//After all data acquire, begin treatment
		US_min_dist = FLT_MAX;
		for(int i=0; i<4; i++){																			//
			if((Ultrasonic_flags & (ULTRASONIC_CAPTURE_1 << i)) == 0){	//Checks if sensor has been measured
				US_distance[i] = 0;																				//If not, set distance to 0
				continue;
			}
			if(US_time_acq[i][0] < US_time_acq[i][1])
				US_time_acq[i][1] = US_time_acq[i][1] - US_time_acq[i][0];
			else
				US_time_acq[i][1] = 0xFFFF - US_time_acq[i][0] + US_time_acq[i][1];
			US_distance[i] = US_time_acq[i][1]*(float)340/(float)(2*1000);					//Each timer tick should be one microsecond -> distance in mm 
			if(US_min_dist > US_distance[i])
				US_min_dist = US_distance[i];													//Gets minimum distance
			if(US_distance[i] < ULTRASONIC_T_ZERO && US_distance[i] != 0){
				Ultrasonic_flags |= ULTRASONIC_WARNING;		//Sets hit collision avoidance state
				//SEND WARNING (could be just the flag)
			}
			if(US_distance[i] < ULTRASONIC_T_MINUS && US_distance[i] != 0){
				Ultrasonic_flags |= ULTRASONIC_DEAD;				//Sets dead state
			}
		}
		US_RxStruct.Front = US_distance[0];			//Assigns the distance to the right fields of the Struct to be sent to PC
		US_RxStruct.Left = US_distance[1];			//We begin in the front and proceed counter-clockwise
		US_RxStruct.Back = US_distance[2];
		US_RxStruct.Right = US_distance[3];
		if(xQueueSend(UltrasonicToPC,&US_RxStruct,0) != pdTRUE)
			Ultrasonic_flags |= ULTRASONIC_FAILSEND;
		else
			Ultrasonic_flags &= ~ULTRASONIC_FAILSEND;
		Ultrasonic_flags &= ~(ULTRASONIC_CAPTURE_1 | ULTRASONIC_CAPTURE_2 | ULTRASONIC_CAPTURE_3 | ULTRASONIC_CAPTURE_4);
		if(US_min_dist > ULTRASONIC_T_ZERO && US_min_dist < ULTRASONIC_T_PLUS){
			Ultrasonic_flags &= ~ULTRASONIC_DEAD;
			Ultrasonic_flags |= ULTRASONIC_WARNING;
		}
		else if(US_min_dist > ULTRASONIC_T_PLUS){
			Ultrasonic_flags &= ~ULTRASONIC_DEAD;
			Ultrasonic_flags &= ~ULTRASONIC_WARNING;
		}
	}
}



void BatteryControlTaskFunction(void const * argument){
//	HAL_ADC_Start_IT(&hadc1);
//	while(1){
//		if( (SF & SFLAG_ADC) != 0){																		//If the values have already been updated in the cycle
//			osDelay(110);																										//Allocates free time for RTOS to go to get busy with other tasks
//		}
//		else{																															//In the case the ADC calculations data has not yet been updated (After TIM3 interruption)
//			BATT_Conversions();																							//Executes calculations and conversions to SOC
//			HighWaterMarks[3] = uxTaskGetStackHighWaterMark(NULL);
//			SF |= SFLAG_ADC;																						//Signals the update in Battery voltages calculations and measurements
//		}
//	}
}

void IndicatorsTaskFunction(void const * argument){
	glcd_draw_string_xy_P(0,0,"Hello!");
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	// - For TIMER 1 - //
	if(htim == &htim1){																			
		uint32_t time_aux;
		uint8_t i = 0;
		switch(htim->Channel){
			case HAL_TIM_ACTIVE_CHANNEL_1:
				i = 0;
				time_aux = htim->Instance->CCR1;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				i = 1;
				time_aux = htim->Instance->CCR2;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				i = 2;
				time_aux = htim->Instance->CCR3;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				i = 3;
				time_aux = htim->Instance->CCR4;
				break;
			default:
				break;
		}
		if(i > 3)																		//If not, then
			_Error_Handler(__FILE__, __LINE__);				//We have a pretty big problem
		xQueueSendFromISR(Ultrasonic_Queue[i],&time_aux,NULL);
		if((Ultrasonic_flags & ULTRASONIC_SGNEDGE) == 0)
			Ultrasonic_flags |= ULTRASONIC_SGNEDGE;
		else
			Ultrasonic_flags &= ~ULTRASONIC_SGNEDGE;
	}
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	ADC_read[0] += hadc->Instance->DR;
//	ADC_read[1] += hadc->Instance->JDR1;
//	ADC_read[2] += hadc->Instance->JDR2;
//	adc_sample_count++;
//	
//	if(adc_sample_count == 0){
//				strcpy(Error_Status,"Error: ADC sample count 0");
//				//Error_Handler();
//			}
//			else{
//				for(int i = 0; i<BATT_N_CELLS; i++){																					//Gets mean value of battery cell voltages in the period time of TIM3
//					ADC_voltages[i] = ((float)ADC_read[i]/(float)adc_sample_count)*v_step;
//					ADC_read[i] = 0;																														//Resets accumulation variables for preparing for next cycle 
//				}
//			}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			HAL_ADC_Start_IT(&hadc1);
		}
	}
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */
	
/* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
