/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(uint16_t SPS);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	int i = 0;
	for (i = 0; i <len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

uint16_t adcValue;
uint8_t Rx_data[3];
int sum=0;
int count = 0;


char str1[3];
char str2[3];


/***********Task handler***************/
xTaskHandle UART_Queue_Handler;
xTaskHandle Data_Processing_Handler;


/************Queue handler ******************/
xQueueHandle SimpleQueue;

xQueueHandle SimpleQueue_02;

xQueueHandle St_Queue_Handler;

/**************** STRUCTURE DEFINITION *****************/

typedef struct {
	char *str;
	uint16_t sampling_rate;
} my_struct;

my_struct *ptrtostruct;

/***************Task function****************/
void UART_Queue_Task (void* argument);
void Data_Processing_Task (void* argument);
void subString(uint8_t input[], int pos, int length, uint8_t sub[]);
void sendToQueue(void);

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
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init(1000);
  /* USER CODE BEGIN 2 */



  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_IT(&hadc1);
  HAL_UART_Receive_IT(&huart2, &Rx_data, 9);

  /************************* Create Integer Queue ****************************/
  SimpleQueue = xQueueCreate(5, sizeof (int));
  if (SimpleQueue == 0)  // Queue not created
  {
//	  char *str = "Unable to create Integer Queue\n\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	  printf("Unable to create Integer Queue\n\n");
  }
  else
  {
//	  char *str = "Integer Queue Created successfully\n\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	  printf("Integer Queue Created successfully\n\n");
  }


  /************************* Create Integer Queue_02 ****************************/
  SimpleQueue_02 = xQueueCreate(5, sizeof (int));
  if (SimpleQueue_02 == 0)  // Queue not created
  {
//	  char *str = "Unable to create Integer Queue\n\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	  printf("Unable to create Integer Queue\n\n");
  }
  else
  {
//	  char *str = "Integer Queue Created successfully\n\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	  printf("Integer Queue Created successfully\n\n");
  }


  /************************** create ST QUEUE **********************************/
  St_Queue_Handler = xQueueCreate(1, sizeof (my_struct));

  if (St_Queue_Handler == 0) // if there is some error while creating queue
	{
	  char *str = "Unable to create STRUCTURE Queue\n\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	}
  else
	{
	  char *str = "STRUCTURE Queue Created successfully\n\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	}

  /****************************** Task related******************************/

  xTaskCreate(Data_Processing_Task, "DSP", 128, NULL, 3, &Data_Processing_Handler);
  xTaskCreate(UART_Queue_Task, "UART_Queue",  128, NULL, 2, &UART_Queue_Handler);



  vTaskStartScheduler();
  /* USER CODE END 2 */


  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(uint16_t SPS)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = SPS; //SPS = 100 --> 100ms,
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  HAL_UART_Receive_IT(&huart2, &Rx_data, 3); //restart the interrupt reception mode
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UART_Queue_Task (void* argument)
{
	my_struct *Rptrtostruct;
	uint32_t TickDelay = pdMS_TO_TICKS(200); // check this delay
	char *ptr;
	while (1)
	{
		if (xQueueReceive(SimpleQueue_02, &Rx_data, portMAX_DELAY) == pdPASS)
		{
//			sprintf(str, "Successfully sent number %d to the queue\n\n", adcValue);
//			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
			MX_TIM2_Init(atoi(Rx_data));
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			printf("Enter UART_queue\n\n");
		}

		vPortFree(Rptrtostruct);  // free the structure memory
		vTaskDelay(TickDelay);
	}
}


/*
 * ADC Conversion to read temperature sensor
 * Temperature (in °C) = ((Vsense – V25) / Avg_Slope) + 25
 * Vense = Voltage Reading From Temperature Sensor
 * V25 = Voltage at 25°C, for STM32F407 = 0.76V
 * Avg_Slope = 2.5mV/°C
 */

void Data_Processing_Task (void* argument)
{
	int received = 0;
	uint32_t TickDelay = pdMS_TO_TICKS(100);
	while (1)
	{
//		char str[100];
		if (xQueueReceive(SimpleQueue, &received, portMAX_DELAY) != pdTRUE)
		{
//			HAL_UART_Transmit(&huart2, (uint8_t *)"Error in Receiving from Queue\n\n", 31, 1000);
			printf("Error in Receiving from Queue\n\n");
		}
		else
		{
			sum = sum + received;
			count++;
		}
		if (count == 5)
		{
			sum = sum/5;
			sum *= 3300;
			sum /= 0xfff; //Reading in mV (adc * 3300/4096)
			sum /= 1000.0; //Reading in Volts
			sum -= 0.760; // Subtract the reference voltage at 25°C
			sum /= .0025; // Divide by slope 2.5mV
			sum += 25.0; // Add the 25°C
			printf("Temperature value: %d\n\n", sum);
			count = 0;
			sum = 0;
		}
//		sprintf(str, "ADC value: %d \n",sum);
//		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

		vTaskDelay(TickDelay);
	}
}


void subString(uint8_t input[], int pos, int length, uint8_t sub[])
{
	int i = 0;
	while (i < length) {
	      sub[i] = input[pos + i - 1];
	      i++;
	   }
	   sub[i] = '\0';

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  adcValue = HAL_ADC_GetValue(&hadc1);

  if (xQueueSendToFrontFromISR(SimpleQueue, &adcValue, &xHigherPriorityTaskWoken) == pdPASS)
  {
//			sprintf(str, "Successfully sent number %d to the queue\n\n", adcValue);
//			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
	printf("Successfully sent number %d to the queue\n\n", adcValue);
  }
// Did sending to the queue unblock a higher priority task?
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}

void sendToQueue(void)
{
	subString(Rx_data, 2, 3, str1); // extract str value
	subString(Rx_data, 7, 3, str2); // extract sampling_rate

	/*****************Extract character received from Rx_data*******************/
	printf("Str value: %s and sampling rate: %d \n\n", str1, atoi(str2));

	/*****************ALOOCATE MEMORY TO THE PTR *******************************/
	ptrtostruct = pvPortMalloc(10*sizeof (my_struct));

	/********** LOAD THE DATA ***********/
	ptrtostruct->str = str1;
	ptrtostruct->sampling_rate = atoi(str2);



}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	 /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
	 it will get set to pdTRUE inside the interrupt safe API function if a
	 context switch is required. */

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//printf("QUEUE -- Str value: %s and sampling rate: %d \n\n", ptrtostruct->str, ptrtostruct->sampling_rate);

	if (xQueueSendFromISR(SimpleQueue_02, &Rx_data, &xHigherPriorityTaskWoken) == pdPASS)
	{
		//HAL_UART_Transmit(huart, (uint8_t *)"\n\nSent from ISR\n\n", 17, 500);
		printf("Sent from IRS of UART \n\n");
	}


	/* Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
	 xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
	 then calling portEND_SWITCHING_ISR() will request a context switch. If
	 xHigherPriorityTaskWoken is still pdFALSE then calling
	 portEND_SWITCHING_ISR() will have no effect */

	HAL_UART_Receive_IT(&huart2, &Rx_data, 3); //restart the interrupt reception mode

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END 4 */


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
