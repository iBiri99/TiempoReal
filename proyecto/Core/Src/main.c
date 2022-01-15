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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERIAL_DELAY	500

#define ACC_X_ZERO	1.627
#define ACC_Y_ZERO	1.650
#define ACC_Z_ZERO	2.04

#define ACC_X_OFFSET -0.012217294
#define ACC_Y_OFFSET 0.026179917
#define ACC_Z_OFFSET 0

#define SENSITIVITY 0.3
#define PI			3.14159265359

#define RESOLUTION 4096.0

#define RED 0
#define GREEN 1
#define BLUE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for serialConfig */
osThreadId_t serialConfigHandle;
const osThreadAttr_t serialConfig_attributes = {
  .name = "serialConfig",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendSerial */
osThreadId_t sendSerialHandle;
const osThreadAttr_t sendSerial_attributes = {
  .name = "sendSerial",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getTemp */
osThreadId_t getTempHandle;
const osThreadAttr_t getTemp_attributes = {
  .name = "getTemp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for getLight */
osThreadId_t getLightHandle;
const osThreadAttr_t getLight_attributes = {
  .name = "getLight",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for getRoll */
osThreadId_t getRollHandle;
const osThreadAttr_t getRoll_attributes = {
  .name = "getRoll",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for getPitch */
osThreadId_t getPitchHandle;
const osThreadAttr_t getPitch_attributes = {
  .name = "getPitch",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for getYaw */
osThreadId_t getYawHandle;
const osThreadAttr_t getYaw_attributes = {
  .name = "getYaw",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for resetTask */
osThreadId_t resetTaskHandle;
const osThreadAttr_t resetTask_attributes = {
  .name = "resetTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tempQueue */
osMessageQueueId_t tempQueueHandle;
const osMessageQueueAttr_t tempQueue_attributes = {
  .name = "tempQueue"
};
/* Definitions for lightQueue */
osMessageQueueId_t lightQueueHandle;
const osMessageQueueAttr_t lightQueue_attributes = {
  .name = "lightQueue"
};
/* Definitions for rollQueue */
osMessageQueueId_t rollQueueHandle;
const osMessageQueueAttr_t rollQueue_attributes = {
  .name = "rollQueue"
};
/* Definitions for pitchQueue */
osMessageQueueId_t pitchQueueHandle;
const osMessageQueueAttr_t pitchQueue_attributes = {
  .name = "pitchQueue"
};
/* Definitions for yawQueue */
osMessageQueueId_t yawQueueHandle;
const osMessageQueueAttr_t yawQueue_attributes = {
  .name = "yawQueue"
};
/* Definitions for adc */
osMutexId_t adcHandle;
const osMutexAttr_t adc_attributes = {
  .name = "adc"
};
/* Definitions for configUnits */
osEventFlagsId_t configUnitsHandle;
const osEventFlagsAttr_t configUnits_attributes = {
  .name = "configUnits"
};
/* Definitions for conversion */
osEventFlagsId_t conversionHandle;
const osEventFlagsAttr_t conversion_attributes = {
  .name = "conversion"
};
/* Definitions for dataReady */
osEventFlagsId_t dataReadyHandle;
const osEventFlagsAttr_t dataReady_attributes = {
  .name = "dataReady"
};
/* USER CODE BEGIN PV */
union serial_t{

	uint8_t i[256];
	char	c[256];
}str;

typedef struct units_t{

	char temp;
	char tilt[4];
};

struct units_t units;

uint8_t size;

HAL_StatusTypeDef status;

typedef struct sensors_t{

	float temp;
	float light;
	float roll;
	float pitch;
	float yaw;
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void startSerialConfig(void *argument);
void startSendSerial(void *argument);
void startGetTemp(void *argument);
void startGetLight(void *argument);
void startGetRoll(void *argument);
void startGetPitch(void *argument);
void startGetYaw(void *argument);
void StartTask08(void *argument);

/* USER CODE BEGIN PFP */
float changeTempUnit (float temp, char unit);
float changeTiltUnit (float  tilt, char* unit);
float convertTemp (uint32_t raw);
uint8_t  buildMessage (uint8_t* string, struct sensors_t sensors);
void convertTilt (float* tilt,uint32_t raw, char axis);
void selTemp ();
void selLight ();
void selRoll ();
void selPitch ();
void selYaw ();
void changeLed (uint8_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void selTemp (){

	  ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void selLight (){

	  ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void selRoll (){

	  ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void selPitch (){

	  ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void selYaw (){

	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

uint8_t buildMessage (uint8_t* string, struct sensors_t sensors){

	uint8_t size;

	size = sprintf(str.c,"%.2f\t\t\t%.2f\t\t\t%.2f\t\t\t%.2f\t\t\t%.2f\n",sensors.temp,sensors.light,sensors.roll,sensors.pitch,sensors.yaw);
	return size;
}

float changeTiltUnit (float tilt, char* unit){//rad to deg

	if (strcmp(unit,"deg") == 0){

		tilt = tilt*180.0 / PI;
	}
	return tilt;
}
float changeTempUnit (float temp, char unit){//C to whatever

	switch (unit){

		case 'C':
			break;

		case 'K':
			temp += 273.15;
			break;

		case 'F':
			temp = temp*1.8 +32;
			break;

		case 'R':
			temp = temp*1.8 + 32 + 459.67;
			break;
	}

	return temp;
}

float convertTemp (uint32_t raw){

	float volt, temp;

	volt = (3.3 * raw) / RESOLUTION;
	temp = (volt - 0.5) * 100;

	return temp;
}

float convertLight (uint32_t raw){

	float light,volt,r;

	volt = (3.3 * raw) / RESOLUTION;
	r = (1000.0 * 3.3 - 1000.0 * volt) / volt;
	light = 500.0 / (r / 1000.0);

	return light;
}

void convertTilt (float* tilt,uint32_t raw, char axis){

	float volt,acc;

	volt = (3.3 * raw) / RESOLUTION;

	switch (axis){

	case 'X':
		acc = (volt - ACC_X_ZERO) / SENSITIVITY;
		*tilt = asin(acc) - ACC_X_OFFSET;//rad
		break;

	case 'Y':
		acc = (volt - ACC_Y_ZERO) / SENSITIVITY;
		*tilt = asin(acc) - ACC_Y_OFFSET;//rad
		break;

	case 'Z':
		acc = (volt - ACC_Z_ZERO) / SENSITIVITY;
		*tilt = asin(acc) - ACC_Z_OFFSET;//rad
		break;
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void changeLed(uint8_t state){

	if (state == BLUE){

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	}

	if (state == RED){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	}

	if (state == GREEN){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
	}
}
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of adc */
  adcHandle = osMutexNew(&adc_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of tempQueue */
  tempQueueHandle = osMessageQueueNew (128, sizeof(float), &tempQueue_attributes);

  /* creation of lightQueue */
  lightQueueHandle = osMessageQueueNew (128, sizeof(float), &lightQueue_attributes);

  /* creation of rollQueue */
  rollQueueHandle = osMessageQueueNew (128, sizeof(float), &rollQueue_attributes);

  /* creation of pitchQueue */
  pitchQueueHandle = osMessageQueueNew (128, sizeof(float), &pitchQueue_attributes);

  /* creation of yawQueue */
  yawQueueHandle = osMessageQueueNew (128, sizeof(float), &yawQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of serialConfig */
  serialConfigHandle = osThreadNew(startSerialConfig, NULL, &serialConfig_attributes);

  /* creation of sendSerial */
  sendSerialHandle = osThreadNew(startSendSerial, NULL, &sendSerial_attributes);

  /* creation of getTemp */
  getTempHandle = osThreadNew(startGetTemp, NULL, &getTemp_attributes);

  /* creation of getLight */
  getLightHandle = osThreadNew(startGetLight, NULL, &getLight_attributes);

  /* creation of getRoll */
  getRollHandle = osThreadNew(startGetRoll, NULL, &getRoll_attributes);

  /* creation of getPitch */
  getPitchHandle = osThreadNew(startGetPitch, NULL, &getPitch_attributes);

  /* creation of getYaw */
  getYawHandle = osThreadNew(startGetYaw, NULL, &getYaw_attributes);

  /* creation of resetTask */
  resetTaskHandle = osThreadNew(StartTask08, NULL, &resetTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of configUnits */
  configUnitsHandle = osEventFlagsNew(&configUnits_attributes);

  /* creation of conversion */
  conversionHandle = osEventFlagsNew(&conversion_attributes);

  /* creation of dataReady */
  dataReadyHandle = osEventFlagsNew(&dataReady_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_UART_Receive(&huart2, str.i, 7, HAL_MAX_DELAY);
	units.temp = str.c[6];

	strcpy(str.c,"");
	HAL_UART_Receive(&huart2, str.i, 9, HAL_MAX_DELAY);
	sprintf(units.tilt,"%c%c%c",str.i[6],str.i[7],str.i[8]);
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startSerialConfig */
/**
  * @brief  Function implementing the serialConfig thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startSerialConfig */
void startSerialConfig(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	vTaskSuspend( NULL );
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startSendSerial */
/**
* @brief Function implementing the sendSerial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSendSerial */
void startSendSerial(void *argument)
{
  /* USER CODE BEGIN startSendSerial */
	struct sensors_t sensors;
	uint8_t prio = 1;
  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(tempQueueHandle,&sensors.temp, &prio, osWaitForever);
	osMessageQueueGet(lightQueueHandle,&sensors.light, &prio, osWaitForever);
	osMessageQueueGet(rollQueueHandle,&sensors.roll, &prio, osWaitForever);
	osMessageQueueGet(pitchQueueHandle,&sensors.pitch, &prio, osWaitForever);
	osMessageQueueGet(yawQueueHandle,&sensors.yaw, &prio, osWaitForever);

	size = buildMessage(str.i, sensors);
	if(HAL_UART_Transmit(&huart2, str.i, size,10) == HAL_OK){changeLed(GREEN);}
	else{changeLed(RED);}
    osDelay(SERIAL_DELAY);
  }
  /* USER CODE END startSendSerial */
}

/* USER CODE BEGIN Header_startGetTemp */
/**
* @brief Function implementing the getTemp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGetTemp */
void startGetTemp(void *argument)
{
  /* USER CODE BEGIN startGetTemp */
	float temp;
	uint32_t raw_temp;
	HAL_StatusTypeDef status;

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(adcHandle, osWaitForever);

	  selTemp();
	  HAL_ADC_Start(&hadc1);
	  status = HAL_ADC_PollForConversion (& hadc1 , 1);
	  if (status == HAL_OK){

		  raw_temp = HAL_ADC_GetValue(&hadc1);
		  temp = convertTemp (raw_temp);									 //convert raw to float
		  temp = changeTempUnit (temp,units.temp);
		  osMessageQueuePut(tempQueueHandle,&temp,1,osWaitForever);  //put message
	  }
	  else{
		  while(1){

			  changeLed(BLUE);
			  HAL_Delay(500);
		  }
	  }
	  HAL_ADC_Stop(&hadc1);
	  osMutexRelease(adcHandle);
	  osDelay(SERIAL_DELAY);
  }
  /* USER CODE END startGetTemp */
}

/* USER CODE BEGIN Header_startGetLight */
/**
* @brief Function implementing the getLight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGetLight */
void startGetLight(void *argument)
{
  /* USER CODE BEGIN startGetLight */
	float light;
	uint32_t raw_light;
	HAL_StatusTypeDef status;

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(adcHandle, osWaitForever);

	  selLight();
	  HAL_ADC_Start(&hadc1);
	  status = HAL_ADC_PollForConversion (& hadc1 , 1);
	  if (status == HAL_OK){

		  raw_light = HAL_ADC_GetValue(&hadc1);
		  light = convertLight(raw_light);					   				 //convert raw to float
		  osMessageQueuePut(lightQueueHandle,&light,1,osWaitForever);  //put message
	  }
	  else{
		  while(1){

			  changeLed(BLUE);
			  HAL_Delay(500);
		  }
	  }
	  HAL_ADC_Stop(&hadc1);
	  osMutexRelease(adcHandle);
	  osDelay(SERIAL_DELAY);
  }
  /* USER CODE END startGetLight */
}

/* USER CODE BEGIN Header_startGetRoll */
/**
* @brief Function implementing the getRoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGetRoll */
void startGetRoll(void *argument)
{
  /* USER CODE BEGIN startGetRoll */
	float roll;
	uint32_t raw_roll;
	HAL_StatusTypeDef status;

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(adcHandle, osWaitForever);

	  selRoll();
	  HAL_ADC_Start(&hadc1);
	  status = HAL_ADC_PollForConversion (& hadc1 , 1);
	  if (status == HAL_OK){

		  raw_roll = HAL_ADC_GetValue(&hadc1);
		  convertTilt(&roll, raw_roll, 'X');		   				 //convert raw to float
		  roll = changeTiltUnit(roll,units.tilt);
		  osMessageQueuePut(rollQueueHandle,&roll,1,osWaitForever);  //put message
	  }
	  else{
		  while(1){

			  changeLed(BLUE);
			  HAL_Delay(500);
		  }
	  }
	  HAL_ADC_Stop(&hadc1);
	  osMutexRelease(adcHandle);

	  osDelay(SERIAL_DELAY);
  }
  /* USER CODE END startGetRoll */
}

/* USER CODE BEGIN Header_startGetPitch */
/**
* @brief Function implementing the getPitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGetPitch */
void startGetPitch(void *argument)
{
  /* USER CODE BEGIN startGetPitch */
	float pitch;
	uint32_t raw_pitch;
	HAL_StatusTypeDef status;

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(adcHandle, osWaitForever);

	  selPitch();
	  HAL_ADC_Start(&hadc1);
	  status = HAL_ADC_PollForConversion (& hadc1 , 1);
	  if (status == HAL_OK){

		  raw_pitch = HAL_ADC_GetValue(&hadc1);
		  convertTilt(&pitch, raw_pitch, 'Y');		   				 //convert raw to float
		  pitch = changeTiltUnit(pitch,units.tilt);
		  osMessageQueuePut(pitchQueueHandle,&pitch,1,osWaitForever);  //put message
	  }
	  else{
		  while(1){

			  changeLed(BLUE);
			  HAL_Delay(500);
		  }
	  }
	  HAL_ADC_Stop(&hadc1);
	  osMutexRelease(adcHandle);

	  osDelay(SERIAL_DELAY);
  }
  /* USER CODE END startGetPitch */
}

/* USER CODE BEGIN Header_startGetYaw */
/**
* @brief Function implementing the getYaw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGetYaw */
void startGetYaw(void *argument)
{
  /* USER CODE BEGIN startGetYaw */
	float yaw;
	uint32_t raw_yaw;
	HAL_StatusTypeDef status;

  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(adcHandle, osWaitForever);

	  selYaw();
	  HAL_ADC_Start(&hadc1);
	  status = HAL_ADC_PollForConversion (& hadc1 , 1);
	  if (status == HAL_OK){

		  raw_yaw = HAL_ADC_GetValue(&hadc1);
		  convertTilt(&yaw, raw_yaw, 'Z');		   				 //convert raw to float
		  yaw = changeTiltUnit(yaw,units.tilt);
		  osMessageQueuePut(yawQueueHandle,&yaw,1,osWaitForever);   //put message
	  }
	  else{
		  while(1){

			  changeLed(BLUE);
			  HAL_Delay(500);
		  }
	  }
	  HAL_ADC_Stop(&hadc1);
	  osMutexRelease(adcHandle);

	  osDelay(SERIAL_DELAY);
  }
  /* USER CODE END startGetYaw */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the resetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
  for(;;)
  {
	HAL_UART_Receive(&huart2, str.i, 1, HAL_MAX_DELAY);
	NVIC_SystemReset();
  }
  /* USER CODE END StartTask08 */
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
