/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"e_papier.h"
#include"GFX_BW.h"
#include "fonts/fonts.h"
#include "printf.h"
#include "spi.h"
#include "usart.h"
#include "i2c.h"
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
/* USER CODE BEGIN Variables */
uint16_t PMSData[3]={0,0,0};
uint8_t hc12Data[100];
float temp,p,h;
float tempAHT15,humAHT15;
void buffer_clear()
{
	for(int i=0;i<100;i++)
	{
		hc12Data[i]=0;
	}
}
uint8_t addr=(0x38<<1);
uint16_t MakePMSWord(uint8_t a,uint8_t b)
{
	uint16_t p=(a<<8);
	return p|b;
}
/* USER CODE END Variables */
/* Definitions for HeartBit */
osThreadId_t HeartBitHandle;
const osThreadAttr_t HeartBit_attributes = {
  .name = "HeartBit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TimeToEnd */
osThreadId_t TimeToEndHandle;
const osThreadAttr_t TimeToEnd_attributes = {
  .name = "TimeToEnd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TimeToSend */
osThreadId_t TimeToSendHandle;
const osThreadAttr_t TimeToSend_attributes = {
  .name = "TimeToSend",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EpapierTask */
osThreadId_t EpapierTaskHandle;
const osThreadAttr_t EpapierTask_attributes = {
  .name = "EpapierTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for Parse */
osThreadId_t ParseHandle;
const osThreadAttr_t Parse_attributes = {
  .name = "Parse",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AHT15Task */
osThreadId_t AHT15TaskHandle;
const osThreadAttr_t AHT15Task_attributes = {
  .name = "AHT15Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Time */
osMessageQueueId_t TimeHandle;
const osMessageQueueAttr_t Time_attributes = {
  .name = "Time"
};
/* Definitions for PMSData */
osMutexId_t PMSDataHandle;
const osMutexAttr_t PMSData_attributes = {
  .name = "PMSData"
};
/* Definitions for EpapierData */
osMutexId_t EpapierDataHandle;
const osMutexAttr_t EpapierData_attributes = {
  .name = "EpapierData"
};
/* Definitions for hc12Data */
osMutexId_t hc12DataHandle;
const osMutexAttr_t hc12Data_attributes = {
  .name = "hc12Data"
};
/* Definitions for AHT15Data */
osMutexId_t AHT15DataHandle;
const osMutexAttr_t AHT15Data_attributes = {
  .name = "AHT15Data"
};
/* Definitions for ParseSemaphore */
osSemaphoreId_t ParseSemaphoreHandle;
const osSemaphoreAttr_t ParseSemaphore_attributes = {
  .name = "ParseSemaphore"
};
/* Definitions for ProcesFlags */
osEventFlagsId_t ProcesFlagsHandle;
const osEventFlagsAttr_t ProcesFlags_attributes = {
  .name = "ProcesFlags"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHeartBit(void *argument);
void StartTimeToEnd(void *argument);
void StartTimeToSend(void *argument);
void StartEpapierTask(void *argument);
void StartParse(void *argument);
void StartAHT15Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of PMSData */
  PMSDataHandle = osMutexNew(&PMSData_attributes);

  /* creation of EpapierData */
  EpapierDataHandle = osMutexNew(&EpapierData_attributes);

  /* creation of hc12Data */
  hc12DataHandle = osMutexNew(&hc12Data_attributes);

  /* creation of AHT15Data */
  AHT15DataHandle = osMutexNew(&AHT15Data_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ParseSemaphore */
  ParseSemaphoreHandle = osSemaphoreNew(1, 1, &ParseSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Time */
  TimeHandle = osMessageQueueNew (16, sizeof(uint32_t), &Time_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartBit */
  HeartBitHandle = osThreadNew(StartHeartBit, NULL, &HeartBit_attributes);

  /* creation of TimeToEnd */
  TimeToEndHandle = osThreadNew(StartTimeToEnd, NULL, &TimeToEnd_attributes);

  /* creation of TimeToSend */
  TimeToSendHandle = osThreadNew(StartTimeToSend, NULL, &TimeToSend_attributes);

  /* creation of EpapierTask */
  EpapierTaskHandle = osThreadNew(StartEpapierTask, NULL, &EpapierTask_attributes);

  /* creation of Parse */
  ParseHandle = osThreadNew(StartParse, NULL, &Parse_attributes);

  /* creation of AHT15Task */
  AHT15TaskHandle = osThreadNew(StartAHT15Task, NULL, &AHT15Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of ProcesFlags */
  ProcesFlagsHandle = osEventFlagsNew(&ProcesFlags_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartBit */
/**
  * @brief  Function implementing the HeartBit thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartBit */
void StartHeartBit(void *argument)
{
  /* USER CODE BEGIN StartHeartBit */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 1);
	  osDelay(100);
	  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 0);
	  osDelay(100);
  }
  /* USER CODE END StartHeartBit */
}

/* USER CODE BEGIN Header_StartTimeToEnd */
/**
* @brief Function implementing the TimeToEnd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTimeToEnd */
void StartTimeToEnd(void *argument)
{
  /* USER CODE BEGIN StartTimeToEnd */
	char w[100];
	uint32_t t=0;
	uint32_t timeToEnd;
	uint32_t time;
  /* Infinite loop */
  for(;;)
  {
	  time=osKernelGetTickCount();
	  osEventFlagsWait(ProcesFlagsHandle, 0x00000001U, osFlagsWaitAny|osFlagsNoClear, osWaitForever);
	  if(osOK==osMessageQueueGet(TimeHandle, &t, 0, 0))
	  {
		  timeToEnd=180000-(osKernelGetTickCount()-t);
		  timeToEnd=timeToEnd/1000;
		  sprintf(w,"%d",timeToEnd);
		  osMutexAcquire(EpapierDataHandle, osWaitForever);
		  osMutexAcquire(hc12DataHandle, osWaitForever);
		  GFX_DrawString(0, 0, w, BLACK, 1);
		  sprintf(w,"PM1: %d PM2.5: %d PM10: %d temp: %0.2f press: %0.2f hum: %0.2f",PMSData[0],PMSData[1],PMSData[2],temp,p,h);
		  GFX_DrawString(0, 20, w, BLACK, 1);
		  osMutexAcquire(AHT15DataHandle, osWaitForever);
		  sprintf(w,"tempAHT: %0.2f humAHT: %0.2f",tempAHT15,humAHT15);
		  GFX_DrawString(0, 40, w, BLACK, 1);
		  osMutexRelease(AHT15DataHandle);
		  osEventFlagsSet(ProcesFlagsHandle, 0x00000002U);
		  osMutexRelease(hc12DataHandle);
		  osMutexRelease(EpapierDataHandle);
	  }
	  else
	  {
		  timeToEnd=180000-(osKernelGetTickCount()-t);
		  timeToEnd=timeToEnd/1000;
		  sprintf(w,"%d",timeToEnd);
		  osMutexAcquire(EpapierDataHandle, osWaitForever);
		  osMutexAcquire(hc12DataHandle, osWaitForever);
		  GFX_DrawString(0, 0, w, BLACK, 1);
		  sprintf(w,"PM1: %d PM2.5: %d PM10: %d temp: %0.2f press: %0.2f hum: %0.2f",PMSData[0],PMSData[1],PMSData[2],temp,p,h);
		  GFX_DrawString(0, 20, w, BLACK, 1);
		  osMutexAcquire(AHT15DataHandle, osWaitForever);
		  sprintf(w,"tempAHT: %0.2f humAHT: %0.2f",tempAHT15,humAHT15);
		  GFX_DrawString(0, 40, w, BLACK, 1);
		  osMutexRelease(AHT15DataHandle);
		  osEventFlagsSet(ProcesFlagsHandle, 0x00000002U);
		  osMutexRelease(hc12DataHandle);
		  osMutexRelease(EpapierDataHandle);
	  }
	  time+=10000;
	  osDelayUntil(time);
  }
  /* USER CODE END StartTimeToEnd */
}

/* USER CODE BEGIN Header_StartTimeToSend */
/**
* @brief Function implementing the TimeToSend thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTimeToSend */
void StartTimeToSend(void *argument)
{
  /* USER CODE BEGIN StartTimeToSend */
uint32_t time;
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(ProcesFlagsHandle, 0x00000004U, osFlagsWaitAny, osWaitForever);
	  time=osKernelGetTickCount();
	  osMessageQueuePut(TimeHandle, &time, 0, 0);
	  osEventFlagsSet(ProcesFlagsHandle, 0x00000001U);
	  osDelay(180000);
	  osMutexAcquire(EpapierDataHandle, osWaitForever);
	  GFX_DrawString(0, 0, "wyslano zapytanie", BLACK, 1);
	  osMutexRelease(EpapierDataHandle);
	  osEventFlagsClear(ProcesFlagsHandle, 0x00000001U);
	  osEventFlagsSet(ProcesFlagsHandle, 0x00000002U);
	  HAL_UART_Transmit(&huart1, (uint8_t*)"start", sizeof("start"), 100);
  }
  /* USER CODE END StartTimeToSend */
}

/* USER CODE BEGIN Header_StartEpapierTask */
/**
* @brief Function implementing the EpapierTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEpapierTask */
void StartEpapierTask(void *argument)
{
  /* USER CODE BEGIN StartEpapierTask */
	e_papier_init(&hspi2);
	GFX_SetFont(font_8x5);
	osMutexAcquire(AHT15DataHandle, osWaitForever);
	HAL_I2C_Master_Transmit(&hi2c1, addr, (uint8_t*)0xe1, 1, 1000);
	osDelay(100);
	osMutexRelease(AHT15DataHandle);
	osMutexAcquire(EpapierDataHandle, osWaitForever);
	e_papier_display();
	osMutexRelease(EpapierDataHandle);
	//osEventFlagsSet(ProcesFlagsHandle, 0x00000004U);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hc12Data, 100);
  /* Infinite loop */
  for(;;)
  {
	  //GFX_DrawString(0, 0, "abs", BLACK, 1);
	  osEventFlagsWait(ProcesFlagsHandle, 0x00000002U, osFlagsWaitAny, osWaitForever);
	  osMutexAcquire(EpapierDataHandle, osWaitForever);
	  e_papier_display();
	  e_papier_clear();
	  osMutexRelease(EpapierDataHandle);

  }
  /* USER CODE END StartEpapierTask */
}

/* USER CODE BEGIN Header_StartParse */
/**
* @brief Function implementing the Parse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParse */
void StartParse(void *argument)
{
  /* USER CODE BEGIN StartParse */
	char w[30];
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(ParseSemaphoreHandle, osWaitForever);
	  if(hc12Data[0]=='t')
	  {
		  osMutexAcquire(EpapierDataHandle, osWaitForever);
		  GFX_DrawString(0, 0, (char*)hc12Data, BLACK, 1);
		  sprintf(w,"PM1: %d PM2.5: %d PM10: %d temp: %0.2f press: %0.2f hum: %0.2f",PMSData[0],PMSData[1],PMSData[2],temp,p,h);
		  GFX_DrawString(0, 20, w, BLACK, 1);
		  osMutexAcquire(AHT15DataHandle, osWaitForever);
		  sprintf(w,"tempAHT: %0.2f humAHT: %0.2f",tempAHT15,humAHT15);
		  GFX_DrawString(0, 40, w, BLACK, 1);
		  osMutexRelease(AHT15DataHandle);
		  osMutexRelease(EpapierDataHandle);
		  osEventFlagsSet(ProcesFlagsHandle, 0x00000002U);
	  }
	  else
	  {
		  osMutexAcquire(hc12DataHandle, osWaitForever);
		  PMSData[0]=MakePMSWord(hc12Data[0], hc12Data[1]);
		  PMSData[1]=MakePMSWord(hc12Data[2], hc12Data[3]);
		  PMSData[2]=MakePMSWord(hc12Data[4], hc12Data[5]);
		  uint8_t *j;
		  j=&temp;
		  for(int i=0;i<4;i++)
		  {
			  *j=hc12Data[i+6];
			  j++;
		  }
		  j=&p;
		  for(int i=0;i<4;i++)
		  {
			  *j=hc12Data[i+10];
			  j++;
		  }
		  j=&h;
		  for(int i=0;i<4;i++)
		  {
			  *j=hc12Data[i+14];
			  j++;
		  }
		  osMutexRelease(hc12DataHandle);
		  osEventFlagsSet(ProcesFlagsHandle, 0x00000004U);
	  }
		buffer_clear();
  }
  /* USER CODE END StartParse */
}

/* USER CODE BEGIN Header_StartAHT15Task */
/**
* @brief Function implementing the AHT15Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAHT15Task */
void StartAHT15Task(void *argument)
{
  /* USER CODE BEGIN StartAHT15Task */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(AHT15DataHandle, osWaitForever);
	  uint8_t data[3]={0xac,0x33,0x00};
	  HAL_I2C_Master_Transmit(&hi2c1, addr, data, 3, 1000);
	  osDelay(300);
	  uint8_t dataRx[6];
	  HAL_I2C_Master_Receive(&hi2c1, addr, dataRx, 6, 1000);
	  uint32_t TempRaw=(((dataRx[3]&0x0f)<<16)|(dataRx[4]<<8)|dataRx[5]);
	  tempAHT15=((TempRaw*200.00)/1048576.00) - 50;
	  uint32_t HumRaw=(((dataRx[1]<<16)|(dataRx[2]<<8)|dataRx[3])>>4);
	  humAHT15=HumRaw*100.00/1048576.00;
	  osMutexRelease(AHT15DataHandle);
	  osDelay(20000);
  }
  /* USER CODE END StartAHT15Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hc12Data, 100);
	osSemaphoreRelease(ParseSemaphoreHandle);
}
/* USER CODE END Application */

