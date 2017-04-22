/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
#include "usart.h"
#include "communication.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId LedBlinkingHandle;
osThreadId ShoreCommunicationHandle;
osThreadId VmaDevCommunicationHandle;
osThreadId SensorsCommunicationHandle;
osThreadId StabilizationHandle;

/* USER CODE BEGIN Variables */
bool shore_RX_enable;
bool shore_TX_enable;
bool VMA_RX_enable;
bool VMA_TX_enable;
bool DEV_RX_enable;
bool DEV_TX_enable;

bool VMA_RX_enable;
bool VMA_TX_enable;
bool DEV_RX_enable;
bool DEV_TX_enable;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void LedBlinkingTask(void const * argument);
void ShoreCommunicationTask(void const * argument);
void VmaDevCommunicationTask(void const * argument);
void SensorsCommunicationTask(void const * argument);
void StabilizationTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LedBlinking */
  osThreadDef(LedBlinking, LedBlinkingTask, osPriorityNormal, 0, 64);
  LedBlinkingHandle = osThreadCreate(osThread(LedBlinking), NULL);

  /* definition and creation of ShoreCommunication */
  osThreadDef(ShoreCommunication, ShoreCommunicationTask, osPriorityNormal, 0, 64);
  ShoreCommunicationHandle = osThreadCreate(osThread(ShoreCommunication), NULL);

  /* definition and creation of VmaDevCommunication */
  osThreadDef(VmaDevCommunication, VmaDevCommunicationTask, osPriorityNormal, 0, 64);
  VmaDevCommunicationHandle = osThreadCreate(osThread(VmaDevCommunication), NULL);

  /* definition and creation of SensorsCommunication */
  osThreadDef(SensorsCommunication, SensorsCommunicationTask, osPriorityNormal, 0, 64);
  SensorsCommunicationHandle = osThreadCreate(osThread(SensorsCommunication), NULL);

  /* definition and creation of Stabilization */
  osThreadDef(Stabilization, StabilizationTask, osPriorityIdle, 0, 64);
  StabilizationHandle = osThreadCreate(osThread(Stabilization), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* LedBlinkingTask function */
void LedBlinkingTask(void const * argument)
{

  /* USER CODE BEGIN LedBlinkingTask */
  /* Infinite loop */
  for(;;){
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		osDelay(250);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		osDelay(250);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
		osDelay(250);		
		HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		osDelay(250);		
		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		osDelay(250);		
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
		osDelay(250);		
		HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		osDelay(250);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		osDelay(250);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
  }
  /* USER CODE END LedBlinkingTask */
}

/* ShoreCommunicationTask function */
void ShoreCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN ShoreCommunicationTask */
	uint8_t packageType;
  /* Infinite loop */
  for(;;){
		receiveByte(SHORE_UART, &packageType);
		switch(packageType){
			case SHORE_REQUEST_CODE:
				receivePackageDMA(SHORE_UART, ShoreRequestBuf, SHORE_REQUEST_LENGTH);
				ShoreRequest(&Q100, ShoreRequestBuf);
				ShoreResponse(&Q100, ShoreResponseBuf);
				transmitPackageDMA(SHORE_UART, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
				break;
			case REQUEST_CONFIG_CODE:
				receivePackageDMA(SHORE_UART, ShoreRequestConfigBuf, REQUEST_CONFIG_LENGTH);
				ShoreConfigRequest(&Q100, ShoreRequestConfigBuf);
				ShoreResponse(&Q100, ShoreResponseBuf);
				transmitPackageDMA(SHORE_UART, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);			
				break;
		}
		osDelay(15);
	}
  /* USER CODE END ShoreCommunicationTask */
}

/* VmaDevCommunicationTask function */
void VmaDevCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN VmaDevCommunicationTask */

  /* Infinite loop */
  for(;;){
		VMARequestUpdate(&Q100, VMARequestBuf, HLB);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, HLB);
		
		VMARequestUpdate(&Q100, VMARequestBuf, HLF);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, HLF);
		
		VMARequestUpdate(&Q100, VMARequestBuf, HRB);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, HRB);
		
		VMARequestUpdate(&Q100, VMARequestBuf, HRF);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, HRF);
		
		VMARequestUpdate(&Q100, VMARequestBuf, VB);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, VB);
		
		VMARequestUpdate(&Q100, VMARequestBuf, VF);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, VF);
		
		VMARequestUpdate(&Q100, VMARequestBuf, VL);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, VL);
		
		VMARequestUpdate(&Q100, VMARequestBuf, VR);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
		VMAResponseUpdate(&Q100, VMAResponseBuf, VR);
	
		DevRequestUpdate(&Q100, DevRequestBuf, AGAR);
		transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_RESPONSE_LENGTH);
		DevResponseUpdate(&Q100, DevRequestBuf, AGAR);

		DevRequestUpdate(&Q100, DevRequestBuf, GRAB);
		transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_RESPONSE_LENGTH);
		DevResponseUpdate(&Q100, DevRequestBuf, GRAB);
		
		DevRequestUpdate(&Q100, DevRequestBuf, GRABROTATION);
		transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_RESPONSE_LENGTH);
		DevResponseUpdate(&Q100, DevRequestBuf, GRABROTATION);

		DevRequestUpdate(&Q100, DevRequestBuf, TILT);
		transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_RESPONSE_LENGTH);
		DevResponseUpdate(&Q100, DevRequestBuf, TILT);
		
		osDelay(10);
  }
  /* USER CODE END VmaDevCommunicationTask */
}

/* SensorsCommunicationTask function */
void SensorsCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN SensorsCommunicationTask */
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SensorsCommunicationTask */
}

/* StabilizationTask function */
void StabilizationTask(void const * argument)
{
  /* USER CODE BEGIN StabilizationTask */
  /* Infinite loop */
  for(;;){
    osDelay(1);
  }
  /* USER CODE END StabilizationTask */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
