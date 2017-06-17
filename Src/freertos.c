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
#include "iwdg.h"
#include "communication.h"
#include "global.h"
#include "stabilization.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId LedBlinkingHandle;
uint32_t LedBlinkingBuffer[ 64 ];
osStaticThreadDef_t LedBlinkingControlBlock;
osThreadId VmaDevCommunicationHandle;
uint32_t VmaDevCommunicationBuffer[ 64 ];
osStaticThreadDef_t VmaDevCommunicationControlBlock;
osThreadId SensorsCommunicationHandle;
uint32_t SensorsCommunicationBuffer[ 128 ];
osStaticThreadDef_t SensorsCommunicationControlBlock;
osThreadId StabilizationHandle;
uint32_t StabilizationBuffer[ 64 ];
osStaticThreadDef_t StabilizationControlBlock;
osThreadId DevCommunicationHandle;
uint32_t DevCommunicationBuffer[ 64 ];
osStaticThreadDef_t DevCommunicationControlBlock;

/* USER CODE BEGIN Variables */
#define SHORE_DELAY	   15

TimerHandle_t UARTTimer;
extern bool uart1PackageReceived;
extern uint8_t RxBuffer[1];
extern uint16_t numberRx;
extern uint16_t counterRx;

bool shoreCommunicationUpdated = false;

extern struct PIDRegulator rollPID;
extern struct PIDRegulator pitchPID;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void LedBlinkingTask(void const * argument);
void VmaDevCommunicationTask(void const * argument);
void SensorsCommunicationTask(void const * argument);
void StabilizationTask(void const * argument);
void StartDevCommunication(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void uartTimerCallback(xTimerHandle xTimer);
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  while(HAL_I2C_GetState(&hi2c1)!= HAL_I2C_STATE_READY) { }
	while(HAL_UART_GetState(&huart4)!= HAL_UART_STATE_READY) { }
	
  IMUReset();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	UARTTimer = xTimerCreate("timer", SHORE_DELAY/portTICK_RATE_MS, pdFALSE, 0, uartTimerCallback);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LedBlinking */
  osThreadStaticDef(LedBlinking, LedBlinkingTask, osPriorityNormal, 0, 64, LedBlinkingBuffer, &LedBlinkingControlBlock);
  LedBlinkingHandle = osThreadCreate(osThread(LedBlinking), NULL);

  /* definition and creation of VmaDevCommunication */
  osThreadStaticDef(VmaDevCommunication, VmaDevCommunicationTask, osPriorityAboveNormal, 0, 64, VmaDevCommunicationBuffer, &VmaDevCommunicationControlBlock);
  VmaDevCommunicationHandle = osThreadCreate(osThread(VmaDevCommunication), NULL);

  /* definition and creation of SensorsCommunication */
  osThreadStaticDef(SensorsCommunication, SensorsCommunicationTask, osPriorityNormal, 0, 128, SensorsCommunicationBuffer, &SensorsCommunicationControlBlock);
  SensorsCommunicationHandle = osThreadCreate(osThread(SensorsCommunication), NULL);

  /* definition and creation of Stabilization */
  osThreadStaticDef(Stabilization, StabilizationTask, osPriorityNormal, 0, 64, StabilizationBuffer, &StabilizationControlBlock);
  StabilizationHandle = osThreadCreate(osThread(Stabilization), NULL);

  /* definition and creation of DevCommunication */
  osThreadStaticDef(DevCommunication, StartDevCommunication, osPriorityAboveNormal, 0, 64, DevCommunicationBuffer, &DevCommunicationControlBlock);
  DevCommunicationHandle = osThreadCreate(osThread(DevCommunication), NULL);

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
	uint32_t sysTime = osKernelSysTick();
	
  /* Infinite loop */
  for(;;){
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
		osDelayUntil(&sysTime, 200);	
		HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		osDelayUntil(&sysTime, 200);	
		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		osDelayUntil(&sysTime, 200);		
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
		osDelayUntil(&sysTime, 200);	
		HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
  }
  /* USER CODE END LedBlinkingTask */
}

/* VmaDevCommunicationTask function */
void VmaDevCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN VmaDevCommunicationTask */
	uint32_t sysTime = osKernelSysTick();
	uint8_t VMATransaction = 0;

  /* Infinite loop */
  for(;;){
		if (shoreCommunicationUpdated){
			switch(VMATransaction){
				case HLB:
					VMARequestUpdate(&Q100, VMARequestBuf, HLB);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, HLB);
				break;
				
				case HLF:
					VMARequestUpdate(&Q100, VMARequestBuf, HLF);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, HLF);
				break;
				
				case HRB:
					VMARequestUpdate(&Q100, VMARequestBuf, HRB);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, HRB);
				break;
				
				case HRF:
					VMARequestUpdate(&Q100, VMARequestBuf, HRF);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, HRF);
				break;
				
				case VB:
					VMARequestUpdate(&Q100, VMARequestBuf, VB);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, VB);
				break;
				
				case VF:		
					VMARequestUpdate(&Q100, VMARequestBuf, VF);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, VF);
				break;
				
				case VL:
					VMARequestUpdate(&Q100, VMARequestBuf, VL);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, VL);
				break;
				
				case VR:
					VMARequestUpdate(&Q100, VMARequestBuf, VR);
					transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(VMA_UART, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMAResponseUpdate(&Q100, VMAResponseBuf, VR);
				break;
			}
			VMATransaction = (VMATransaction + 1) % VMA_DRIVER_NUMBER;
		}
		osDelayUntil(&sysTime, 10);
  }
  /* USER CODE END VmaDevCommunicationTask */
}

/* SensorsCommunicationTask function */
void SensorsCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN SensorsCommunicationTask */
	uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;){
		uint8_t ErrorCode = 0;
		IMUReceive(&Q100, IMUReceiveBuf, &ErrorCode);
		BTRequest(BTReceiveBuf);
		
    osDelayUntil(&sysTime, 100);
  }
  /* USER CODE END SensorsCommunicationTask */
}

/* StabilizationTask function */
void StabilizationTask(void const * argument)
{
  /* USER CODE BEGIN StabilizationTask */
	uint32_t sysTime = osKernelSysTick();
	stabilizationInit(&Q100);
	/* Infinite loop */
  for(;;){
		Q100.pitchStabilization.speedError = stabilizePitch(&Q100);
		Q100.rollStabilization.speedError = stabilizeRoll(&Q100);
    osDelayUntil(&sysTime, 100);
  }
  /* USER CODE END StabilizationTask */
}

/* StartDevCommunication function */
void StartDevCommunication(void const * argument)
{
  /* USER CODE BEGIN StartDevCommunication */
	uint32_t sysTime = osKernelSysTick();
	uint8_t DevTransaction = 0;
  /* Infinite loop */		
  for(;;){
		if (shoreCommunicationUpdated){
			switch(DevTransaction){
				case AGAR:
					DevRequestUpdate(&Q100, DevRequestBuf, AGAR);
					transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(DEV_UART, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DevResponseUpdate(&Q100, DevResponseBuf, AGAR);
				break;
				
				case GRAB:
					DevRequestUpdate(&Q100, DevRequestBuf, GRAB);
					transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(DEV_UART, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DevResponseUpdate(&Q100, DevResponseBuf, GRAB);
				break;
				
				case GRAB_ROTATION:
					DevRequestUpdate(&Q100, DevRequestBuf, GRAB_ROTATION);
					transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(DEV_UART, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DevResponseUpdate(&Q100, DevResponseBuf, GRAB_ROTATION);
				break;
				
				case TILT:
					DevRequestUpdate(&Q100, DevRequestBuf, TILT);
					transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					receivePackageDMA(DEV_UART, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DevResponseUpdate(&Q100, DevResponseBuf, TILT);
				break;
			}
			DevTransaction = (DevTransaction + 1) % DEV_DRIVER_NUMBER;
		}
		osDelayUntil(&sysTime, 20);
  }
  /* USER CODE END StartDevCommunication */
}

/* USER CODE BEGIN Application */
void uartTimerCallback(xTimerHandle xTimer)
{
		if (uart1PackageReceived){
			//HAL_IWDG_Refresh(&hiwdg);
			shoreCommunicationUpdated = true;
			uart1PackageReceived = false;
			
			switch(numberRx){
				case SHORE_REQUEST_LENGTH:
					ShoreRequest(&Q100, ShoreRequestBuf);
					break;
				case REQUEST_CONFIG_LENGTH:
					ShoreConfigRequest(&Q100, ShoreRequestConfigBuf);
					break;
			}		
			
			ShoreResponse(&Q100, ShoreResponseBuf);
			transmitPackageDMA(SHORE_UART, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
			HAL_HalfDuplex_EnableReceiver(&huart1);
			HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
		}
		else{
			shoreCommunicationUpdated = false;
			counterRx = 0;
			switch(numberRx){
				case SHORE_REQUEST_CODE:
					for (uint16_t i = 0; i < numberRx; ++i){
						ShoreRequestBuf[i] = 0x00;
					}
					break;
				case REQUEST_CONFIG_CODE:
					for (uint16_t i = 0; i < numberRx; ++i){
						ShoreRequestConfigBuf[i] = 0x00;
					}
					break;
			}
		}		
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
