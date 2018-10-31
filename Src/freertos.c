/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "usart.h"
#include "i2c.h"
#include "timers.h"
#include "messages.h"
#include "robot.h"
#include "global.h"
#include "communication.h"
#include "stabilization.h"
#include "checksum.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId tLedBlinkingTaskHandle;
uint32_t tLedBlinkingTaskBuffer[ 128 ];
osStaticThreadDef_t tLedBlinkingTaskControlBlock;
osThreadId tVmaCommTaskHandle;
uint32_t tVmaCommTaskBuffer[ 128 ];
osStaticThreadDef_t tVmaCommTaskControlBlock;
osThreadId tImuCommTaskHandle;
uint32_t tImuCommTaskBuffer[ 128 ];
osStaticThreadDef_t tImuCommTaskControlBlock;
osThreadId tStabilizationTaskHandle;
uint32_t tStabilizationTaskBuffer[ 128 ];
osStaticThreadDef_t tStabilizationTaskControlBlock;
osThreadId tDevCommTaskHandle;
uint32_t tDevCommTaskBuffer[ 128 ];
osStaticThreadDef_t tDevCommTaskControlBlock;
osThreadId tSensCommTaskHandle;
uint32_t tSensCommTaskBuffer[ 128 ];
osStaticThreadDef_t tSensCommTaskControlBlock;
osThreadId tPcCommTaskHandle;
uint32_t tPcCommTaskBuffer[ 128 ];
osStaticThreadDef_t tPcCommTaskControlBlock;
osTimerId tUartTimerHandle;
osMutexId mutDataHandle;
osStaticMutexDef_t mutDataControlBlock;

/* USER CODE BEGIN Variables */
#define SHORE_DELAY	  45

TimerHandle_t UARTTimer;
extern bool uart1PackageReceived;
extern uint8_t RxBuffer[1];
extern uint16_t numberRx;
extern uint16_t counterRx;

bool shoreCommunicationUpdated = false;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void func_tLedBlinkingTask(void const * argument);
void func_tVmaCommTask(void const * argument);
void func_tImuCommTask(void const * argument);
void func_tStabilizationTask(void const * argument);
void func_tDevCommTask(void const * argument);
void func_tSensCommTask(void const * argument);
void func_tPcCommTask(void const * argument);
void func_tUartTimer(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void nullIntArray(uint8_t *array, uint8_t size);
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
    variableInit();
    stabilizationInit(&Q100);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of mutData */
  osMutexStaticDef(mutData, &mutDataControlBlock);
  mutDataHandle = osMutexCreate(osMutex(mutData));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of tUartTimer */
  osTimerDef(tUartTimer, func_tUartTimer);
  tUartTimerHandle = osTimerCreate(osTimer(tUartTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  UARTTimer = xTimerCreate("timer", SHORE_DELAY/portTICK_RATE_MS, pdFALSE, 0, (TimerCallbackFunction_t) func_tUartTimer);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of tLedBlinkingTask */
  osThreadStaticDef(tLedBlinkingTask, func_tLedBlinkingTask, osPriorityLow, 0, 128, tLedBlinkingTaskBuffer, &tLedBlinkingTaskControlBlock);
  tLedBlinkingTaskHandle = osThreadCreate(osThread(tLedBlinkingTask), NULL);

  /* definition and creation of tVmaCommTask */
  osThreadStaticDef(tVmaCommTask, func_tVmaCommTask, osPriorityBelowNormal, 0, 128, tVmaCommTaskBuffer, &tVmaCommTaskControlBlock);
  tVmaCommTaskHandle = osThreadCreate(osThread(tVmaCommTask), NULL);

  /* definition and creation of tImuCommTask */
  osThreadStaticDef(tImuCommTask, func_tImuCommTask, osPriorityBelowNormal, 0, 128, tImuCommTaskBuffer, &tImuCommTaskControlBlock);
  tImuCommTaskHandle = osThreadCreate(osThread(tImuCommTask), NULL);

  /* definition and creation of tStabilizationTask */
  osThreadStaticDef(tStabilizationTask, func_tStabilizationTask, osPriorityBelowNormal, 0, 128, tStabilizationTaskBuffer, &tStabilizationTaskControlBlock);
  tStabilizationTaskHandle = osThreadCreate(osThread(tStabilizationTask), NULL);

  /* definition and creation of tDevCommTask */
  osThreadStaticDef(tDevCommTask, func_tDevCommTask, osPriorityBelowNormal, 0, 128, tDevCommTaskBuffer, &tDevCommTaskControlBlock);
  tDevCommTaskHandle = osThreadCreate(osThread(tDevCommTask), NULL);

  /* definition and creation of tSensCommTask */
  osThreadStaticDef(tSensCommTask, func_tSensCommTask, osPriorityBelowNormal, 0, 128, tSensCommTaskBuffer, &tSensCommTaskControlBlock);
  tSensCommTaskHandle = osThreadCreate(osThread(tSensCommTask), NULL);

  /* definition and creation of tPcCommTask */
  osThreadStaticDef(tPcCommTask, func_tPcCommTask, osPriorityBelowNormal, 0, 128, tPcCommTaskBuffer, &tPcCommTaskControlBlock);
  tPcCommTaskHandle = osThreadCreate(osThread(tPcCommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* func_tLedBlinkingTask function */
void func_tLedBlinkingTask(void const * argument)
{

  /* USER CODE BEGIN func_tLedBlinkingTask */
    uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
        HAL_GPIO_TogglePin(DBG_LED_GPIO_Port, DBG_LED_Pin);
        osDelayUntil(&sysTime, DELAY_LED_TASK);
  }
  /* USER CODE END func_tLedBlinkingTask */
}

/* func_tVmaCommTask function */
void func_tVmaCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tVmaCommTask */
    uint32_t sysTime = osKernelSysTick();
    uint8_t VmaTransaction = 0;
  /* Infinite loop */
  for(;;)
  {
         if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_VMA_TASK) == pdTRUE) {
            VmaRequestUpdate(&Q100, VmaRequestBuf, VmaTransaction);
            xSemaphoreGive(mutDataHandle);
        }

        transmitPackageDMA(VMA_UART, VmaRequestBuf, VMA_REQUEST_LENGTH);
        receivePackageDMA(VMA_UART, VmaResponseBuf[VmaTransaction], VMA_RESPONSE_LENGTH);

        if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_VMA_TASK) == pdTRUE) {
        	VmaResponseUpdate(&Q100, VmaResponseBuf[VmaTransaction], VmaTransaction);
            xSemaphoreGive(mutDataHandle);
        }

        VmaTransaction = (VmaTransaction + 1) % VMA_DRIVER_NUMBER;
        osDelayUntil(&sysTime, DELAY_VMA_TASK);
  }
  /* USER CODE END func_tVmaCommTask */
}

/* func_tImuCommTask function */
void func_tImuCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tImuCommTask */
  uint32_t sysTime = osKernelSysTick();
  uint8_t ErrorCode = 0;
  /* Infinite loop */
  for(;;)
  {
	  	if(Q100.i_sensors.resetIMU) {
	  		transmitPackageDMA(IMU_UART, ImuResetRequestBuf, IMU_REQUEST_LENGTH);
	  		Q100.i_sensors.resetIMU = false;
	  	}
	  	else {
	  		transmitPackageDMA(IMU_UART, ImuRequestBuf, IMU_REQUEST_LENGTH);
	  		receivePackageDMA(IMU_UART, ImuResponseBuf, IMU_RESPONSE_LENGTH*IMU_CHECKSUMS);
	  		if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_IMU_TASK) == pdTRUE) {
	  			ImuReceive(&Q100, ImuResponseBuf, &ErrorCode);
	        	xSemaphoreGive(mutDataHandle);
	  		}
	  	}

        osDelayUntil(&sysTime, DELAY_IMU_TASK);
  }
  /* USER CODE END func_tImuCommTask */
}

/* func_tStabilizationTask function */
void func_tStabilizationTask(void const * argument)
{
  /* USER CODE BEGIN func_tStabilizationTask */
    uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  	if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_STAB_TASK) == pdTRUE) {
            if (Q100.pitchStabCons.enable) {
                stabilizePitch(&Q100);
            }
            else {
                Q100.pitchStabSt.speedError = 0;
            }

            if (Q100.rollStabCons.enable) {
                stabilizeRoll(&Q100);
            }
            else {
                Q100.rollStabSt.speedError = 0;
            }

            if (Q100.yawStabCons.enable) {
                stabilizeYaw(&Q100);
            }
            else {
                Q100.yawStabSt.speedError = 0;
            }

            xSemaphoreGive(mutDataHandle);
        }

        osDelayUntil(&sysTime, DELAY_STAB_TASK);
  }
  /* USER CODE END func_tStabilizationTask */
}

/* func_tDevCommTask function */
void func_tDevCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tDevCommTask */
    uint32_t sysTime = osKernelSysTick();
    uint8_t DevTransaction = 0;
  /* Infinite loop */
  for(;;)
  {
        if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_DEV_TASK) == pdTRUE) {
            DevRequestUpdate(&Q100, DevRequestBuf, DevTransaction);
            xSemaphoreGive(mutDataHandle);
        }

        transmitPackageDMA(DEV_UART, DevRequestBuf, DEV_REQUEST_LENGTH);
        receivePackageDMA(DEV_UART, DevResponseBuf[DevTransaction], DEV_RESPONSE_LENGTH);

        if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_DEV_TASK) == pdTRUE) {
            DevResponseUpdate(&Q100, DevResponseBuf[DevTransaction], DevTransaction);
            xSemaphoreGive(mutDataHandle);
        }

        DevTransaction = (DevTransaction + 1) % DEV_DRIVER_NUMBER;
        osDelayUntil(&sysTime, DELAY_DEV_TASK);
  }
  /* USER CODE END func_tDevCommTask */
}

/* func_tSensCommTask function */
void func_tSensCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tSensCommTask */
	uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  // TODO infinite loop starts here
	  /*
	  receiveI2cPackageDMA (DEV_I2C, SENSORS_PRESSURE_ADDR, SensorsResponseBuf[0], SENSORS_PACKAGE_SIZE);
	  if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_SENSOR_TASK) == pdTRUE) {
		  SensorsRequestUpdate(&Q100, DevRequestBuf, DEV_I2C);
		  xSemaphoreGive(mutDataHandle);
	  }
	  */
	  osDelayUntil(&sysTime, DELAY_SENSOR_TASK);
  }
  /* USER CODE END func_tSensCommTask */
}

/* func_tPcCommTask function */
void func_tPcCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tPcCommTask */
	uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  osDelayUntil(&sysTime, DELAY_PC_TASK);
  }
  /* USER CODE END func_tPcCommTask */
}

/* func_tUartTimer function */
void func_tUartTimer(void const * argument)
{
  /* USER CODE BEGIN func_tUartTimer */
	if (uart1PackageReceived) {
		shoreCommunicationUpdated = true;
		uart1PackageReceived = false;

		if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_TIMER_TASK) == pdTRUE) {
			if(numberRx == SHORE_REQUEST_LENGTH) {
				ShoreRequest(&Q100, ShoreRequestBuf);
			}
			else if(numberRx == REQUEST_CONFIG_LENGTH) {
				ShoreConfigRequest(&Q100, ShoreRequestConfigBuf);
			}

			ShoreResponse(&Q100, ShoreResponseBuf);
			xSemaphoreGive(mutDataHandle);
		}
		transmitPackageDMA(SHORE_UART, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
		HAL_HalfDuplex_EnableReceiver(&huart1);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
	}
	else {
		shoreCommunicationUpdated = false;
		uart1PackageReceived = false;
		counterRx = 0;

		if(numberRx == SHORE_REQUEST_CODE) {
			nullIntArray(ShoreRequestBuf, numberRx);
		}
		else if(numberRx == REQUEST_CONFIG_CODE) {
			nullIntArray(ShoreRequestConfigBuf, numberRx);
		}
	}
  /* USER CODE END func_tUartTimer */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
