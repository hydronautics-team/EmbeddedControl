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
#include "usart.h"
#include "timers.h"
#include "u_messages.h"
#include "u_robot.h"
#include "u_global.h"
#include "u_communication.h"
#include "u_stabilization.h"

// eto kostil, potomu chto inache sizeof(xData) ne pashet((
enum eBufTypes {
	DTYPE_VMA=1,
	DTYPE_DEV,
	DTYPE_IMU,
	DTYPE_SHORE,
	DTYPE_STAB
};

typedef struct {
	uint8_t ucBufType;
	uint8_t ucBufNumber;
	uint8_t *ucBufInt;
	float		*ucBufPitch;
	float		*ucBufRoll;
} xData;
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
osThreadId tDataPackTaskHandle;
uint32_t tDataPackTaskBuffer[ 128 ];
osStaticThreadDef_t tDataPackTaskControlBlock;
osMessageQId qDataPackQueueHandle;
osTimerId tUartTimerHandle;

uint16_t taskCalls;

/* USER CODE BEGIN Variables */
TickType_t xLastWakeTime;

//GOVNO
/*
osMessageQId pitchErrorQueueHandle;
uint8_t pitchErrorQueueBuffer[ 2 * sizeof( int16_t ) ];
osStaticMessageQDef_t pitchErrorQueueControlBlock;

osMessageQId rollErrorQueueHandle;
uint8_t rollErrorQueueBuffer[ 2 * sizeof( int16_t ) ];
osStaticMessageQDef_t rollErrorQueueControlBlock;
*/

#define SHORE_DELAY	  45

TimerHandle_t UARTTimer;
extern bool uart1PackageReceived;
extern uint8_t RxBuffer[1];
extern uint16_t numberRx;
extern uint16_t counterRx;

bool shoreCommunicationUpdated = false;

extern struct PIDRegulator rollPID;
extern struct PIDRegulator pitchPID;
//GOVNO END
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void func_tLedBlinkingTask(void const * argument);
void func_tVmaCommTask(void const * argument);
void func_tImuCommTask(void const * argument);
void func_tStabilizationTask(void const * argument);
void func_tDevCommTask(void const * argument);
void func_tDataPackTask(void const * argument);
void func_tUartTimer(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void qDataSend(uint8_t type, uint8_t number, uint8_t *intlink,  float pidRoll, float pidPitch);
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
	HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);		
	taskCalls = 0;
  /* USER CODE END Init */

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
  UARTTimer = xTimerCreate("timer", SHORE_DELAY/portTICK_RATE_MS, pdFALSE, 0, func_tUartTimer);
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

  /* definition and creation of tDataPackTask */
  osThreadStaticDef(tDataPackTask, func_tDataPackTask, osPriorityNormal, 0, 128, tDataPackTaskBuffer, &tDataPackTaskControlBlock);
  tDataPackTaskHandle = osThreadCreate(osThread(tDataPackTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of qDataPackQueue */
  osMessageQDef(qDataPackQueue, 16, xData);
  qDataPackQueueHandle = osMessageCreate(osMessageQ(qDataPackQueue), NULL);

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
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);	
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);	
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);		
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);	
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//vTaskDelayUntil(&xLastWakeTime, 200);
		osDelayUntil(&sysTime, 200);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
  }
  /* USER CODE END func_tLedBlinkingTask */
}

/* func_tVmaCommTask function */
void func_tVmaCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tVmaCommTask */
	uint32_t sysTime = osKernelSysTick();
	uint8_t VMATransaction = 0;
  /* Infinite loop */
  for(;;)
  {
		VMARequestUpdate(&Q100, VMARequestBuf, VMATransaction);
		transmitPackageDMA(VMA_UART, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(VMA_UART, VMAResponseBuf[VMATransaction], VMA_DEV_RESPONSE_LENGTH);
		//VMAResponseUpdate(&Q100, VMAResponseBuf[VMATransaction], VMATransaction);
	
		qDataSend(DTYPE_VMA, VMATransaction, &VMAResponseBuf[VMATransaction][0], 0, 0);
		
		VMATransaction = (VMATransaction + 1) % VMA_DRIVER_NUMBER;
	
		//vTaskDelayUntil(&xLastWakeTime, 10);
		osDelayUntil(&sysTime, 10);
  }
  /* USER CODE END func_tVmaCommTask */
}

/* func_tImuCommTask function */
void func_tImuCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tImuCommTask */
	uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
		transmitPackageDMA(IMU_UART, IMURequestBuf, IMU_REQUEST_LENGTH);
		receivePackageDMA(IMU_UART, IMUResponseBuf, IMU_RESPONSE_LENGTH*IMU_CHECKSUMS);
		
		qDataSend(DTYPE_IMU, VMA_DEV_NULL, &IMUResponseBuf[0], 0, 0);
    
		//uint8_t ErrorCode = 0;
		//IMUReceive(&Q100, IMUResponseBuf, &ErrorCode);
		//BTRequest(BTReceiveBuf);
		
    //vTaskDelayUntil(&xLastWakeTime, 10);
		osDelayUntil(&sysTime, 10);
  }
  /* USER CODE END func_tImuCommTask */
}

/* func_tStabilizationTask function */
void func_tStabilizationTask(void const * argument)
{
  /* USER CODE BEGIN func_tStabilizationTask */
	uint32_t sysTime = osKernelSysTick();
	//uint8_t errtype=0;
	float rollError, pitchError;
  /* Infinite loop */
  for(;;)
  {
    if (Q100.pitchStabilization.enable){
			pitchError = stabilizePitch(&Q100);
			//xQueueSend( pitchErrorQueueHandle, ( void *) &Q100.pitchStabilization.speedError, 5);
		}
		else {
			//Q100.pitchStabilization.speedError = 0;
			pitchError = 0;
		}
		
		if (Q100.rollStabilization.enable){
			rollError = stabilizeRoll(&Q100);
			//xQueueSend( rollErrorQueueHandle, ( void *) &Q100.rollStabilization.speedError, 50);
		}
		else {
			//Q100.rollStabilization.speedError = 0;
			rollError = 0;
		}
		
    //vTaskDelayUntil(&xLastWakeTime, 50);
		osDelayUntil(&sysTime, 10);
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
		DevRequestUpdate(&Q100, DevRequestBuf, DevTransaction);
		transmitPackageDMA(DEV_UART, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
		receivePackageDMA(DEV_UART, DevResponseBuf[DevTransaction], VMA_DEV_RESPONSE_LENGTH);
		//DevResponseUpdate(&Q100, DevResponseBuf[DevTransaction], DevTransaction);
		
		qDataSend(DTYPE_DEV, DevTransaction, &DevResponseBuf[DevTransaction][0], 0, 0);
		
		DevTransaction = (DevTransaction + 1) % DEV_DRIVER_NUMBER;
		
    //vTaskDelayUntil(&xLastWakeTime, 20);
		osDelayUntil(&sysTime, 10);
  }
  /* USER CODE END func_tDevCommTask */
}

/* func_tDataPackTask function */
void func_tDataPackTask(void const * argument)
{
  /* USER CODE BEGIN func_tDataPackTask */
	uint32_t sysTime = osKernelSysTick();
	xData qDataStruct;
  /* Infinite loop */
  for(;;)
  {
	  //taskENTER_CRITICAL();
		/*if(uxQueueMessagesWaiting(qDataPackQueueHandle) == 0) {
			osDelayUntil(&sysTime, 10);
		}*/
		++taskCalls;
		xQueueReceive(qDataPackQueueHandle,&qDataStruct,(TickType_t) 20);
    switch(qDataStruct.ucBufType) {
			case DTYPE_VMA:
				VMAResponseUpdate(&Q100, &qDataStruct.ucBufInt[qDataStruct.ucBufNumber], qDataStruct.ucBufNumber);
			break;
			
			case DTYPE_DEV:
				DevResponseUpdate(&Q100, &qDataStruct.ucBufInt[qDataStruct.ucBufNumber], qDataStruct.ucBufNumber);
			break;
			
			case DTYPE_IMU:
				IMUReceive(&Q100, qDataStruct.ucBufInt, 0);
			break;
			
			case DTYPE_SHORE:
				ShoreRequest(&Q100, ShoreRequestBuf, 0, 0);
				ShoreResponse(&Q100, ShoreResponseBuf);
				transmitPackageDMA(SHORE_UART, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
				HAL_HalfDuplex_EnableReceiver(&huart1);
				HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);
			break;
			
			case DTYPE_STAB:
				//Q100.rollStabilization.speedError = qDataStruct.ucBufRoll;
				//Q100.rollStabilization.speedError = qDataStruct.ucBufPitch;
			break;
		}
		//taskEXIT_CRITICAL();
		//vTaskDelayUntil(&xLastWakeTime, 20);
  }
  /* USER CODE END func_tDataPackTask */
}

/* func_tUartTimer function */
void func_tUartTimer(void const * argument)
{
  /* USER CODE BEGIN func_tUartTimer */
  if (uart1PackageReceived){
			//HAL_IWDG_Refresh(&hiwdg);
			shoreCommunicationUpdated = true;
			uart1PackageReceived = false;
			
			int16_t pitchError = 0, rollError = 0;
			switch(numberRx){
				case SHORE_REQUEST_LENGTH:
//					xQueueReceive( pitchErrorQueueHandle, &( pitchError ), 1 );
//					xQueueReceive( rollErrorQueueHandle, &( rollError ), 1 );
					ShoreRequest(&Q100, ShoreRequestBuf, &pitchError, &rollError);
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
  /* USER CODE END func_tUartTimer */
}

/* USER CODE BEGIN Application */
void qDataSend(uint8_t type, uint8_t number, uint8_t *intlink, float pidRoll, float pidPitch)
{
	xData qDataStruct;
	qDataStruct.ucBufType = type;
	qDataStruct.ucBufNumber = number;
	qDataStruct.ucBufInt = intlink;
	qDataStruct.ucBufRoll = &pidRoll;
	qDataStruct.ucBufPitch = &pidPitch;
	xQueueSend(qDataPackQueueHandle, &qDataStruct, (TickType_t) 20);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
