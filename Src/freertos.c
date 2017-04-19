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
  * 1. Redistribution of source code must retain the above copyright notq100, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notq100,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devq100s manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this lq100nse is void and will automatically terminate your rights under 
  *    this lq100nse. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVq100S; LOSS OF USE, DATA, 
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
#include "messages.h"
#include "checksum.h"
#include "robot.h"
#include "stdlib.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId LedBlinkingHandle;
osThreadId ShoreCommunicationHandle;
osThreadId VmaDevCommunicationHandle;
osThreadId SensorsCommunicationHandle;
osThreadId StabilizationHandle;

/* USER CODE BEGIN Variables */
extern struct Robot Q100;

extern bool shore_RX_enable;
extern bool shore_TX_enable;
extern bool VMA_RX_enable;
extern bool VMA_TX_enable;
extern bool DEV_RX_enable;
extern bool DEV_TX_enable;

extern bool VMA_RX_enable;
extern bool VMA_TX_enable;
extern bool DEV_RX_enable;
extern bool DEV_TX_enable;


extern uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
extern uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
extern uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

extern uint8_t IMURequestBuf[IMU_REQUEST_LENGTH];
extern uint8_t IMUResponseBuf[IMU_RESPONSE_LENGTH];

extern uint8_t VMARequestBuf[VMA_DEV_REQUEST_LENGTH];
extern uint8_t VMAResponseBuf[VMA_DEV_RESPONSE_LENGTH];

extern uint8_t DevRequestBuf[VMA_DEV_REQUEST_LENGTH];
extern uint8_t DevResponseBuf[VMA_DEV_RESPONSE_LENGTH];


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void LedBlinkingTask(void const *argument);
void ShoreCommunicationTask(void const *argument);
void VmaDevCommunicationTask(void const *argument);
void SensorsCommunicationTask(void const *argument);
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
	shore_RX_enable = true;
	shore_TX_enable = false;
	
	HAL_HalfDuplex_EnableReceiver(&shoreUart);
  /* Infinite loop */
  for(;;){
		if(shore_RX_enable && shoreUart.RxState == HAL_UART_STATE_READY){
			HAL_UART_Receive_DMA(&shoreUart, ShoreRequestBuf, SHORE_REQUEST_LENGTH);
			shore_RX_enable = false;
		}
		else if(shore_TX_enable && shoreUart.gState == HAL_UART_STATE_READY){
			for(uint8_t i = 0; i < SHORE_REQUEST_LENGTH; ++i){
				ShoreResponseBuf[i] = (ShoreResponseBuf[i] + 1 + i) % 256;
			}
			HAL_UART_Transmit_DMA(&shoreUart, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
			shore_TX_enable = false;
		}
  }
  /* USER CODE END ShoreCommunicationTask */
}

/* VmaDevCommunicationTask function */
void VmaDevCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN VmaDevCommunicationTask */
	VMA_RX_enable = false;
	VMA_TX_enable = true;
	DEV_RX_enable = false;
	DEV_TX_enable = true;
	
	int8_t VMATransaction = 0, DevTransaction = 0;
	
	HAL_HalfDuplex_EnableTransmitter(&vmaUart);
	HAL_HalfDuplex_EnableTransmitter(&devUart);
	
  /* Infinite loop */
  for(;;){
		if (VMA_TX_enable && vmaUart.gState == HAL_UART_STATE_READY){
			switch(VMATransaction){
				case 0:
					VMARequestUpdate(&Q100, VMARequestBuf, HLF);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 1:
					VMARequestUpdate(&Q100, VMARequestBuf, HLB);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 2:
					VMARequestUpdate(&Q100, VMARequestBuf, HRB);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 3:
					VMARequestUpdate(&Q100, VMARequestBuf, HRF);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 4:
					VMARequestUpdate(&Q100, VMARequestBuf, VF);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 5:
					VMARequestUpdate(&Q100, VMARequestBuf, VL);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 6:
					VMARequestUpdate(&Q100, VMARequestBuf, VB);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 7:
					VMARequestUpdate(&Q100, VMARequestBuf, VR);
					HAL_UART_Transmit_IT(&vmaUart, VMARequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
			}
		}
		
		else if (DEV_TX_enable && devUart.gState == HAL_UART_STATE_READY){
			switch(DevTransaction){
				case 0:
					DevRequestUpdate(&Q100, DevRequestBuf, AGAR);
					HAL_UART_Transmit_IT(&devUart, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					DEV_TX_enable = false;
					break;
				case 1:
					DevRequestUpdate(&Q100, DevRequestBuf, GRUB);
					HAL_UART_Transmit_IT(&devUart, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					DEV_TX_enable = false;
					break;
				case 2:
					DevRequestUpdate(&Q100, DevRequestBuf, GRUBROTATION);
					HAL_UART_Transmit_IT(&devUart, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					DEV_TX_enable = false;
					break;
				case 3:
					DevRequestUpdate(&Q100, DevRequestBuf, TILT);
					HAL_UART_Transmit_IT(&devUart, DevRequestBuf, VMA_DEV_REQUEST_LENGTH);
					DEV_TX_enable = false;
					break;
			}
		}
		
		else if (VMA_RX_enable && vmaUart.RxState == HAL_UART_STATE_READY){
			switch(VMATransaction){
				case 0:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 1:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 2:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 3:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 4:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 5:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 6:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 7:
					HAL_UART_Receive_IT(&vmaUart, VMAResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
			}
			VMATransaction = (VMATransaction + 1) % VMA_DRIVER_NUMBER;
			HAL_Delay(20);
		}
		else if (DEV_RX_enable && devUart.RxState == HAL_UART_STATE_READY){
			switch(DevTransaction){
				case 0:
					HAL_UART_Receive_IT(&devUart, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DEV_TX_enable = false;
					break;
				case 1:
					HAL_UART_Receive_IT(&devUart, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DEV_TX_enable = false;
					break;
				case 2:
					HAL_UART_Receive_IT(&devUart, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DEV_TX_enable = false;
					break;
				case 3:
					HAL_UART_Receive_IT(&devUart, DevResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					DEV_TX_enable = false;
					break;
			}
			DevTransaction = (DevTransaction + 1) % DEV_DRIVER_NUMBER;
			HAL_Delay(20);
		}
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



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &shoreUart){
		HAL_HalfDuplex_EnableReceiver(&shoreUart);
		shore_RX_enable = true;
	}
	else if(huart == &vmaUart){
		HAL_HalfDuplex_EnableReceiver(&vmaUart);
		VMA_RX_enable = true;
	}
	else if(huart == &devUart){
		HAL_HalfDuplex_EnableReceiver(&devUart);
		DEV_RX_enable = true;
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &shoreUart){
		HAL_HalfDuplex_EnableTransmitter(&shoreUart);
		shore_TX_enable = true;
	}
	else if(huart == &vmaUart){
		HAL_HalfDuplex_EnableTransmitter(&vmaUart);
		VMA_TX_enable = true;
	}
	else if(huart == &devUart){
		HAL_HalfDuplex_EnableTransmitter(&devUart);
		DEV_TX_enable = true;		
	}
}



void IMU_Reset()
{
  IMURequestBuf[0] = 's';
  IMURequestBuf[1] = 'n';
  IMURequestBuf[2] = 'p';
  IMURequestBuf[3] = 0x00;
  IMURequestBuf[4] = 0xAC;  // Zero_gyros
  uint16_t checksum = IMUchecksum(IMURequestBuf, 5);
  IMURequestBuf[5] = checksum >> 8;
  IMURequestBuf[6] = (uint8_t) checksum;
  IMURequestBuf[7] = 's';
  IMURequestBuf[8] = 'n';
  IMURequestBuf[9] = 'p';
  IMURequestBuf[10] = 0x00;
  IMURequestBuf[11] = 0xAD;  // Reset EKF
  checksum = IMUchecksum(IMURequestBuf + 7, 5);
  IMURequestBuf[12] = checksum >> 8;
  IMURequestBuf[13] = (uint8_t) checksum;
  IMURequestBuf[14] = 's';
  IMURequestBuf[15] = 'n';
  IMURequestBuf[16] = 'p';
  IMURequestBuf[17] = 0x00;
  IMURequestBuf[18] = 0xAF;  // Set Accelerometer Reference vector
  checksum = IMUchecksum(IMURequestBuf + 14, 5);
  IMURequestBuf[19] = checksum >> 8;
  IMURequestBuf[20] = (uint8_t) checksum;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
