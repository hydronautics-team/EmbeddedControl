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
#include "messages.h"
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
struct robot Ice;

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



uint8_t ShoreRequestBuf[SHORE_REQUEST_LENGTH];
uint8_t ShoreRequestConfigBuf[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuf[SHORE_RESPONSE_LENGTH];

uint8_t IMU_RequestBuf[IMU_REQUEST_LENGTH];
uint8_t IMU_ResponseBuf[IMU_RESPONSE_LENGTH];

uint8_t HLF_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x01, 0x7F, 1, 2, 0xD7};
uint8_t HLF_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t HLB_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x02, 0x7F, 1, 2, 0xD8};
uint8_t HLB_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t HRB_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x03, 0x7F, 1, 2, 0xD9};
uint8_t HRB_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t HRF_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x04, 0x7F, 1, 2, 0xDA};
uint8_t HRF_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t VF_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x05, 0x7F, 1, 2, 0xDB};
uint8_t VF_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t VL_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x06, 0x7F, 1, 2, 0xDC};
uint8_t VL_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t VB_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x07, 0x7F, 1, 2, 0xDD};
uint8_t VB_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t VR_RequestBuf[VMA_DEV_REQUEST_LENGTH] = {0xAA, 0xAA, 0x08, 0x7F, 1, 2, 0xDE};
uint8_t VR_ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t Dev1RequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t Dev1ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t Dev2RequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t Dev2ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t Dev3RequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t Dev3ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

uint8_t Dev4RequestBuf[VMA_DEV_REQUEST_LENGTH];
uint8_t Dev4ResponseBuf[VMA_DEV_RESPONSE_LENGTH];

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
	shore_RX_enable = true;
	shore_TX_enable = false;
	
	HAL_HalfDuplex_EnableReceiver(&shore_uart);
  /* Infinite loop */
  for(;;){
		if(shore_RX_enable && shore_uart.RxState == HAL_UART_STATE_READY){
			HAL_UART_Receive_DMA(&shore_uart, ShoreRequestBuf, SHORE_REQUEST_LENGTH);
			shore_RX_enable = false;
		}
		else if(shore_TX_enable && shore_uart.gState == HAL_UART_STATE_READY){
			for(uint8_t i = 0; i < SHORE_REQUEST_LENGTH; ++i){
				ShoreResponseBuf[i] = (ShoreResponseBuf[i] + 1 + i) % 256;
			}
			HAL_UART_Transmit_DMA(&shore_uart, ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
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
	
	uint8_t i = 2;
	
	HAL_HalfDuplex_EnableTransmitter(&vma_dev_uart);
	HAL_HalfDuplex_EnableTransmitter(&dev_uart);
	
  /* Infinite loop */
  for(;;){
		if (VMA_TX_enable && vma_dev_uart.gState == HAL_UART_STATE_READY){
			switch(i){
				case 1:
					HAL_UART_Transmit_IT(&vma_dev_uart, HLF_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 2:
					HAL_UART_Transmit_IT(&vma_dev_uart, HLB_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 3:
					HAL_UART_Transmit_IT(&vma_dev_uart, HRB_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 4:
					HAL_UART_Transmit_IT(&vma_dev_uart, HRF_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 5:
					HAL_UART_Transmit_IT(&vma_dev_uart, VF_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 6:
					HAL_UART_Transmit_IT(&vma_dev_uart, VL_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 7:
					HAL_UART_Transmit_IT(&vma_dev_uart, VB_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
				case 8:
					HAL_UART_Transmit_IT(&vma_dev_uart, VR_RequestBuf, VMA_DEV_REQUEST_LENGTH);
					VMA_TX_enable = false;
					break;
			}
		}
		else if (DEV_TX_enable && dev_uart.gState == HAL_UART_STATE_READY){
			HAL_UART_Transmit_IT(&dev_uart, HLF_RequestBuf, VMA_DEV_REQUEST_LENGTH);
			DEV_TX_enable = false;
		}
		else if (VMA_RX_enable && vma_dev_uart.RxState == HAL_UART_STATE_READY){
			switch(i){
				case 1:
					HAL_UART_Receive_IT(&vma_dev_uart, HLF_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 2:
					HAL_UART_Receive_IT(&vma_dev_uart, HLB_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 3:
					HAL_UART_Receive_IT(&vma_dev_uart, HRB_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 4:
					HAL_UART_Receive_IT(&vma_dev_uart, HRF_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 5:
					HAL_UART_Receive_IT(&vma_dev_uart, VF_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 6:
					HAL_UART_Receive_IT(&vma_dev_uart, VL_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 7:
					HAL_UART_Receive_IT(&vma_dev_uart, VB_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
				case 8:
					HAL_UART_Receive_IT(&vma_dev_uart, VR_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
					VMA_TX_enable = false;
					break;
			}
			i = (i + 1) % 5;
			if (i == 0){
				i = 2;
			}
			HAL_Delay(20);
		}
		else if (DEV_RX_enable && dev_uart.RxState == HAL_UART_STATE_READY){
			HAL_UART_Receive_IT(&dev_uart, HLF_ResponseBuf, VMA_DEV_RESPONSE_LENGTH);
			DEV_RX_enable = false;
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StabilizationTask */
}

/* USER CODE BEGIN Application */



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &shore_uart){
		HAL_HalfDuplex_EnableReceiver(&shore_uart);
		shore_RX_enable = true;
	}
	else if(huart == &vma_dev_uart){
		HAL_HalfDuplex_EnableReceiver(&vma_dev_uart);
		VMA_RX_enable = true;
	}
	else if(huart == &dev_uart){
		HAL_HalfDuplex_EnableReceiver(&dev_uart);
		DEV_RX_enable = true;
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &shore_uart){
		HAL_HalfDuplex_EnableTransmitter(&shore_uart);
		shore_TX_enable = true;
	}
	else if(huart == &vma_dev_uart){
		HAL_HalfDuplex_EnableTransmitter(&vma_dev_uart);
		VMA_TX_enable = true;
	}
	else if(huart == &dev_uart){
		HAL_HalfDuplex_EnableTransmitter(&dev_uart);
		DEV_TX_enable = true;		
	}
}



/* CRC16-CCITT algorithm */
uint8_t IsChecksumm16bCorrect(uint8_t *msg, uint16_t length)
{
  int i;
	uint16_t crc = 0, crc_got = (uint16_t)((msg[length - 2] << 8) + msg[length - 1]);
	
		for(i = 0; i < length - 2; ++i){
			crc = (uint8_t)(crc >> 8) | (crc << 8);
			crc ^= msg[i];
			crc ^= (uint8_t)(crc & 0xff) >> 4;
			crc ^= (crc << 8) << 4;
			crc ^= ((crc & 0xff) << 4) << 1;
		}
	
	if(crc == crc_got ){
    return 1;
  }
	else{
    return 0;
  }
}



/* CRC16-CCITT algorithm */
void AddChecksumm16b(uint8_t *msg, uint16_t length)
{
	uint16_t crc = 0;
	int i = 0;
	
	for(i = 0; i < length - 2; ++i){
			crc = (uint8_t)(crc >> 8) | (crc << 8);
			crc ^= msg[i];
			crc ^= (uint8_t)(crc & 0xff) >> 4;
			crc ^= (crc << 8) << 4;
			crc ^= ((crc & 0xff) << 4) << 1;
		}
	
	msg[length - 2] = (uint8_t) (crc >> 8);
	msg[length - 1] = (uint8_t) crc;
}



void AddChecksumm8b(uint8_t *msg, uint16_t length)
{
	uint8_t crc = 0;
	int i = 0;
	
	for(i = 0; i < length - 1; i++){
			crc ^= msg[i];
		}
	
	msg[length-1] = crc;
}



float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos)
{
  float result;
  result = (float)((buff[high_byte_pos] << 24) + (buff[high_byte_pos + 1] << 16) + (buff[high_byte_pos + 2] << 8) + buff[high_byte_pos + 3]);
  return result;
}



void ShoreConfigRequest()
{
  Ice.depth_stabilization.const_time = ShoreRequestConfigBuf[REQUEST_CONFIG_CONST_TIME_DEPTH];
  Ice.depth_stabilization.K1 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K1_DEPTH);
  Ice.depth_stabilization.K2 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K2_DEPTH);
  Ice.depth_stabilization.start_value = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_START_DEPTH);
  Ice.depth_stabilization.gain = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_GAIN_DEPTH);
  
  Ice.roll_stabilization.const_time = ShoreRequestConfigBuf[REQUEST_CONFIG_CONST_TIME_ROLL];
  Ice.roll_stabilization.K1 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K1_ROLL);
  Ice.roll_stabilization.K2 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K2_ROLL);
  Ice.roll_stabilization.start_value = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_START_ROLL);
  Ice.roll_stabilization.gain = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_GAIN_ROLL);
  
  Ice.pitch_stabilization.const_time = ShoreRequestConfigBuf[REQUEST_CONFIG_CONST_TIME_PITCH];
  Ice.pitch_stabilization.K1 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K1_PITCH);
  Ice.pitch_stabilization.K2 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K2_PITCH);
  Ice.pitch_stabilization.start_value = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_START_PITCH);
  Ice.pitch_stabilization.gain = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_GAIN_PITCH);
  
  Ice.yaw_stabilization.const_time = ShoreRequestConfigBuf[REQUEST_CONFIG_CONST_TIME_YAW];
  Ice.yaw_stabilization.K1 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K1_YAW);
  Ice.yaw_stabilization.K2 = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K2_YAW);
  Ice.yaw_stabilization.start_value = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_START_YAW);
  Ice.yaw_stabilization.gain = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_GAIN_YAW);
  
  Ice.HLB.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_HLB];
  Ice.HLF.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_HLF];
  Ice.HRB.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_HRB];
  Ice.HRF.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_HRF];
  Ice.VB.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_VB];
  Ice.VF.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_VF];
  Ice.VL.address  = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_VL];
  Ice.VR.address = ShoreRequestConfigBuf[REQUEST_CONFIG_POSITION_VR];
  
  Ice.HLB.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_HLB];
  Ice.HLF.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_HLF];
  Ice.HRB.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_HRB];
  Ice.HRF.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_HRF];
  Ice.VB.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_VB];
  Ice.VF.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_VF];
  Ice.VL.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_VL];
  Ice.VR.inverse = ShoreRequestConfigBuf[REQUEST_CONFIG_INVERSE_VR];
  
  Ice.HLB.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_HLB);
  Ice.HLF.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_HLF);
  Ice.HRB.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_HRB);
  Ice.HRF.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_HRF);
  Ice.VB.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_VB);
  Ice.VF.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_VF);
  Ice.VL.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_VL);
  Ice.VR.k_forward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_FORWARD_VR);
  
  Ice.HLB.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_HLB);
  Ice.HLF.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_HLF);
  Ice.HRB.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_HRB);
  Ice.HRF.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_HRF);
  Ice.VB.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_VB);
  Ice.VF.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_VF);
  Ice.VL.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_VL);
  Ice.VR.k_backward = FloatFromUint8(ShoreRequestConfigBuf, REQUEST_CONFIG_K_BACKWARD_VR);
}



void ShoreRequest()
{
  Ice.movement.march = (int16_t)((ShoreRequestBuf[SHORE_REQUEST_MARCH] << 8) + ShoreRequestBuf[SHORE_REQUEST_MARCH + 1]);
  Ice.movement.lag = (int16_t)((ShoreRequestBuf[SHORE_REQUEST_LAG] << 8) + ShoreRequestBuf[SHORE_REQUEST_LAG + 1]);
  Ice.movement.depth = (int16_t)((ShoreRequestBuf[SHORE_REQUEST_DEPTH] << 8) + ShoreRequestBuf[SHORE_REQUEST_DEPTH + 1]);
  Ice.movement.pitch = (int16_t)((ShoreRequestBuf[SHORE_REQUEST_ROLL] << 8) + ShoreRequestBuf[SHORE_REQUEST_ROLL + 1]);
  Ice.movement.roll = (int16_t)((ShoreRequestBuf[SHORE_REQUEST_PITCH] << 8) + ShoreRequestBuf[SHORE_REQUEST_PITCH + 1]);
  Ice.movement.yaw = (int16_t)((ShoreRequestBuf[SHORE_REQUEST_YAW] << 8) + ShoreRequestBuf[SHORE_REQUEST_YAW + 1]);

  Ice.device.light = ShoreRequestBuf[SHORE_REQUEST_LIGHT];
  Ice.device.bluetooth_light = ShoreRequestBuf[SHORE_REQUEST_BLUETOOTH];
  Ice.device.bottom_light = ShoreRequestBuf[SHORE_REQUEST_BOTTOM_LIGHT];
  Ice.device.grab = ShoreRequestBuf[SHORE_REQUEST_GRAB];
  Ice.device.grab_rotate  = ShoreRequestBuf[SHORE_REQUEST_GRAB_ROTATE]; 
  Ice.device.tilt = ShoreRequestBuf[SHORE_REQUEST_TILT];

  Ice.depth_stabilization.enable = (bool) ShoreRequestBuf[SHORE_REQUEST_STABILIZE_DEPTH];
  Ice.roll_stabilization.enable = (bool) ShoreRequestBuf[SHORE_REQUEST_STABILIZE_ROLL];
  Ice.pitch_stabilization.enable = (bool) ShoreRequestBuf[SHORE_REQUEST_STABILIZE_PITCH ];
  Ice.yaw_stabilization.enable = (bool) ShoreRequestBuf[SHORE_REQUEST_STABILIZE_YAW ];

  Ice.device.reset_IMU = (bool) ShoreRequestBuf[SHORE_REQUEST_RESET_IMU];
}



void ShoreResponse()
{ 
  ShoreResponseBuf[SHORE_RESPONSE_ROLL] = (uint8_t)(Ice.sensors.roll >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_ROLL + 1] = (uint8_t)(Ice.sensors.roll);  // does it work?
  ShoreResponseBuf[SHORE_RESPONSE_PITCH] = (uint8_t)(Ice.sensors.pitch >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_PITCH + 1] = (uint8_t)(Ice.sensors.pitch);
  ShoreResponseBuf[SHORE_RESPONSE_YAW] = (uint8_t)(Ice.sensors.yaw >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_YAW + 1] = (uint8_t)(Ice.sensors.yaw);
  ShoreResponseBuf[SHORE_RESPONSE_ROLL_SPEED] = (uint8_t)(Ice.sensors.roll_speed >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_ROLL_SPEED + 1] = (uint8_t)(Ice.sensors.roll_speed);
  ShoreResponseBuf[SHORE_RESPONSE_PITCH_SPEED] = (uint8_t)(Ice.sensors.pitch_speed >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_PITCH_SPEED + 1] = (uint8_t)(Ice.sensors.pitch_speed);
  ShoreResponseBuf[SHORE_RESPONSE_YAW_SPEED] = (uint8_t)(Ice.sensors.yaw_speed >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_YAW_SPEED + 1] = (uint8_t)(Ice.sensors.yaw_speed);
  ShoreResponseBuf[SHORE_RESPONSE_TEMPERATURE] = 0x55;
  ShoreResponseBuf[SHORE_RESPONSE_TEMPERATURE + 1] = 0xAA;
  ShoreResponseBuf[SHORE_RESPONSE_PRESSURE] = (uint8_t)(Ice.sensors.pressure >> 8);
  ShoreResponseBuf[SHORE_RESPONSE_PRESSURE + 1] = (uint8_t)(Ice.sensors.pressure);
  ShoreResponseBuf[SHORE_RESPONSE_MOTOR_ERRORS] = 0x55;
  ShoreResponseBuf[SHORE_RESPONSE_MOTOR_ERRORS + 1] = 0xAA;
  
  AddChecksumm16b(ShoreResponseBuf, SHORE_RESPONSE_LENGTH);
}



void VelocityDetermination()
{
  Ice.HLB.speed = -  Ice.movement.march + Ice.movement.lag  - (Ice.movement.yaw + Ice.yaw_stabilization.error_speed); 
  Ice.HLF.speed = +  Ice.movement.march + Ice.movement.lag  + (Ice.movement.yaw + Ice.yaw_stabilization.error_speed); 
  Ice.HRB.speed = -  Ice.movement.march - Ice.movement.lag  + (Ice.movement.yaw + Ice.yaw_stabilization.error_speed);
  Ice.HRF.speed = +  Ice.movement.march - Ice.movement.lag  - (Ice.movement.yaw + Ice.yaw_stabilization.error_speed);
  Ice.VB.speed =  -  Ice.movement.depth + (Ice.movement.pitch + Ice.pitch_stabilization.error_speed); 
  Ice.VF.speed =  +  Ice.movement.depth + (Ice.movement.pitch + Ice.pitch_stabilization.error_speed); 
  Ice.VL.speed =  - (Ice.movement.depth + Ice.depth_stabilization.error_speed) + (Ice.movement.roll + Ice.roll_stabilization.error_speed);
  Ice.VR.speed =  - (Ice.movement.depth + Ice.depth_stabilization.error_speed) - (Ice.movement.roll + Ice.roll_stabilization.error_speed);
}



void IMU_Response()
{
  uint8_t i = 0;
	while(i < IMU_RESPONSE_LENGTH){
		if ((IMU_ResponseBuf[i] == 's') && (IMU_ResponseBuf[(i + 1) % IMU_RESPONSE_LENGTH] == 'n') && (IMU_ResponseBuf[(i + 2) % IMU_RESPONSE_LENGTH] == 'p') && (IMU_ResponseBuf[(i + 3) % IMU_RESPONSE_LENGTH] == 0xC8)){
			switch(IMU_ResponseBuf[(i + 4) % IMU_RESPONSE_LENGTH]){
				case 0x62:
					Ice.sensors.roll = (int16_t) ((IMU_ResponseBuf[(i + 5) % IMU_RESPONSE_LENGTH] << 8 ) + IMU_ResponseBuf[(i + 6) % IMU_RESPONSE_LENGTH]);
					Ice.sensors.pitch = (int16_t) ((IMU_ResponseBuf[(i + 7) % IMU_RESPONSE_LENGTH] << 8 ) + IMU_ResponseBuf[(i + 8) % IMU_RESPONSE_LENGTH]);
					Ice.sensors.yaw = (int16_t) ((IMU_ResponseBuf[(i + 9) % IMU_RESPONSE_LENGTH] << 8 ) + IMU_ResponseBuf[(i + 10) % IMU_RESPONSE_LENGTH]);
					i = i + 14;
				break;
				case 0x5C:
					Ice.sensors.roll_speed = (int16_t) ((IMU_ResponseBuf[(i + 5) % IMU_RESPONSE_LENGTH] << 8 ) + IMU_ResponseBuf[(i + 6) % IMU_RESPONSE_LENGTH]);
					Ice.sensors.pitch_speed = (int16_t) ((IMU_ResponseBuf[(i + 7) % IMU_RESPONSE_LENGTH] << 8 ) + IMU_ResponseBuf[(i + 8) % IMU_RESPONSE_LENGTH]);
					Ice.sensors.yaw_speed = (int16_t) ((IMU_ResponseBuf[(i + 9) % IMU_RESPONSE_LENGTH] << 8 ) + IMU_ResponseBuf[(i + 10) % IMU_RESPONSE_LENGTH]);
					i = i + 14;
				break;
			}
		}
		++i;
	}
}



uint16_t IMU_checksum(uint8_t* arr, uint8_t size)
{
  uint16_t checksum = 0;
  for(uint8_t i = 0; i < size; ++i){
    checksum += arr[i];
  }
  return checksum;
}



void IMU_Reset()
{
  IMU_RequestBuf[0] = 's';
  IMU_RequestBuf[1] = 'n';
  IMU_RequestBuf[2] = 'p';
  IMU_RequestBuf[3] = 0x00;
  IMU_RequestBuf[4] = 0xAC;  // Zero_gyros
  uint16_t checksum = IMU_checksum(IMU_RequestBuf, 5);
  IMU_RequestBuf[5] = checksum >> 8;
  IMU_RequestBuf[6] = (uint8_t) checksum;
  IMU_RequestBuf[7] = 's';
  IMU_RequestBuf[8] = 'n';
  IMU_RequestBuf[9] = 'p';
  IMU_RequestBuf[10] = 0x00;
  IMU_RequestBuf[11] = 0xAD;  // Reset EKF
  checksum = IMU_checksum(IMU_RequestBuf + 7, 5);
  IMU_RequestBuf[12] = checksum >> 8;
  IMU_RequestBuf[13] = (uint8_t) checksum;
  IMU_RequestBuf[14] = 's';
  IMU_RequestBuf[15] = 'n';
  IMU_RequestBuf[16] = 'p';
  IMU_RequestBuf[17] = 0x00;
  IMU_RequestBuf[18] = 0xAF;  // Set Accelerometer Reference vector
  checksum = IMU_checksum(IMU_RequestBuf + 14, 5);
  IMU_RequestBuf[19] = checksum >> 8;
  IMU_RequestBuf[20] = (uint8_t) checksum;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
