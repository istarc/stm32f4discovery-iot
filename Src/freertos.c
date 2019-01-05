/* USER CODE BEGIN Header */
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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#define USE_PRINTF
#define USE_NETIF

#ifdef USE_PRINTF
/* Used to override _write */
#include <errno.h>
#include <sys/unistd.h>
#endif

#ifdef USE_NETIF
#include "socket.h"
#include <string.h>
#endif

#include "gpio.h"
#include "usart.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId blinkyTaskHandle;
osThreadId buttonTaskHandle;
osThreadId toggleTaskHandle;
osMessageQId buttonEventQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#ifdef USE_PRINTF
extern int _write(int file, char *data, int len);
#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartBlinkyTask(void const * argument);
void StartButtonTask(void const * argument);
void StartToggleTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of blinkyTask */
  osThreadDef(blinkyTask, StartBlinkyTask, osPriorityNormal, 0, 128);
  blinkyTaskHandle = osThreadCreate(osThread(blinkyTask), NULL);

  /* definition and creation of buttonTask */
  osThreadDef(buttonTask, StartButtonTask, osPriorityNormal, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* definition and creation of toggleTask */
  osThreadDef(toggleTask, StartToggleTask, osPriorityNormal, 0, 128);
  toggleTaskHandle = osThreadCreate(osThread(toggleTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of buttonEventQueue */
  osMessageQDef(buttonEventQueue, 10, uint8_t);
  buttonEventQueueHandle = osMessageCreate(osMessageQ(buttonEventQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBlinkyTask */
/**
* @brief Function implementing the blinkyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkyTask */
void StartBlinkyTask(void const * argument)
{
	/* USER CODE BEGIN StartBlinkyTask */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

		/*
		 Delay for a period of time. vTaskDelay() places the task into
		 the Blocked state until the period has expired.
		 The delay period is spacified in 'ticks'. We can convert
		 yhis in milisecond with the constant portTICK_RATE_MS.
		 */
		vTaskDelay(1500 / portTICK_RATE_MS);
	}
	/* USER CODE END StartBlinkyTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
	/* USER CODE BEGIN StartButtonTask */
	uint8_t sig = 1u;
	char msg[] = ".";
#ifdef USE_NETIF
	struct sockaddr_in dstaddr;
	memset(&dstaddr, 0, sizeof(dstaddr));
	dstaddr.sin_family = AF_INET;
	dstaddr.sin_port = PP_HTONS(8080);
	dstaddr.sin_addr.s_addr = inet_addr("192.168.0.1");
	int sh = socket(AF_INET, SOCK_DGRAM, 0); /* TODO Implement Error Handling */
	connect(sh, (struct sockaddr* )&dstaddr, sizeof(dstaddr)); /* TODO Implement Error Handling */
#endif

	/* Infinite loop */
	for (;;) {
		/* Detect Button Press  */
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) > 0) {
			while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) > 0)
				vTaskDelay(100 / portTICK_RATE_MS); /* Button Debounce Delay */
			while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
				vTaskDelay(100 / portTICK_RATE_MS); /* Button Debounce Delay */

			xQueueSendToBack(buttonEventQueueHandle, &sig, 0); /* Send Message */

#ifdef USE_PRINTF
			printf("%s", msg);
			fflush(stdout);
#else
			HAL_UART_Transmit(&huart6, msg, sizeof(msg), 0); /* TODO Implement Error Handling */
#endif

#ifdef USE_NETIF
			send(sh, msg, sizeof(msg), 0); /* TODO Implement Error Handling */
#endif
		}
	}
	/* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartToggleTask */
/**
* @brief Function implementing the toggleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartToggleTask */
void StartToggleTask(void const * argument)
{
	/* USER CODE BEGIN StartToggleTask */
	uint8_t sig;
	portBASE_TYPE status;
	/* Infinite loop */
	for (;;) {
		status = xQueueReceive(buttonEventQueueHandle, &sig, portMAX_DELAY); /* Receive Message */
		/* portMAX_DELAY blocks task indefinitely if queue is empty */
		if (status == pdTRUE) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		}
	}
	/* USER CODE END StartToggleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#ifdef USE_PRINTF
/**
 * @brief Function overriding _write routine to redirect stdout to USART6
 * @param file: File descriptor
 * @param data: Pointer to data buffer
 * @param len:  Amount of data to be sent
 * @retval 0:   OK, -1: Error
 * @note Reentrancy:  No (because it calls non-reentrant function that uses a global variable and HW registers)
 * @note Thread-safe: No (the called function does not ensure the global data in a consistent state during execution nor synchronizes access to the HW registers)
 * @note Sync/Async:  Synchronous (the called transmit function busy waits and returns after data is physically sent over the wire, caveat: wastes CPU time)
 */
extern int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, (uint8_t*)data, len, 0); /* TODO Implement Busy, Timeout, Error Handling */

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}
#endif
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
