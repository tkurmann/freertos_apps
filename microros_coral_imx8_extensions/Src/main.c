/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "api.h"

#include <allocators.h>
#include <rcl/rcl.h>
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>



#include "FreeRTOS.h"
#include "task.h"

#include <rmw_microxrcedds_c/config.h>



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


/* Definitions for initTask */
osThreadId_t initTaskHandle;
osThreadId_t idleTaskHandle;

const osThreadAttr_t initTask_attributes = {
  .name = "initTask",
  .priority = (osPriority_t) osPriorityBelowNormal7,
  .stack_size = 1500
};

const osThreadAttr_t idleTask_attributes = {
  .name = "idleTask",
  .priority = (osPriority_t) osPriorityIdle,
  .stack_size = 500
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initTaskFunction(void *argument);

/* USER CODE BEGIN PFP */
#define BUFSIZE 4096
char buffer[BUFSIZE];
extern struct netif gnetif;
// extern appMain;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  // if (printf_uart != NULL){
  //   // TODO: some transmit code here
  //    // HAL_UART_Transmit(printf_uart, &c[0], 1, 10);
  // }
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++){
  __io_putchar(*ptr++);
  }
  return len;
}

static void idleTaskFunction(void *param) {
  gpio_pin_config_t out_config = {kGPIO_DigitalOutput, 0,  kGPIO_NoIntmode};

  GPIO_PinInit(GPIO1, 8u, &out_config);


  for(;;) {
    // vTaskDelay(100);
     GPIO_PinWrite(GPIO1, 8u, 0U);
     GPIO_PinWrite(GPIO1, 8u, 1U);

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
  BOARD_RdcInit();

  BOARD_InitBootPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();
  // CLOCK_EnableClock(kCLOCK_Uart1);
  NVIC_SetPriority(UART3_IRQn, 5);
  // NVIC_SetPriority(UART1_IRQn, 6);
  PRINTF("\r\nIt boots..\r\n");







  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  // SystemClock_Config();
  //
  // /* USER CODE BEGIN SysInit */
  //
  // /* USER CODE END SysInit */
  //
  // /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  // MX_DMA_Init();
  // MX_USART3_UART_Init();
  // MX_LWIP_Init();
  // MX_USART1_UART_Init();
  // MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

// #ifdef MICRO_XRCEDDS_UDP
//   printf_uart = &huart3;
// #elif defined(MICRO_XRCEDDS_CUSTOM_SERIAL)
//   if (!strcmp("3",RMW_UXRCE_DEFAULT_SERIAL_DEVICE)){
//     printf_uart = &huart6;
//   }else{
//     printf_uart = &huart3;
//   }
// #endif

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of initTask */
  initTaskHandle = osThreadNew(initTaskFunction, NULL, &initTask_attributes);
  idleTaskHandle = osThreadNew(idleTaskFunction, NULL, &idleTask_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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


/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_initTaskFunction */
/**
  * @brief  Function implementing the initTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_initTaskFunction */
void initTaskFunction(void *argument)
{

  PRINTF("\r\nIt even creates a task...\r\n");
  // while (1){
  //   PRINTF("\r\nIt keeps going and going...\r\n");
  //   osDelay(1000);
  // }
  /* USER CODE BEGIN 5 */
  // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
  bool availableNetwork = false;

  availableNetwork = true;


  // Launch app thread when IP configured
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n",__LINE__);
  }
  PRINTF("\r\nHere\r\n");

  osThreadAttr_t attributes;
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "microROS_app";
  attributes.stack_size = 4*3000;
  attributes.priority = (osPriority_t) osPriorityNormal1;
  PRINTF("\r\nHere\r\n");
  osThreadNew(appMain, NULL, &attributes);
  PRINTF("\r\nHere\r\n");

  // osDelay(500);
  // char ptrTaskList[500];
  // vTaskList(ptrTaskList);
  // PRINTF("**********************************\n");
  // PRINTF("Task  State   Prio    Stack    Num\n");
  // PRINTF("**********************************\n");
  // PRINTF(ptrTaskList);
  // PRINTF("**********************************\n");
  //
  // PRINTF("\r\nHere\r\n");
  // TaskHandle_t xHandle;
  // xHandle = xTaskGetHandle("microROS_app");
  // PRINTF("\r\nHere\r\n");

  while (1){
    // PRINTF("\r\nIt keeps going and going...\r\n");
    osDelay(100000);
    char ptrTaskList[500];
    vTaskList(ptrTaskList);
    PRINTF("**********************************\n");
    PRINTF("Task  State   Prio    Stack    Num\n");
    PRINTF("**********************************\n");
    PRINTF(ptrTaskList);
    PRINTF("**********************************\n");

    PRINTF("\r\nHere\r\n");
    TaskHandle_t xHandle;
    xHandle = xTaskGetHandle("microROS_app");
    PRINTF("\r\nHere\r\n");
  }
//
//   while (1){
//     if (eTaskGetState(xHandle) != eSuspended && availableNetwork){
//       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//       osDelay(100);
//       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//       osDelay(100);
//       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//       osDelay(150);
//       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//       osDelay(500);
//     }else{
//       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//       osDelay(1000);
//       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//       osDelay(1000);
//     }
//   }


  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   /* USER CODE BEGIN Callback 0 */
//
//   /* USER CODE END Callback 0 */
//   if (htim->Instance == TIM1) {
//     HAL_IncTick();
//   }
//   /* USER CODE BEGIN Callback 1 */
//
//   /* USER CODE END Callback 1 */
// }

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
