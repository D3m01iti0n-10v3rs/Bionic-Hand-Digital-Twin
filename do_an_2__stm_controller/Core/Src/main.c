/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t finger;
    uint8_t angle;
} ServoCommand_t;

typedef struct {
	uint8_t angle;
	uint8_t target_angle;
} SensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FINGER0_CHANNEL TIM_CHANNEL_1
#define FINGER1_CHANNEL TIM_CHANNEL_2
#define FINGER2_CHANNEL TIM_CHANNEL_3
#define FINGER3_CHANNEL TIM_CHANNEL_4
#define FINGER4_CHANNEL TIM_CHANNEL_1

#define ANGLE_STEP 1
#define SERVO_UPDATE_INTERVAL 10

#define SAMPLING_INTERVAL 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for comsTask */
osThreadId_t comsTaskHandle;
uint32_t comsTaskBuffer[ 512 ];
osStaticThreadDef_t comsTaskControlBlock;
const osThreadAttr_t comsTask_attributes = {
  .name = "comsTask",
  .cb_mem = &comsTaskControlBlock,
  .cb_size = sizeof(comsTaskControlBlock),
  .stack_mem = &comsTaskBuffer[0],
  .stack_size = sizeof(comsTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
uint32_t servoTaskBuffer[ 128 ];
osStaticThreadDef_t servoTaskControlBlock;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .cb_mem = &servoTaskControlBlock,
  .cb_size = sizeof(servoTaskControlBlock),
  .stack_mem = &servoTaskBuffer[0],
  .stack_size = sizeof(servoTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
uint32_t sensorTaskBuffer[ 512 ];
osStaticThreadDef_t sensorTaskControlBlock;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .cb_mem = &sensorTaskControlBlock,
  .cb_size = sizeof(sensorTaskControlBlock),
  .stack_mem = &sensorTaskBuffer[0],
  .stack_size = sizeof(sensorTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for finger_data_mutex */
osMutexId_t finger_data_mutexHandle;
osStaticMutexDef_t finger_data_mutexControlBlock;
const osMutexAttr_t finger_data_mutex_attributes = {
  .name = "finger_data_mutex",
  .cb_mem = &finger_data_mutexControlBlock,
  .cb_size = sizeof(finger_data_mutexControlBlock),
};
/* Definitions for sensorSem */
osSemaphoreId_t sensorSemHandle;
osStaticSemaphoreDef_t sensorSemControlBlock;
const osSemaphoreAttr_t sensorSem_attributes = {
  .name = "sensorSem",
  .cb_mem = &sensorSemControlBlock,
  .cb_size = sizeof(sensorSemControlBlock),
};
/* Definitions for rxSem */
osSemaphoreId_t rxSemHandle;
osStaticSemaphoreDef_t rxSemControlBlock;
const osSemaphoreAttr_t rxSem_attributes = {
  .name = "rxSem",
  .cb_mem = &rxSemControlBlock,
  .cb_size = sizeof(rxSemControlBlock),
};
/* USER CODE BEGIN PV */
osSemaphoreId_t sensorSemHandle;

osMessageQueueId_t servoCmdQueue;
Servo_t fingers[5];
SensorData_t finger_data[5];

char uart_rx_buf[128];
char uart_tx_buf[128];
volatile uint8_t rx_received = 0;
volatile uint16_t rx_len = 0;

char coms_processing_buf[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartComsTask(void *argument);
void StartServoTask(void *argument);
void StartSensorTask(void *argument);

/* USER CODE BEGIN PFP */
void uart_send(const char *fmt, ...){
  va_list args;
  va_start(args, fmt);
  vsnprintf(uart_tx_buf, sizeof(uart_tx_buf), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), 1000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  fingers[0] = ServoInit(&htim2, FINGER0_CHANNEL, 90);
  fingers[1] = ServoInit(&htim2, FINGER1_CHANNEL, 90);
  fingers[2] = ServoInit(&htim2, FINGER2_CHANNEL, 90);
  fingers[3] = ServoInit(&htim2, FINGER3_CHANNEL, 90);
  fingers[4] = ServoInit(&htim3, FINGER4_CHANNEL, 90);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf) - 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of finger_data_mutex */
  finger_data_mutexHandle = osMutexNew(&finger_data_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sensorSem */
  sensorSemHandle = osSemaphoreNew(1, 0, &sensorSem_attributes);

  /* creation of rxSem */
  rxSemHandle = osSemaphoreNew(1, 1, &rxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  servoCmdQueue = osMessageQueueNew(16, sizeof(ServoCommand_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of comsTask */
  comsTaskHandle = osThreadNew(StartComsTask, NULL, &comsTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        uart_rx_buf[Size] = '\0';
        memcpy(coms_processing_buf, uart_rx_buf, Size + 1);
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf) - 1);
        osSemaphoreRelease(rxSemHandle);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {  // fixed the uart bug
  if (huart->Instance == USART1) {
	  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf) - 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartComsTask */
/**
  * @brief  Function implementing the comsTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartComsTask */
void StartComsTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	int finger_num = 0;
	int angle = 0;
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(rxSemHandle, osWaitForever);
	  if (sscanf(coms_processing_buf, "FINGER%d %d", &finger_num, &angle) == 2){
		  ServoCommand_t cmd;
		  cmd.finger = (uint8_t)finger_num;
		  cmd.angle = (uint8_t)angle;
		  osMessageQueuePut(servoCmdQueue, &cmd, 0, 0);
	  }

	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  for(;;)
  {
      ServoCommand_t cmd;
      osMutexAcquire(finger_data_mutexHandle, osWaitForever);

      while (osMessageQueueGet(servoCmdQueue, &cmd, NULL, 0) == osOK) {
          if (cmd.finger < 5) fingers[cmd.finger].target_angle = (cmd.angle > 180) ? 180 : cmd.angle;
      }

      uint8_t moving = 0;
      for (uint8_t i = 0; i < 5; i++){
          uint8_t prev_angle = GetServoAngle(&fingers[i]);
          ServoMoveStep(&fingers[i], fingers[i].target_angle, ANGLE_STEP);
          if (GetServoAngle(&fingers[i]) != prev_angle) {
              moving = 1;
          }
      }

      osMutexRelease(finger_data_mutexHandle);

      if (moving) osSemaphoreRelease(sensorSemHandle);

      osDelay(SERVO_UPDATE_INTERVAL);
  }
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
    //char buffer[64];

    for(;;)
    {
    	osSemaphoreAcquire(sensorSemHandle, osWaitForever);
        osMutexAcquire(finger_data_mutexHandle, osWaitForever);

        for (uint8_t i = 0; i < 5; i++)
        {
            finger_data[i].angle = GetServoAngle(&fingers[i]);
            finger_data[i].target_angle = GetServoTargetAngle(&fingers[i]);
        }

        osMutexRelease(finger_data_mutexHandle);

        /*
        snprintf(buffer, sizeof(buffer),
                 "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 finger_data[0].angle, finger_data[0].target_angle,
                 finger_data[1].angle, finger_data[1].target_angle,
                 finger_data[2].angle, finger_data[2].target_angle,
                 finger_data[3].angle, finger_data[3].target_angle,
                 finger_data[4].angle, finger_data[4].target_angle);
        */

        uart_send(
                 "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 finger_data[0].angle, finger_data[0].target_angle,
                 finger_data[1].angle, finger_data[1].target_angle,
                 finger_data[2].angle, finger_data[2].target_angle,
                 finger_data[3].angle, finger_data[3].target_angle,
                 finger_data[4].angle, finger_data[4].target_angle);

        osDelay(1);
    }
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
#ifdef USE_FULL_ASSERT
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
