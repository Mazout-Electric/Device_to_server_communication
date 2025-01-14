/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define RX_BUFFER_SIZE 256
#define UART_TIMEOUT 100
#define PACKET_MIN_LENGTH 6
#define SERVER_IP "13.233.25.158"
#define SERVER_PORT 3000
#define SOCKET_INDEX 0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendTask */
osThreadId_t sendTaskHandle;
const osThreadAttr_t sendTask_attributes = {
  .name = "sendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for receiveTask */
osThreadId_t receiveTaskHandle;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sl_send_lock */
osMutexId_t sl_send_lockHandle;
const osMutexAttr_t sl_send_lock_attributes = {
  .name = "sl_send_lock"
};
/* Definitions for dl_send_lock */
osMutexId_t dl_send_lockHandle;
const osMutexAttr_t dl_send_lock_attributes = {
  .name = "dl_send_lock"
};
/* Definitions for ph_send_lock */
osMutexId_t ph_send_lockHandle;
const osMutexAttr_t ph_send_lock_attributes = {
  .name = "ph_send_lock"
};
/* Definitions for uart_lock */
osMutexId_t uart_lockHandle;
const osMutexAttr_t uart_lock_attributes = {
  .name = "uart_lock"
};
/* USER CODE BEGIN PV */
char txBuffer[RX_BUFFER_SIZE];        // Buffer for sending AT commands
char rxBuffer[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
char checkBuffer[RX_BUFFER_SIZE];

uint16_t writeIndex = 0;  // Updated by DMA
uint16_t readIndex = 0;   // Updated by application
uint8_t propertyIndex = 0;

typedef enum {
    IMMOBILIZE_STATUS = 0x01,
    RPM_PRESET        = 0x02,
    GPS               = 0x03,
    CURRENT           = 0x04,
    VOLTAGE           = 0x05,
    RPM               = 0x06,
    TEMPERATURE       = 0x07,
    NETWORK_STRENGTH  = 0x08
} ServerPropertyType;


typedef struct {
    uint8_t immobilizeStatus[1]; // 1 byte
    uint8_t rpmPreset[1];        // 1 byte
    uint8_t gpsData[32];          // 6 bytes
    uint8_t currentData[2];      // 2 bytes
    uint8_t voltageData[2];      // 2 bytes
    uint8_t rpm[1];              // 1 byte
    uint8_t temperature[1];      // 1 byte
    uint8_t networkStrength[1];  // 1 byte
} serverProperties;

serverProperties serverAttributes;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartSendTask(void *argument);
void StartReceiveTask(void *argument);
void StartGpsTask(void *argument);

/* USER CODE BEGIN PFP */
void NetworkInit();
void OpenSocket();
void SocketSendData(void);
void SocketReceiveData(void);
void HandleReceivedData(uint8_t writeIndex);
uint8_t encodeServerData(ServerPropertyType type, uint8_t *packet);
void decodeServerData(uint8_t *packet, uint8_t length);
void gps(void);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize SIM7600
  NetworkInit();

  // Open a socket
  OpenSocket();

  serverAttributes.immobilizeStatus[0] = 0x01;
  serverAttributes.rpmPreset[0] = 0x64;
  memset(serverAttributes.gpsData, 0x00, sizeof(serverAttributes.gpsData));
  serverAttributes.currentData[0] = 0x12;
  serverAttributes.currentData[1] = 0x34;
  serverAttributes.voltageData[0] = 0x56;
  serverAttributes.voltageData[1] = 0x78;
  serverAttributes.rpm[0] = 0x32;
  serverAttributes.temperature[0] = 0x20;
  serverAttributes.networkStrength[0] = 0x05;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of sl_send_lock */
  sl_send_lockHandle = osMutexNew(&sl_send_lock_attributes);

  /* creation of dl_send_lock */
  dl_send_lockHandle = osMutexNew(&dl_send_lock_attributes);

  /* creation of ph_send_lock */
  ph_send_lockHandle = osMutexNew(&ph_send_lock_attributes);

  /* creation of uart_lock */
  uart_lockHandle = osMutexNew(&uart_lock_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of sendTask */
  sendTaskHandle = osThreadNew(StartSendTask, NULL, &sendTask_attributes);

  /* creation of receiveTask */
  receiveTaskHandle = osThreadNew(StartReceiveTask, NULL, &receiveTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void NetworkInit() {

	strcpy(txBuffer, "ATE0\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, '\0' , sizeof(txBuffer));

	memset(rxBuffer, '\0' , sizeof(rxBuffer));
	HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Check SIM is ready
    strcpy(txBuffer, "AT+CPIN?\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    memset(rxBuffer, '\0' , sizeof(rxBuffer));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Check network registration
    strcpy(txBuffer, "AT+CREG=2\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    memset(rxBuffer, '\0' , sizeof(rxBuffer));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // PDP Contextn
	strcpy(txBuffer, "AT+CGACT=1,1\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, '\0' , sizeof(txBuffer));

	memset(rxBuffer, '\0' , sizeof(rxBuffer));
	HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);

    // Set APN
    strcpy(txBuffer, "AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    memset(rxBuffer, '\0' , sizeof(rxBuffer));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Set GPS Mode
	strcpy(txBuffer, "AT+CGPS=1\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, '\0' , sizeof(txBuffer));

	memset(rxBuffer, '\0' , sizeof(rxBuffer));
	HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);


    // Start TCP/IP service
    strcpy(txBuffer, "AT+NETOPEN\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    memset(rxBuffer, '\0' , sizeof(rxBuffer));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);

    // Check response
    if (strstr(rxBuffer, "+NETOPEN: 0") == NULL) {
        //Error_Handler();
    }
}

void OpenSocket() {
    sprintf(txBuffer, "AT+CIPOPEN=%d,\"TCP\",\"%s\",%d\r\n", SOCKET_INDEX, SERVER_IP, SERVER_PORT);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    memset(rxBuffer, '\0' , sizeof(rxBuffer));
    HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxBuffer, 256);
    memset(rxBuffer, '\0' , sizeof(rxBuffer));


    //memset(txBuffer, '\0' , sizeof(txBuffer));
    // Check response
    if (strstr(rxBuffer, "+CIPOPEN: 0,0") == NULL) {
        //Error_Handler();
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuffer, RX_BUFFER_SIZE);

    // Enable UART IDLE line detection interrupt
    //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}


void SocketSendData(void) {
	uint8_t data[40];

	encodeServerData(propertyIndex, data);

	osMutexAcquire(uart_lockHandle, osWaitForever);

	sprintf(txBuffer, "AT+CIPSEND=%d,%d\r\n", SOCKET_INDEX, sizeof(data));

    // Wait for `>` prompt
    while (!strstr((char *)checkBuffer, ">")) {

    	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    	osDelay(10);  // Wait for the response
    }
    memset(txBuffer, '\0' , sizeof(txBuffer));

    // Send data
    HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    osMutexRelease(uart_lockHandle);

    if (strstr(rxBuffer, "SEND OK") == NULL) {
        //Error_Handler();
    }
    if(++propertyIndex > 8)	propertyIndex  = 0;
}

void SocketReceiveData(void) {
	int length = sizeof(rxBuffer);

	osMutexAcquire(uart_lockHandle, osWaitForever);

    sprintf(txBuffer, "AT+CIPRXGET=3,%d,%d\r\n", SOCKET_INDEX, length);

    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, '\0' , sizeof(txBuffer));

    osMutexRelease(uart_lockHandle);

    // Check if data received successfully
    if (strstr(rxBuffer, "+CIPRXGET:") == NULL) {
        //Error_Handler();
    }
}

void gps(void) {
	char *gpsString;
	osMutexAcquire(uart_lockHandle, osWaitForever);

	strcpy(txBuffer, "AT+CGPSINFO\r\n");

	while (!strstr((char *)checkBuffer, "+CGPSINFO:")) {

	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	osDelay(10);  // Wait for the response
	}

	gpsString = strstr((char *)checkBuffer, "+CGPSINFO:");

	memset(txBuffer, '\0' , sizeof(txBuffer));

	// Move pointer past "+CGPSINFO:"
	gpsString += 10;

	// Example response: 3113.343286,N,12121.234064,E,...
	char latitude[16] = {0};
	char longitude[16] = {0};

	// Extract latitude and longitude strings
	char *token = strtok(gpsString, ",");
	if (token != NULL) strncpy(latitude, token, sizeof(latitude) - 1);
	token = strtok(NULL, ","); // Skip N/S indicator
	token = strtok(NULL, ",");
	if (token != NULL) strncpy(longitude, token, sizeof(longitude) - 1);

	// Format and store in gpsData
	sprintf((char *)serverAttributes.gpsData, "%s,%s", latitude, longitude);

	osMutexRelease(uart_lockHandle);
}

void HandleReceivedData(uint8_t writeIndex) {

	uint16_t newDataCount = (writeIndex >= readIndex)
	                            ? (writeIndex - readIndex)
	                            : (RX_BUFFER_SIZE - readIndex + writeIndex);
	memset(checkBuffer, '\0', RX_BUFFER_SIZE);
	for (uint16_t i = 0; i < newDataCount; i++) {
		// Copy new data to the process buffer
		uint8_t newByte = rxBuffer[readIndex];
		checkBuffer[i] = newByte;

		// Increment read index circularly
		readIndex = (readIndex + 1) % RX_BUFFER_SIZE;
	}
	// Check if we have a complete packet
	if (readIndex >= PACKET_MIN_LENGTH) { // Assume minimum length is 2 bytes (Type + Length)
		if (checkBuffer[0] == 0xAA) {
			decodeServerData((uint8_t *)checkBuffer, readIndex);
		}
	}
}


uint8_t encodeServerData(ServerPropertyType type, uint8_t *packet) {
    uint8_t payloadLength = 0;
    uint8_t *payload;

    switch (type) {
        case IMMOBILIZE_STATUS:
            payload = serverAttributes.immobilizeStatus;
            payloadLength = sizeof(serverAttributes.immobilizeStatus);
            break;
        case RPM_PRESET:
            payload = serverAttributes.rpmPreset;
            payloadLength = sizeof(serverAttributes.rpmPreset);
            break;
        case GPS:
            payload = serverAttributes.gpsData;
            payloadLength = sizeof(serverAttributes.gpsData);
            break;
        case CURRENT:
            payload = serverAttributes.currentData;
            payloadLength = sizeof(serverAttributes.currentData);
            break;
        case VOLTAGE:
            payload = serverAttributes.voltageData;
            payloadLength = sizeof(serverAttributes.voltageData);
            break;
        case RPM:
            payload = serverAttributes.rpm;
            payloadLength = sizeof(serverAttributes.rpm);
            break;
        case TEMPERATURE:
            payload = serverAttributes.temperature;
            payloadLength = sizeof(serverAttributes.temperature);
            break;
        case NETWORK_STRENGTH:
            payload = serverAttributes.networkStrength;
            payloadLength = sizeof(serverAttributes.networkStrength);
            break;
        default:
            return 0; // Unknown type
    }

    // Create the packet
    uint8_t index = 0;
    packet[index++] = 0xAA;  // Header byte 1
    packet[index++] = 0xBB;  // Header byte 2
    packet[index++] = type;  // Property type
    packet[index++] = payloadLength; // Payload length

    // Copy payload
    memcpy(&packet[index], payload, payloadLength);
    index += payloadLength;

    // Add checksum
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < index; i++) {
        checksum ^= packet[i];
    }
    packet[index++] = checksum;

    return index; // Total packet length
}

void decodeServerData(uint8_t *packet, uint8_t length) {
    if (length < 5) return; // Invalid packet length

    // Validate header
    if (packet[0] != 0xAA || packet[1] != 0xBB) return;

    // Extract type and payload length
    ServerPropertyType type = packet[2];
    uint8_t payloadLength = packet[3];

    // Validate checksum
   // uint8_t checksum = 0;
   // for (uint8_t i = 2; i < 4 + payloadLength; i++) {
   //     checksum ^= packet[i];
   // }
   // if (checksum != packet[4 + payloadLength]) return;

    // Extract payload
    uint8_t *payload = &packet[4];

    // Update serverAttributes
    switch (type) {
        case IMMOBILIZE_STATUS:
            memcpy(serverAttributes.immobilizeStatus, payload, payloadLength);
            break;
        case RPM_PRESET:
            memcpy(serverAttributes.rpmPreset, payload, payloadLength);
            break;
        case GPS:
            memcpy(serverAttributes.gpsData, payload, payloadLength);
            break;
        case CURRENT:
            memcpy(serverAttributes.currentData, payload, payloadLength);
            break;
        case VOLTAGE:
            memcpy(serverAttributes.voltageData, payload, payloadLength);
            break;
        case RPM:
            memcpy(serverAttributes.rpm, payload, payloadLength);
            break;
        case TEMPERATURE:
            memcpy(serverAttributes.temperature, payload, payloadLength);
            break;
        case NETWORK_STRENGTH:
            memcpy(serverAttributes.networkStrength, payload, payloadLength);
            break;
        default:
            // Unknown type
            break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    //if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
      //  __HAL_UART_CLEAR_IDLEFLAG(&huart1);  // Clear the idle flag

	// Process received data
	writeIndex = Size;//(RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx));// % RX_BUFFER_SIZE;
	HandleReceivedData(writeIndex);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSendTask */
/**
* @brief Function implementing the sendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendTask */
void StartSendTask(void *argument)
{
  /* USER CODE BEGIN StartSendTask */
  /* Infinite loop */
  for(;;)
  {
	  SocketSendData();
	  osDelay(20);
  }
  /* USER CODE END StartSendTask */
}

/* USER CODE BEGIN Header_StartReceiveTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveTask */
void StartReceiveTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveTask */
  /* Infinite loop */
  for(;;)
  {
	  SocketReceiveData();
	  osDelay(20);
  }
  /* USER CODE END StartReceiveTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  /* Infinite loop */
  for(;;)
  {
	gps();
    osDelay(20);
  }
  /* USER CODE END StartGpsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
