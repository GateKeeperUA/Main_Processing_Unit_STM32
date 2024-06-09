/*
 *  MAIN FILE
 *  Author: GateKeeper
 *  Copyright (C) 2024
*/

/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "bme680.h"
#include "bme680_defs.h"
#include "http_ssi.h"
#include "lwip/apps/httpd.h"
#include "pn532.h"
#include "pn532_stm32f1.h"
#include "udpClientRAW.h"
#include "IPs.h"
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// BME680 Variables
int8_t rslt = BME680_OK;
int IAQ;
struct bme680_dev gas_sensor;
int gas_target = 30000;
int BME_Presence = 0; // Means BME is off


// RFID PN532 Variables
PN532 pn532;
struct bme680_field_data data; // Sampling results variable
uint8_t buff[255];
int32_t uid_len = 0;
uint8_t uid[MIFARE_UID_MAX_LENGTH];
uint8_t key_a[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t pn532_error = PN532_ERROR_NONE;
int RFID_Presence = 0; // Means RFID reader is off
int TAG_Read = 0;

// Server Status Variables
int client_connected = 0;
int message_type;
struct udp_pcb *upcb;
int emergencyFlag = 0;
int People_Counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

// BME680 Forward Declarations
int8_t bme680I2cRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme680I2cWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

// Functions Declarations
void BME680_Initialize();
void RFID_PN532_Initialize();
void Open_Door(int emergency, int InOut);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_LWIP_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  extern struct netif gnetif;
  http_server_init();
  HAL_TIM_Base_Start_IT(&htim1);


  // ----------------------- DEVICES CONFIG START -----------------------
  BME680_Initialize();
  RFID_PN532_Initialize();
  int HTML_Server_ON = 1;
  // ------------------------ DEVICES CONFIG END ------------------------

  // ------------------ IP WEBSITE CONFIG START -------------------
  if (HTML_Server_ON) {
	  printf("\n\rActual board IP:");
	  printf("\n\rIPv4: %d.%d.%d.%d",ipv4_address[0],ipv4_address[1],ipv4_address[2],ipv4_address[3]);
	  printf("\n\rMask: %d.%d.%d.%d",mask_address[0],mask_address[1],mask_address[2],mask_address[3]);
	  printf("\n\rGateway: %d.%d.%d.%d\n\r",gateway_address[0],gateway_address[1],gateway_address[2],gateway_address[3]);
	  do {
		  ethernetif_input(&gnetif);
		  MX_LWIP_Process();
	  } while(changed_ip_status == 0);
	  printf("\n\rNew board IP:");
	  printf("\n\rIPv4: %d.%d.%d.%d",ipv4_address[0],ipv4_address[1],ipv4_address[2],ipv4_address[3]);
	  printf("\n\rMask: %d.%d.%d.%d",mask_address[0],mask_address[1],mask_address[2],mask_address[3]);
	  printf("\n\rGateway: %d.%d.%d.%d\n\r",gateway_address[0],gateway_address[1],gateway_address[2],gateway_address[3]);
	  // -------------------- IP WEBSITE CONFIG END --------------------
	  netif_remove(&gnetif);
	  MX_LWIP_Init();
	  udpClient_connect();
  }

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
	  ethernetif_input(&gnetif);
	  sys_check_timeouts();
	  if(RFID_Presence == 1 || client_connected == 1) {
		uid_len = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 1000);
		if (uid_len != PN532_STATUS_ERROR) {
			// Uncomment below if want to read UID without the memory blocks
			/*printf("Found card with UID: ");
			  memset(message,0,keyLen);
			  for (uint8_t i = 0; i < uid_len; i++) {
				printf("%02x ", uid[i]);
				sprintf(message,"%s%02x",message,uid[i]);
			  }*/
		  memset(message, 0, keyLen);
		  for (uint8_t block_number = 0; block_number < 1; block_number++) {
			pn532_error = PN532_MifareClassicAuthenticateBlock(&pn532, uid, uid_len, block_number, MIFARE_CMD_AUTH_A, key_a);
			if (pn532_error != PN532_ERROR_NONE) break;
			pn532_error = PN532_MifareClassicReadBlock(&pn532, buff, block_number);
			if (pn532_error != PN532_ERROR_NONE) break;
			if ((pn532_error != 0x02) &&  (buff[0] != 0x00 && buff[1] != 0x00)) {	// Verifies if there were not any errors during the block reading
				TAG_Read = 1;	// A tag is being read
				printf("%d: ", block_number);
			    for (uint8_t i = 0; i < 16; i++) {
				  printf("%02x ", buff[i]);
				  sprintf(message,"%s%02x",message,buff[i]);
			    }
			    printf("\r\n");
			} else {
				break;
			}
		  }
		  message_type = 1;	// Message will be sent has ID Information
		  if (TAG_Read == 1) {
			  if (HTML_Server_ON) udpClient_send();
			  RFID_Received = 0;	// It will wait for a reply from the server
			  HAL_Delay(1000);
			  TAG_Read = 0;	// A new card is now ready to be read
		  }
		}
	  }

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x6000030D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 43200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 21600000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 43200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 49463-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 25500-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 43200-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback

  if (htim == &htim1 && BME_Presence == 1)	// 10 sec timer to read BME data
  {
	// Query the sample data
	rslt = bme680_get_sensor_data(&data, &gas_sensor);

	printf("\n\r----------Data read----------");
	printf("\n\rTemperature:    %.2fºC", data.temperature / 100.0f);
	printf("\n\rAtm. Pressure:  %.2f hPa", data.pressure / 100.0f);
	printf("\n\rHumidity:       %.2f %%", data.humidity / 1000.0f );
	printf("\n\rGas Resistance: %ld ohms", data.gas_resistance);

	IAQ = CalculateIAQ(data.temperature,data.humidity,data.gas_resistance);

	if (IAQ <= 2 && client_connected == 1) {	// IAQ less than 3 is a bad air quality, mandatory to trigger the Emergency System
		sprintf(message,"9Emergency\0");
		udpClient_send();
	}

	printf("\n\rAir Quality:    %d", IAQ);
	printf("\n\r-----------------------------\n\r");
	sprintf(message,"2%d %d %d %d %d \0",data.temperature,data.pressure,data.humidity,IAQ,People_Counter);
	//printf("\n\rMessage to send: %s\n\r",message);
	if (client_connected == 1) {
		message_type = 2;
		udpClient_send();
	}

	// Request the next sample
	if (gas_sensor.power_mode == BME680_FORCED_MODE) {
	  rslt = bme680_set_sensor_mode(&gas_sensor);
	}

  }

  if(htim == &htim2) {	// 100 msec timer to read any Emergency situation that can be sent by the server
	  receiver();
  }

  if(htim == &htim3) {	// 5 sec timer to close the door after it has been opened
	  // Close door and restore the normal state
	  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
	  HAL_TIM_Base_Stop(&htim3);
  }

  if(htim == &htim4) {	// 15 sec timer to cancel the Emergency System
	  // Cancel Emergency and restore the normal state
	  emergencyFlag = 0;	// Disable Emergency Flag
	  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
  	  HAL_TIM_Base_Stop(&htim4);
  }

  if(htim == &htim5) {	// 5 sec timer to turn off the Not Access Led
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);
	  HAL_TIM_Base_Stop(&htim5);
  }
}

int8_t bme680I2cRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  int8_t result;

  if (HAL_I2C_Master_Transmit(&hi2c1, (dev_id << 1), &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK) {
	result = -1;
  } else if (HAL_I2C_Master_Receive (&hi2c1, (dev_id << 1) | 0x01, reg_data, len, HAL_MAX_DELAY) != HAL_OK) {
	result = -1;
  } else {
	result = 0;
  }

  return result;
}

int8_t bme680I2cWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  int8_t result;
  int8_t *buf;

  // Allocate and load I2C transmit buffer
  buf = malloc(len + 1);
  buf[0] = reg_addr;
  memcpy(buf + 1, reg_data, len);

  if (HAL_I2C_Master_Transmit(&hi2c1, (dev_id << 1), (uint8_t *) buf, len + 1, HAL_MAX_DELAY) != HAL_OK) {
	result = -1;
  } else {
	result = 0;
  }

  free(buf);
  return result;
}

void BME680_Initialize() {
  // Configure the BME680 driver
  gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
  gas_sensor.intf = BME680_I2C_INTF;
  gas_sensor.read = bme680I2cRead;
  gas_sensor.write = bme680I2cWrite;
  gas_sensor.delay_ms = HAL_Delay;
  gas_sensor.amb_temp = 25;

  // Initialize the driver
  if (bme680_init(&gas_sensor) != BME680_OK) {
	char bme_msg[] = "\n\rBME680 Initialization Error\r\n";
	printf("%s", bme_msg);
  } else {
	char bme_msg[] = "\n\rBME680 Initialized and Ready\r\nCalibration Startup\r\n";
	printf("%s", bme_msg);
  }

  // Select desired oversampling rates
  gas_sensor.tph_sett.os_hum = BME680_OS_2X;
  gas_sensor.tph_sett.os_pres = BME680_OS_4X;
  gas_sensor.tph_sett.os_temp = BME680_OS_8X;
  gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

  gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
  gas_sensor.gas_sett.heatr_temp = 320;
  gas_sensor.gas_sett.heatr_dur = 150;

  // Set sensor to "always on"
  gas_sensor.power_mode = BME680_FORCED_MODE;

  // Set oversampling settings
  static uint8_t required_settings = (BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL);
  rslt = bme680_set_sensor_settings(required_settings, &gas_sensor);

  // Set sensor mode
  rslt = bme680_set_sensor_mode(&gas_sensor);

  // Query minimum sampling period
  static uint16_t min_sampling_period;
  bme680_get_profile_dur(&min_sampling_period, &gas_sensor);

  // BME preparation and calibration in order to heat up the module to read the right values
  int readings = 100;
  for (int i = 1; i <= readings + 3; i++) {
	if (gas_sensor.power_mode == BME680_FORCED_MODE) bme680_set_sensor_mode(&gas_sensor);
	bme680_get_sensor_data(&data, &gas_sensor);
	//printf("\r\nValue %d : %ld",i,data.gas_resistance);
	if(i > 3 && data.gas_resistance > gas_target) {
		break;
	}
	HAL_Delay(100);
  }
  printf("\n\rBME Initialized\n\r");
  BME_Presence = 1;
}

void RFID_PN532_Initialize() {
  // Configure RFID Driver
  PN532_I2C_Init(&pn532);
  PN532_GetFirmwareVersion(&pn532, buff);
  if (PN532_GetFirmwareVersion(&pn532, buff) == PN532_STATUS_OK) {
	printf("\n\rFound PN532 with firmware version: %d.%d", buff[1], buff[2]);
  } else {
	return;
  }
  PN532_SamConfiguration(&pn532);
  printf("\n\rPN532 Initialized");
  printf("\n\rWaiting for RFID/NFC card...\r\n");
  RFID_Presence = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	// Interrupt for when C13 button is pressed
{
    if (GPIO_Pin == GPIO_PIN_13) {
	  Open_Door(0,1);	// Meaning it is exiting the room (no emergency)
    }
}

void Open_Door(int emergency, int InOut) {
	// Open the door
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);

	if(emergency == 0 && emergencyFlag == 0 && InOut == 0) {	// Normal opening of door from outside to inside
		TIM3 -> CNT = 0;
		HAL_TIM_Base_Start(&htim3);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);
		// Turn on outside Green LED meaning the person can enter the room
	}
	else if (emergency == 1) {	// Emergency opening of door
		emergencyFlag = 1;
		TIM4 -> CNT = 0;
		HAL_TIM_Base_Start(&htim4);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);
		// Turn on outside Green LED meaning the person can enter the room
	}

	if (InOut == 0) { 			// 0 - In
		People_Counter++;
	} else if (InOut == 1) { 	// 1 - Out
		People_Counter--;
		TIM3 -> CNT = 0;
		HAL_TIM_Base_Start(&htim3);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);
		// Turn on inside Green LED meaning the person will exit the room
	} else {
		People_Counter = 0;		// 2 - Emergency (all people exit the room)
	}
	//printf("\n\rPeople Counter: %d\n\r", People_Counter);
}

/* USER CODE END 4 */

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
