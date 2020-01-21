/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

volatile  uint16_t adcBuffer[14]; // Buffer for store the results of the ADC conversion
double adc_ch[12];

uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];

uint32_t TIMER1;
volatile u32  SysTickDelay; 
unsigned int timer_INIT_FAPCH1;

u32 FLAG_T1;
u32 FLAG_T2;
u8 FLAG_FAPCH_ON;

uint8_t RX_uBUF[1];


unsigned int timer_DMA2;
u8 flag_pachka_TXT; //
uint16_t  text_lengh;
uint8_t text_buffer[Bufer_size];

volatile char          rx_buffer1[RX_BUFFER_SIZE1];
volatile unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
volatile u8  rx_buffer_overflow1;


char sr[BUFFER_SR+1];
unsigned char lsr;
unsigned char lk;
unsigned int led_tick;


char  strng[buf_IO];
char      InOut[BUFFER_SR+1];
char      Word [buf_Word];    //
char DATA_Word [buf_DATA_Word];    //
char DATA_Word2[buf_DATA_Word];    //
 
char Master_flag; // 
char lsym;
char  sym;
char flag;
char    NB;
char Adress;  //
char packet_sum;
char crc,comanda;
      
char In_data[BUF_STR];
char ink1; //
char data_in;

u16 lenght;
u16 SCH_LENGHT_PACKET;
	
unsigned     int index1;
unsigned     char crc_ok;
unsigned     char packet_ok;
unsigned     char packet_flag;
unsigned     int indexZ; 
unsigned     int index_word;
unsigned     int index_data_word;
unsigned     int index_data_word2;
unsigned     int lenght_data;//
unsigned     char data_flag;
unsigned     char data_flag2;
unsigned     char FLAG_lenght;//
unsigned     int sch_lenght_data;
unsigned     char FLAG_DATA;
unsigned char FLAG_CW;
float time_uart; //
unsigned char flag_pcf;
char lsym1;
char pack_ok1;
char pack_sum1;
char sym1;


u32 sch_rx_byte;
  
u8  Adress_ATT1;
u8  Adress_ATT2;
u8  Adress_ATT3;
u8  Adress_ATT4;
  
u8  Adress_SWITCH;
u8  Adress_KONTROL_WIRE;
u8  Adress_KONTROL_1;
u8  Adress_KONTROL_2;

u8 PWRDN_DAC1;
u8 PWRDN_DAC2;
u8 PWRDN_ADC1;
u8 PWRDN_ADC2;

u8 RESET_DAC1;
u8 RESET_DAC2;
u8 RESET_ADC1;
u8 RESET_ADC2;

u8 EVENT_INT0=0;
u8 EVENT_INT1=0;
u8 EVENT_INT2=0;
u8 EVENT_INT3=0;
u8 EVENT_INT4=0;
u8 EVENT_INT5=0;
u8 EVENT_INT6=0;
u8 EVENT_INT7=0;
u8 EVENT_INT8=0;

u32 TIME_SYS;
u32 TIME_TEST;

//  описание структур управления и квитанций

 Frame INVOICE[quantity_SENDER];//структура квитанций о состоянии дел, по числу потенциальный адресатов
 Frame FRM1[quantity_SENDER];	//принятый фрейм, по числу потенциальный адресатов
 SERVER SERV1;					//структура "Хранилище"
 ID_SERVER ID_SERV1;			//структура указатель для "Хранилище"
 CMD_RUN CMD1;
 ADR_SENDER ADDR_SNDR;			//структура хранит массив адресов ОТПРАВИТЕЛЕЙ 
 
 u32 ID_CMD=0;  //переменная хранить текущий ID наших квитанций 

extern reg_DAC37j82 dac1;
extern reg_DAC37j82 dac2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 12;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;//04-09-2019 было SPI_BAUDRATEPRESCALER_4
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 256000;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC8 */
  GPIO_InitStruct.Pin   = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE9 */
  GPIO_InitStruct.Pin   = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE10 PE11 PE12 
                           PE13 PE14 PE15 */
  GPIO_InitStruct.Pin   =   GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin   = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin  = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins :  PD3 PD4 PD5 
                           PD6 PD7 */
  GPIO_InitStruct.Pin  = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin  = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin   = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  //-----------exti-------------------------
   /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin  = GPIO_PIN_2;   //прерывание от ETH2
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin  = GPIO_PIN_7;   //прерывание от ETH1
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin  = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Delay( unsigned int Val)  
{  
   SysTickDelay = Val;  
   while (SysTickDelay != 0) {};  
}
//----------------------------------------------
volatile void delay_us( uint32_t time_delay)
{	
	time_delay=time_delay*10;
    while(time_delay--)	;
} 
//-------------------------------------------
unsigned int leng ( char *s)
{
  unsigned  char i=0;
  while ((s[i]!='\0')&&(i<120)) { i++;}
  return i;
}

void Transf(char* s)  // процедура отправки строки символов в порт
{
  unsigned  short l=0;
  unsigned  short i=0;
         
  if ((flag_pachka_TXT==0) )
  {
    l=strlen(s);
    if ((text_lengh+l)>Bufer_size-5) text_lengh=0u;
    for (i=text_lengh;i<(text_lengh+l);i++) text_buffer[i]=s[i-text_lengh];
    text_lengh=text_lengh+l;
  } 
}

void itoa(int val,  char *bufstr, int base) //
{
    u8 buf[32] = {0};
    int i = 30;
    int j;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    i++; j=0;
    while (buf[i]!=0){ bufstr[j]=buf[i]; i++; j++;}
}

void f_out (char s[],double a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%.2f",a);
   Transf(strng);
   Transf ("\r\n");
}

void d_out (char s[],int a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%d",a);
   Transf(strng);
   Transf ("\r");
}

void u_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   Transf ("\r");
}

void un_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
}

void nu_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   //Transf ("\r\n");
}

void x32_out (char s[],u32 a)
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void x_out (char s[],u32 a)//было u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   if (a<10) Transf("0x0"); else Transf("0x");
   Transf(strng);
   Transf ("\r\n");
}

void xn_out (char s[],u32 a)//было u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   if (a<10) Transf("0x0"); else Transf("0x");
   Transf(strng);   
}

void in_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);   
}

void i_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);
   Transf ("\r\n");   
}

void hn_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);   
}

void h_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);  
   Transf ("\r\n");   
}


void UART_IT_TX (void)
{
 uint16_t k;

if ((flag_pachka_TXT==0)&&(text_lengh>1u))
{ 
    k = text_lengh;
  	HAL_UART_Transmit_IT(&huart1,text_buffer,k);
    text_lengh=0u;  //обнуление счётчика буфера 
    flag_pachka_TXT=1; //устанавливаем флаг передачи
  }
}
 
void UART_DMA_TX (void)
{
 uint16_t k;

if (HAL_UART_GetState(&huart1)!=HAL_UART_STATE_BUSY_TX )
{
	if ((flag_pachka_TXT==0)&&(text_lengh>1u)&(timer_DMA2>250))
	 {
		k = text_lengh;
		HAL_UART_Transmit_DMA(&huart1,text_buffer,k);
		text_lengh=0u;  //обнуление счётчика буфера 
		flag_pachka_TXT=1; //устанавливаем флаг передачи
	  }
  }	
} 

void spisend32 (u32 d) //32 бита
{
	u8 a1;
	u8 a2;
	u8 a3;
	u8 a4;
	
	u8 b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d >> 24)&0xff;
  a2 = (d >> 16)&0xff;
  a3 = (d >>  8)&0xff;
  a4 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a2, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a3, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a4, &b,1, 5000);  
}

u8 spisend8 (u8 d) //8 бит
{
	u8 a1;
	u8  b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000); 
  return b; 
}

void spisend_FPGA (u8 adr,u8 d) //8 бит
{  
   CS_FPGA1_0;
   Delay(1);
   
   Transf("." );
   
   spisend8(adr|0x80);//устанавливаем бит записи
   spisend8(d);
         
   Delay(1);  
   CS_FPGA1_1;
   Delay(1); 
}



void init_FAPCH (u8 a)
{
	if (a==1)
	{
	 Transf("\r\n" );
	 Transf("-----------------------------------\r\n" );
	 Transf("Пришла команда активации кассеты!!!\r\n" );
	 Transf("Программирую ФАПЧ:\r" );
     
	 Transf("...\r" );
	 Transf("Выполненно!\r" );
	 Transf("\r\n" );	
	}
}

u32 FPGA2_wSPI (u8 size,u8 adr,u64 data)
{
   u8 d[8];
   u8 i,k;
   
   k=size/8;
    
   if (k==1)  d[3]=data;
   
   if (k==2) {d[2]=data;
        d[3]=data>>8;
        }
        
   if (k==3) {d[1]=data;
        d[2]=data>>8;
        d[3]=data>>16;
        }
		
   if (k==4) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
        }
		
   if (k==5) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
        }
		
	if (k==6) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
        }
	if (k==7) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
        }
	if (k==8) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
		d[7]=data>>56;
        }
  
   CS_FPGA2_0;
//   delay_us(1);
 
   spisend8 (adr|0x80); //передаём адресс и устанавливаем бит записи
   for (i=0;i<(size/8);i++)  spisend8 (d[3-i]);  //считываем данные
   
//   delay_us(1);  
   CS_FPGA2_1;

   return 0;
} 

u32 FPGA_wSPI (u8 size,u8 adr,u64 data)
{
   u8 d[8];
   u8 i,k;
   
   k=size/8;
    
   if (k==1)  d[3]=data;
   
   if (k==2) {d[2]=data;
        d[3]=data>>8;
        }
        
   if (k==3) {d[1]=data;
        d[2]=data>>8;
        d[3]=data>>16;
        }
		
   if (k==4) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
        }
		
   if (k==5) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
        }
		
	if (k==6) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
        }
	if (k==7) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
        }
	if (k==8) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
		d[7]=data>>56;
        }
  
   FPGA_CS_0;
//   delay_us(1);
 
   spisend8 (adr|0x80); //передаём адресс
   //for (i=0;i<(size/8);i++)  spisend8 (d[3-i]);  //считываем данные  - НЕ РАБОТАЕТ ДЛЯ ДАННЫХ БОЛЬШЕ 32 БИТ!!!
   k--;
   for (i=0;i<(size/8);i++) spisend8 (d[k-i]); //пишем данные
   
//   delay_us(1);  
   FPGA_CS_1;

   return 0;
}  
  
u64 FPGA2_rSPI (u8 size,u8 adr)
{
   u64 d[8];
   u8 i,k;
   u8 adr_a=0;
   u64 value;
   
   k=size/8;
   adr_a=adr;
  
   CS_FPGA2_0;
//   delay_us(1);
 
   spisend8 (adr_a); //
   for (i=0;i<(size/8);i++) d[i] = spisend8 (0);  //считываем данные
//   delay_us(1);  
   CS_FPGA2_1;
   
   if (k==1) value =   d[0];
   if (k==2) value =  (d[0]<< 8)+ d[1];
   if (k==3) value =  (d[0]<<16)+(d[1]<< 8)+ d[2];
   if (k==4) value =  (d[0]<<24)+(d[1]<<16)+(d[2]<< 8)+ d[3];
   if (k==5) value =  (d[0]<<32)+(d[1]<<24)+(d[2]<<16)+(d[3]<< 8)+ d[4];
   if (k==6) value =  (d[0]<<40)+(d[1]<<32)+(d[2]<<24)+(d[3]<<16)+(d[4]<< 8)+ d[5];
   if (k==7) value =  (d[0]<<48)+(d[1]<<40)+(d[2]<<32)+(d[3]<<24)+(d[4]<<16)+(d[5]<< 8)+ d[6];
   if (k==8) value =  (d[0]<<56)+(d[1]<<48)+(d[2]<<40)+(d[3]<<32)+(d[4]<<24)+(d[5]<<16)+(d[6]<<8)+d[7];

   return value;
}
  
  
u64 FPGA_rSPI (u8 size,u8 adr)
{
   u64 d[8];
   u8 i,k;
   u8 adr_a=0;
   u64 value;
   
   k=size/8;
   adr_a=adr;
  
   FPGA_CS_0;
//   delay_us(1);
 
   spisend8 (adr_a); //
   for (i=0;i<(size/8);i++) d[i] = spisend8 (0);  //считываем данные
//   delay_us(1);  
   FPGA_CS_1;
   
   if (k==1) value =   d[0];
   if (k==2) value =  (d[0]<< 8)+ d[1];
   if (k==3) value =  (d[0]<<16)+(d[1]<< 8)+ d[2];
   if (k==4) value =  (d[0]<<24)+(d[1]<<16)+(d[2]<< 8)+ d[3];
   if (k==5) value =  (d[0]<<32)+(d[1]<<24)+(d[2]<<16)+(d[3]<< 8)+ d[4];
   if (k==6) value =  (d[0]<<40)+(d[1]<<32)+(d[2]<<24)+(d[3]<<16)+(d[4]<< 8)+ d[5];
   if (k==7) value =  (d[0]<<48)+(d[1]<<40)+(d[2]<<32)+(d[3]<<24)+(d[4]<<16)+(d[5]<< 8)+ d[6];
   if (k==8) value =  (d[0]<<56)+(d[1]<<48)+(d[2]<<40)+(d[3]<<32)+(d[4]<<24)+(d[5]<<16)+(d[6]<<8)+d[7];

   return value;
}

u32 i2c_read (u32 a)
{
	u32 ccc=0;

  ccc=(1u<<31)|(((0x5600u|(a&0xff))&0xffff)<<16)|(0x0000&0xffff);//0 - WRITE 1 - READ

  FPGA_wSPI (32,10,ccc);//считываем из блока i2c слово{бит записи|адрес регистра|байт данных} 
  Delay(10);
  ccc=FPGA_rSPI (24,11);
  return ccc;
}


u32 i2c_write (u32 a,u32 data)
{
  u32 ccc=0;

  ccc=(0u<<31)|(((0x5600u|(a&0xff))&0xffff)<<16)|(data&0xffff);//0 - WRITE 1 - READ

  FPGA_wSPI (32,10,ccc);//записываем в блок i2c {бит записи|адрес регистра|байт данных}  
  return 1;    
}

void test_spi_read(u32 k)
{
  u32 i;
  u32 var;
  u32 error=0;
  
  //UART_DMA_TX (); //отправка по DMA сообщений 
  
  for (i=0;i<k;i++) 
  { var=FPGA_rSPI(32,30);
    if (var!=0xdeedbeef) error++;
  }
  
  u_out("проведено:",k);
  Transf(" попыток\r\n");
  u_out("Число ошибок:",error);
}

void test_spi_write(u32 k)
{
  u32 i;
  u32 var;
  u32 error=0;
  
  UART_DMA_TX (); //отправка по DMA сообщений 
  
  for (i=0;i<k;i++) 
  {
    FPGA_wSPI(32,36,0xdeedbeef);//запись
    var=FPGA_rSPI(32,35);//чтение
    if (var!=0xdeedbeef) error++;//проверка
  }
  u_out("проведено:",k);
  Transf(" попыток\r\n");
  u_out("Число ошибок:",error);
}

void info ()
{

}

u8 TMP_func(void)
{
	u8 a;
	a=
	(RESET_DAC1<<7)+
	(RESET_DAC2<<6)+
	(RESET_ADC1<<5)+
	(RESET_ADC2<<4)+
	(PWRDN_DAC1<<3)+
	(PWRDN_DAC2<<2)+
	(PWRDN_ADC1<<1)+
	(PWRDN_ADC2<<0);
	return a;
}

int temp_fpga (void)
{
	u8 tmp=0;
	int Temperature=0;
	tmp=FPGA_rSPI (8,0);//скачиваем значение температуры из плис
	if ((tmp>>7)&0x01) Temperature=tmp&0x7f;
	else Temperature=tmp*(-1); 
	
	return Temperature;
}

u8 adc_init (u8 a)
{
  IO("~0 init_lmk;");    //инициализируем ФАПЧ платы
  IO("~0 ADC1_PWRDN:0;");		//включаем питание АЦП1
  IO("~0 adc1_init:0;"); 		//инициализируем   АЦП1
  IO("~0 adc1_phy_info:0;");	//проверяем состояние канала связи с АЦП1
  IO("~0 adc1_test:0;");		//рабочий режим
  IO("~0 upr_switch:0;");		//переключение ключа на сигнал из-вне
  IO("~0 upr_at1:0;");		//отключение аттенюатора по входу
  IO("~0 upr_at2:0;");    //отключение аттенюатора по входу
  IO("~0 upr_at3:0;");    //отключение аттенюатора по входу
  IO("~0 upr_at4:0;");    //отключение аттенюатора по входу
  
  return 0;
}

void SFP_read (void)
{
	u32 crc_comp =0;
	u16 crc_input=0;
	u16 i=0;
	u32 ccc=0;
	char firm[6];
	char model[12];
	u32 z=0;
	// I2C 0xA0; 
	for (i=20;i<52;i++)
	{
	  crc_comp=i;   //адресс памяти eeprom i2c A0
	  crc_input=0x0000;//пустышка
	  ccc=(1u<<31)|((0x5000u+(crc_comp&0xff))<<16)|(crc_input&0xffff);//0 - WRITE 1 - READ  0x5600
	  FPGA_wSPI (32,10,ccc);//считываем из блока i2c слово{бит записи|адрес регистра|байт данных} 
	  Delay(10);
	  crc_input=FPGA_rSPI (24,11);
	  //nu_out("sfp1 adr(",crc_comp);	 x_out("):",crc_input);
	  
	  if (i==20)  firm[0]=crc_input>>8;
	  if (i==21)  firm[1]=crc_input>>8;
	  if (i==22)  firm[2]=crc_input>>8;
	  if (i==23)  firm[3]=crc_input>>8;
	  if (i==24)  firm[4]=crc_input>>8;
				  firm[5]=0x00;
	  
	  if (i==40)  model[ 0]=crc_input>>8;
	  if (i==41)  model[ 1]=crc_input>>8;
	  if (i==42)  model[ 2]=crc_input>>8;
	  if (i==43)  model[ 3]=crc_input>>8;
	  if (i==44)  model[ 4]=crc_input>>8;
	  if (i==45)  model[ 5]=crc_input>>8;
	  if (i==46)  model[ 6]=crc_input>>8;
	  if (i==47)  model[ 7]=crc_input>>8;
	  if (i==48)  model[ 8]=crc_input>>8;
	  if (i==49)  model[ 9]=crc_input>>8;
	  if (i==50)  model[10]=crc_input>>8;
				  model[11]=0x00;
	  
	 }
	 
	Transf("SFP:"); Transf(firm);Transf(" "); Transf(model);Transf("\r\n--------------\r\n");
	  
	z=i2c_read(0);//читаем содержимое регистра 0 , I2C 0xAC;
	Transf("reg 0:\r\n");
	x_out("Loopback                 :",(z>>14)&0x01);
	x_out("Speed Selection          :",(z>> 6)&0x01);
	x_out("Auto-Negotiation Enable  :",(z>>12)&0x01);
	x_out("Power Down               :",(z>>11)&0x01);
	x_out("Isolate                  :",(z>>10)&0x01);
	x_out("Duplex Mode              :",(z>> 8)&0x01);
	x_out("Collision Test           :",(z>> 7)&0x01);
	x_out("N/A to SFP Module        :",(z>> 0)&0x1f);
	Transf("\r\n");
	
	z=i2c_read(1);//читаем содержимое регистра 1 , I2C 0xAC;
	Transf("reg 1:\r\n");
	x_out("Copper autoneg  complete :",(z>>5)&0x01);
	x_out("Copper remote fault      :",(z>>4)&0x01);
	x_out("Autonegotiation ability  :",(z>>3)&0x01);
	x_out("Copper Link status       :",(z>>2)&0x01);
	x_out("Jabber Detect            :",(z>>1)&0x01);
	x_out("Extended Capability      :",(z>>0)&0x01);
	Transf("\r\n");
	
	z=i2c_read(2);//читаем содержимое регистра 2 , I2C 0xAC;
	Transf("reg 2:\r\n");
	x_out(">",z);
	
	z=i2c_read(3);//читаем содержимое регистра 3 , I2C 0xAC;
	Transf("reg 3:\r\n");
	x_out("Model    number:",(z&0x3f)>>4);//88e1111 - 001100
	x_out("Revision number:",(z&0x0f)>>0);
	
	z=i2c_read(4);//читаем содержимое регистра 4 , I2C 0xAC;
	Transf("reg 4:\r\n");
	x_out("Remote fault             :",(z>>13)&0x01);
	x_out("PAUSE Encoding           :",(((z>>11)&0x01)<<1)+((z>>10)&0x01));
	x_out("IEEE 802.3 Selector Field:",(z>> 0)&0x0f);//always 00001
	Transf("\r\n");
	
	z=i2c_read(9);//читаем содержимое регистра 9 , I2C 0xAC;
	Transf("reg 9:\r\n");
	x_out("Transmitter Test Mode    :",(z>>13)&0x07);
	x_out("Port Type                :",(z>>10)&0x01);
	x_out("1000BASE-T Full Duplex   :",(z>> 9)&0x01);
	x_out("1000BASE-T Half Duplex   :",(z>>2) &0x01);
	Transf("\r\n");

	z=i2c_read(10);//читаем содержимое регистра 10 , I2C 0xAC;
	Transf("reg 10:\r\n");
	x_out("MASTER-SLAVE Configuration Fault     :",(z>>15)&0x01);
	x_out("MASTER-SLAVE Configuration Resolution:",(z>>14)&0x01);
	x_out("Local Receiver Status                :",(z>>13)&0x01);
	x_out("Remote Receiver Status               :",(z>>12) &0x01);
	x_out("Link Partner Full Duplex             :",(z>>11) &0x01);
	x_out("Link Partner Half Duplex             :",(z>>10) &0x01);
	x_out("Idle Error Count                     :",(z>> 0) &0xff);
	Transf("\r\n");
	
	z=i2c_read(17);//читаем содержимое регистра 17 , I2C 0xAC;
	Transf("reg 17:\r\n");
	x_out("Speed                                :",(z>>14)&0x03);
	x_out("Duplex                               :",(z>>13)&0x01);
	x_out("Page Received                        :",(z>>12)&0x01);
	x_out("Speed and Duplex Resolved            :",(z>>11)&0x01);
	x_out("Link(real time)                      :",(z>>10)&0x01);
	x_out("Cable Length                         :",(z>> 7)&0x07);
	x_out("MDI Crossover Status                 :",(z>> 6)&0x01);
	x_out("MAC Transmit Pause Enabled           :",(z>> 3)&0x01);
	x_out("MAC Receive  Pause Enabled           :",(z>> 2)&0x01);
	x_out("Polarity                             :",(z>> 1)&0x01);
	x_out("Jabber                               :",(z>> 0)&0x01);
	Transf("\r\n");
	
	z=i2c_read(18);//читаем содержимое регистра 18 , I2C 0xAC;
	Transf("reg 18:\r\n");
	x_out("Auto-Neg ERROR int en                :",(z>>15)&0x01);
	x_out("Speed changed int en                 :",(z>>14)&0x01);
	Transf("\r\n");
	
	z=i2c_read(20);//читаем содержимое регистра 20 , I2C 0xAC;
	Transf("reg 20:\r\n");
	x_out("Link down on no idles                :",(z>>15)&0x01);
	x_out("Line Loopback                        :",(z>>14)&0x01);
	
	Transf("\r\n");

	z=i2c_read(21);//читаем содержимое регистра 21 , I2C 0xAC;
	Transf("reg 21:\r\n");
	x_out("Receive errors                       :",z&0xffff);
	Transf("\r\n");
	
	z=i2c_read(22);//читаем содержимое регистра 27 , I2C 0xAC;
	Transf("reg 22:\r\n");
	x_out("MDI Pair Select                      :",z&0x03);
	Transf("\r\n");
	
	z=i2c_read(27);//читаем содержимое регистра 1 , I2C 0xAC;
	Transf("reg 27:\r\n");
	x_out("1000BASE-X Autonegotiation Bypass Enable :",(z>>12)&0x01);
	x_out("1000BASE-X Autonegotiation Bypass Status :",(z>>11)&0x01);
	
	z=i2c_read(28);//читаем содержимое регистра 28 , I2C 0xAC;
	Transf("reg 28:\r\n");
	x_out("Enable Cable Diagnostic Test :",(z>>15)&0x01);
	x_out("Status                       :",(z>>13)&0x03);
	Transf("\r\n");
	  
}

void PCS0_status(void)
{
	u32 a=0;
	u32 b=0;
	u32 c=0;
	u8 z=0;
	
	Transf("\r\n");
	Transf("----------\r\n");
	Transf("PCS0 status:\r\n");	
	
	c=IO("~0 mac0_config_reg_rd:0x94;");//IF_Mode Register
	b=IO("~0 mac0_config_reg_rd:0x80;");//считываем PCS Control Register
	a=IO("~0 mac0_config_reg_rd:0x81;");//считываем PCS Status Register
	
	x_out("LINK_STATUS              :",(a>>2)&0x01);
	x_out("AUTO NEGOTIATION ABILITY :",(a>>3)&0x01);
	x_out("AUTO NEGOTIATION COMPLETE:",(a>>5)&0x01);
	x_out("UNIDIRECTIONAL ABILITY   :",(a>>7)&0x01);
	Transf("\r\n");
	Transf("PCS0 config:\r\n");
	z=(((b>>6)&0x01)<<1)+((b>>13)&0x01);
	x_out("SPEED SELECTION         :",z);
	x_out("COLLISION TEST          :",(b>>7)&0x01);
	x_out("DUPLEX MODE             :",(b>>8)&0x01);
	x_out("RESTART AUTO NEGOTIATION:",(b>>9)&0x01);
	x_out("ISOLATE                 :",(b>>10)&0x01);
	x_out("POWERDOWN               :",(b>>11)&0x01);
	x_out("AUTO NEGOTIATION ENABLE :",(b>>12)&0x01);
	x_out("LOOPBACK                :",(b>>14)&0x01);
	x_out("RESET                   :",(b>>15)&0x01);
	
	Transf("\r\n");
	Transf("PCS0 IF_Mode Register :\r\n");
	
	x_out("SGMII_ENA                   :",(c>>0)&0x01);
	z=(((c>>3)&0x01)<<1)+((c>>2)&0x01);
	x_out("SGMII_SPEED                 :",z);
	
	
	a=IO("~0 mac0_config_reg_rd:0x82;");//считываем PCS phy_identifier
	b=IO("~0 mac0_config_reg_rd:0x83;");//считываем PCS phy_identifier
	a=IO("~0 mac0_config_reg_rd:0x91;");//считываем PCS rev	
}
void PCS1_status(void)
{
	u32 a=0;
	u32 b=0;
	u32 c=0;
	u8 z=0;
	
	Transf("\r\n");
	Transf("----------\r\n");
	Transf("PCS1 status:\r\n");	
	
	c=IO("~0 mac1_config_reg_rd:0x94;");//IF_Mode Register
	b=IO("~0 mac1_config_reg_rd:0x80;");//считываем PCS Control Register
	a=IO("~0 mac1_config_reg_rd:0x81;");//считываем PCS Status Register
	
	x_out("LINK_STATUS              :",(a>>2)&0x01);
	x_out("AUTO NEGOTIATION ABILITY :",(a>>3)&0x01);
	x_out("AUTO NEGOTIATION COMPLETE:",(a>>5)&0x01);
	x_out("UNIDIRECTIONAL ABILITY   :",(a>>7)&0x01);
	Transf("\r\n");
	Transf("PCS1 config:\r\n");
	z=(((b>>6)&0x01)<<1)+((b>>13)&0x01);
	x_out("SPEED SELECTION         :",z);
	x_out("COLLISION TEST          :",(b>>7)&0x01);
	x_out("DUPLEX MODE             :",(b>>8)&0x01);
	x_out("RESTART AUTO NEGOTIATION:",(b>>9)&0x01);
	x_out("ISOLATE                 :",(b>>10)&0x01);
	x_out("POWERDOWN               :",(b>>11)&0x01);
	x_out("AUTO NEGOTIATION ENABLE :",(b>>12)&0x01);
	x_out("LOOPBACK                :",(b>>14)&0x01);
	x_out("RESET                   :",(b>>15)&0x01);
	
	Transf("\r\n");
	Transf("PCS1 IF_Mode Register :\r\n");
	
	x_out("SGMII_ENA                   :",(c>>0)&0x01);
	z=(((c>>3)&0x01)<<1)+((c>>2)&0x01);
	x_out("SGMII_SPEED                 :",z);
	
	
	a=IO("~0 mac1_config_reg_rd:0x82;");//считываем PCS phy_identifier
	b=IO("~0 mac1_config_reg_rd:0x83;");//считываем PCS phy_identifier
	a=IO("~0 mac1_config_reg_rd:0x91;");//считываем PCS rev	
}

void MAC0_adr_write (u64 mac)  //функция записи мак адреса в IP ПЛИС
{
	u32 d0= mac    &0xff;
	u32 d1=(mac>> 8)&0xff;
	u32 d2=(mac>>16)&0xff;
	u32 d3=(mac>>24)&0xff;
	u32 d4=(mac>>32)&0xff;
	u32 d5=(mac>>40)&0xff;
	
	u32 tmp1=0;
	u32 tmp2=0;
	
	tmp1=(d2<<24)|(d3<<16)|(d4<<8)|(d5<<0);
	tmp2=(00<<24)|(00<<16)|(d0<<8)|(d1<<0);
	    //// mac=0x01c23174acb; 
		//// MAC address is 00-1C-23-17-4A-CB
//	IO("~0 mac0_config_reg_wr:0x03.0x17231C00;");//mac_0 0x17231C00
//	IO("~0 mac0_config_reg_wr:0x04.0x0000CB4A;");//mac_1 0x0000CB4A
	
	FPGA_wSPI ( 8,13,0x03);	 //записываем адресс конфигурационного регистра
	FPGA_wSPI (32,12,tmp1);	 //записываем конфигурационный регистр
	
	FPGA_wSPI ( 8,13,0x04);	 //записываем адресс конфигурационного регистра
	FPGA_wSPI (32,12,tmp2);	 //записываем конфигурационный регистр
}

void MAC1_adr_write (u64 mac)  //функция записи мак адреса в IP ПЛИС
{
	u32 d0= mac    &0xff;
	u32 d1=(mac>> 8)&0xff;
	u32 d2=(mac>>16)&0xff;
	u32 d3=(mac>>24)&0xff;
	u32 d4=(mac>>32)&0xff;
	u32 d5=(mac>>40)&0xff;
	
	u32 tmp1=0;
	u32 tmp2=0;
	
	tmp1=(d2<<24)|(d3<<16)|(d4<<8)|(d5<<0);
	tmp2=(00<<24)|(00<<16)|(d0<<8)|(d1<<0);
	    //// mac=0x01c23174acb; 
		//// MAC address is 00-1C-23-17-4A-CB
//	IO("~0 mac0_config_reg_wr:0x03.0x17231C00;");//mac_0 0x17231C00
//	IO("~0 mac0_config_reg_wr:0x04.0x0000CB4A;");//mac_1 0x0000CB4A
	
	FPGA_wSPI ( 8,122,0x03);	 //записываем адресс конфигурационного регистра
	FPGA_wSPI (32,121,tmp1);	 //записываем конфигурационный регистр
	
	FPGA_wSPI ( 8,122,0x04);	 //записываем адресс конфигурационного регистра
	FPGA_wSPI (32,121,tmp2);	 //записываем конфигурационный регистр
}

		
void MAC0_init (void)
{
	u64 mac=0x01c23174ac0; //mac адрес
	u32 x=0;
	int i=0;
	char a[64];
	for (i=0;i<64;i++) a[i]=0;	
	
	Transf("\r\n");
	Transf("=====================================================\r\n");
	Transf("       Starting TSE PCS - MAC0 Configuration\r\n");
	Transf("=====================================================\r\n");
	Transf("\r\n");
	Transf("----------\r\n");
	Transf("PCS0 init:\r\n");
	
//---------PCS configuration--------------
//Set Auto Negotiation Link Timer
	IO("~0 mac0_config_reg_wr:0x92.0x12d0;");//link_timer PCS registr adr : 0x80 + 0x12
	IO("~0 mac0_config_reg_wr:0x93.0x13;");  //Link_timer
//Configure SGMII	or 1000BASE-X  registr: dev_ability 0x04
    x=(0<< 0)|//SGMII_ENA
	  (0<< 1)|//USE_SGMII_AN 
	  (0<< 2)|//SGMII_SPEED[1:0]
	  (0<< 4)|//SGMII_DUPLEX
	  (0<< 5);//SGMII_AN_MODE	  
//	 x_out("PCS Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac0_config_reg_wr:0x94.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register

	 // Enable Auto Negotiation
//AUTO_NEGOTIATION_ENA = 0, Bit 6,8,13 can be ignore
//--------------------
Transf("\r\nформируем config PCS0 Register\r\n");
	x=(0<< 5)|//UNIDIRECTIONAL_ENABLE
	  (1<< 6)|//SPEED_SELECTION
	  (0<< 7)|//COLLISION_TEST
	  (1<< 8)|//DUPLEX_MODE
	  (0<< 9)|//RESTART_AUTO_NEGOTIATION
	  (0<<10)|//ISOLATE
	  (0<<11)|//POWERDOWN 
	  (0<<12)|//AUTO_NEGOTIATION_ENABLE 
	  (0<<13)|//SPEED_SELECTION
	  (0<<14)|//LOOPBACK 
	  (0<<15);//RESET 
	  
//	 x_out("PCS Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac0_config_reg_wr:0x80.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register
//PCS Software reset is recommended where there any configuration changed	 
	 //Set SW_RESET bit to 1
	 
	 Transf("\r\nшлём RESET в PCS0 config Register\r\n");
	 
	 x=x|(1<<15);
	 
	 sprintf (strng,"%x",x);	 
	 strcpy(a,"~0 mac0_config_reg_wr:0x80.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
//	 Transf(a);
	 IO(a);//PCS Command_config Register
//--------------------
	IO("~0 mac0_config_reg_rd:0x80;");//считываем PCS Control Register
	Transf("PCS0 status register:\r\n");
	IO("~0 mac0_config_reg_rd:0x81;");//считываем PCS Status Register

	Transf("PCS0 partner_ability register:\r\n");
	IO("~0 mac0_config_reg_rd:0x85;");//считываем PCS partner_ability

	Transf("\r\nMAC0 config\r\n");
	//MAC config Register
Transf("\r\nDESable transmit and receive operations \r\n");

	x=(0<< 0)|//TX_ENA 
	  (0<< 1)|//RX_ENA
	  (0<< 2)|//XON_GEN
	  (0<< 3)|//ETH_SPEED 
	  (0<< 4)|//PROMIS_EN приём всех пакетов!!!
	  (1<< 5)|//PAD_EN
	  (0<< 6)|//CRC_FWD
	  (0<< 7)|//PAUSE_FWD
	  (0<< 8)|//PAUSE_IGNORE
	  (1<< 9)|//TX_ADDR_INS
	  (0<<10)|//HD_ENA
	  (0<<11)|//EXCESS_COL 
	  (0<<12)|//LATE_COL 
	  (1<<13)|//SW_RESET
	  (0<<14)|//MHASH_SEL 
	  (0<<15)|//LOOP_ENA 
	  (0<<16)|//TX_ADDR_SEL
	  (0<<19)|//MAGIC_ENA
	  (0<<20)|//SLEEP
	  (0<<21)|//WAKEUP
	  (0<<22)|//XOFF_GEN
	  (1<<23)|//CNTL_FRM_ENA
	  (0<<24)|//NO_LGTH_CHECK
	  (0<<25)|//ENA_10
	  (0<<26)|//RX_ERR_DISC
	  (0<<27)|//DISABLE_READ_TIMEOUT
	  (0<<28)|//Reserved
	  (0<<31);//CNT_RESET
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac0_config_reg_wr:2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);

	 IO(a);//Command_config Register	
     IO("~0 mac0_config_reg_rd:2;");//Read the TX_ENA and RX_ENA bit is set 0 to ensure TX and RX path is disable

	IO("~0 mac0_config_reg_wr:0x09.0x7f0;");//Tx_section_empty  = Max FIFO size-16   (2048-16) 
	IO("~0 mac0_config_reg_rd:0x09;");
	IO("~0 mac0_config_reg_wr:0x0e.0x3;");//Tx_almost_full = 3
	IO("~0 mac0_config_reg_rd:0x0e;");
	IO("~0 mac0_config_reg_wr:0x0d.0x08;");//Tx_almost_empty = 8
	IO("~0 mac0_config_reg_rd:0x0d;");
	IO("~0 mac0_config_reg_wr:0x07.0x7f0;");//Rx_section_empty = Max FIFO size - 16
	IO("~0 mac0_config_reg_rd:0x07;");
	IO("~0 mac0_config_reg_wr:0x0c.0x8;");//Rx_almost_full = 8
	IO("~0 mac0_config_reg_rd:0x0c;");
	IO("~0 mac0_config_reg_wr:0x0b.0x00;");//Rx_almost_empty = 0 Store and forward 
	IO("~0 mac0_config_reg_rd:0x0b;");
	IO("~0 mac0_config_reg_wr:0x0a.0x10;");//Tx_section_full = 16
	IO("~0 mac0_config_reg_rd:0x0a;");
	IO("~0 mac0_config_reg_wr:0x08.0x00;");//Rx_section_full = 0 Store and forward   
	IO("~0 mac0_config_reg_rd:0x08;");
	////MAC address is 00-1C-23-17-4A-CB
//	IO("~0 mac0_config_reg_wr:0x03.0x17231C00;");//mac_0 0x17231C00
//	IO("~0 mac0_config_reg_wr:0x04.0x0000CB4A;");//mac_1 0x0000CB4A

	 MAC0_adr_write (mac);  //функция записи мак адреса в IP ПЛИС
	 
	IO("~0 mac0_config_reg_wr:0x05.0x5dc;");//Maximum Frame Length is 1500 bytes
	IO("~0 mac0_config_reg_rd:0x05;");
	IO("~0 mac0_config_reg_wr:0x17.0x0c;");//Minimum Inter Packet Gap is 12 bytes
	IO("~0 mac0_config_reg_rd:0x17;");
	IO("~0 mac0_config_reg_wr:0x06.0xffff;");//Maximum Pause Quanta Value for Flow Control
	IO("~0 mac0_config_reg_rd:0x06;");

	x=(0<< 0)|//TX_ENA 
	  (0<< 1)|//RX_ENA
	  (0<< 2)|//XON_GEN
	  (0<< 3)|//ETH_SPEED 
	  (0<< 4)|//PROMIS_EN приём всех пакетов!!!
	  (0<< 5)|//PAD_EN
	  (0<< 6)|//CRC_FWD
	  (0<< 7)|//PAUSE_FWD
	  (0<< 8)|//PAUSE_IGNORE
	  (1<< 9)|//TX_ADDR_INS
	  (0<<10)|//HD_ENA
	  (0<<11)|//EXCESS_COL 
	  (0<<12)|//LATE_COL 
	  (0<<13)|//SW_RESET
	  (0<<14)|//MHASH_SEL 
	  (0<<15)|//LOOP_ENA 
	  (0<<16)|//TX_ADDR_SEL
	  (0<<19)|//MAGIC_ENA
	  (0<<20)|//SLEEP
	  (0<<21)|//WAKEUP
	  (0<<22)|//XOFF_GEN
	  (1<<23)|//CNTL_FRM_ENA
	  (0<<24)|//NO_LGTH_CHECK
	  (0<<25)|//ENA_10
	  (0<<26)|//RX_ERR_DISC
	  (0<<27)|//DISABLE_READ_TIMEOUT
	  (0<<28)|//Reserved
	  (0<<31);//CNT_RESET
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac0_config_reg_wr:2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register
     IO("~0 mac0_config_reg_rd:2;");//чтение

	
 Transf("\r\n шлём RESET в MAC0 config Register\r\n");

	 x=x|(1<<13);
	 
	 sprintf (strng,"%x",x);	 
	 strcpy(a,"~0 mac0_config_reg_wr:0x2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
//	 Transf(a);
	 IO(a);//Command_config Register
	 IO("~0 mac0_config_reg_rd:2;");//считываем 	 
//desable local loopback on	 
//Transf("\r\n enable local loopback on\r\n");

     
Transf("\r\nEnable transmit and receive operations \r\n");

	 x=x&(~(1<<13));//сбрасываем бит ресета
	 x=x|0x03;
	  
	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac0_config_reg_wr:0x2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register		 

//----------------------------------------------------

	 Transf("Считываем конфигурационный регистр\r\n");
	 IO("~0 mac0_config_reg_rd:2;");//считываем 
	 
//----------------------------------------------------	
//записываем в ПЛИС в eth наш IP адрес и номер рабочего порта
//Тут должны быть SPI адреса с одного модуля ETH !!!!

u64   IP_my  = 0x0103013C;//1.3.1.60
u64   IP_dest= 0x01030101;
u16 PORT_my  = 77;
u16 PORT_dest=139;
 
    ETH0_IP_write   (IP_my  ,IP_dest  );
	ETH0_PORT_write (PORT_my,PORT_dest);
	ETH0_MAC_write  (mac);	 
	 
}
void MAC1_init (void)
{
	u64 mac=0x01c23174ac1; //mac адрес
	u32 x=0;
	int i=0;
	char a[64];
	for (i=0;i<64;i++) a[i]=0;	
	
	Transf("\r\n");
	Transf("=====================================================\r\n");
	Transf("       Starting TSE PCS - MAC1 Configuration\r\n");
	Transf("=====================================================\r\n");
	Transf("\r\n");
	Transf("----------\r\n");
	Transf("PCS1 init:\r\n");
	
//---------PCS configuration--------------
//Set Auto Negotiation Link Timer
	IO("~0 mac1_config_reg_wr:0x92.0x12d0;");//link_timer PCS registr adr : 0x80 + 0x12
	IO("~0 mac1_config_reg_wr:0x93.0x13;");  //Link_timer
//Configure SGMII	or 1000BASE-X  registr: dev_ability 0x04
    x=(0<< 0)|//SGMII_ENA
	  (0<< 1)|//USE_SGMII_AN 
	  (0<< 2)|//SGMII_SPEED[1:0]
	  (0<< 4)|//SGMII_DUPLEX
	  (0<< 5);//SGMII_AN_MODE	  
//	 x_out("PCS Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac1_config_reg_wr:0x94.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register

	 // Enable Auto Negotiation
//AUTO_NEGOTIATION_ENA = 0, Bit 6,8,13 can be ignore
//--------------------
Transf("\r\nформируем config PCS1 Register\r\n");
	x=(0<< 5)|//UNIDIRECTIONAL_ENABLE
	  (1<< 6)|//SPEED_SELECTION
	  (0<< 7)|//COLLISION_TEST
	  (1<< 8)|//DUPLEX_MODE
	  (0<< 9)|//RESTART_AUTO_NEGOTIATION
	  (0<<10)|//ISOLATE
	  (0<<11)|//POWERDOWN 
	  (0<<12)|//AUTO_NEGOTIATION_ENABLE 
	  (0<<13)|//SPEED_SELECTION
	  (0<<14)|//LOOPBACK 
	  (0<<15);//RESET 
	  
//	 x_out("PCS Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac1_config_reg_wr:0x80.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register
//PCS Software reset is recommended where there any configuration changed	 
	 //Set SW_RESET bit to 1
	 
	 Transf("\r\nшлём RESET в PCS1 config Register\r\n");
	 
	 x=x|(1<<15);
	 
	 sprintf (strng,"%x",x);	 
	 strcpy(a,"~0 mac1_config_reg_wr:0x80.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
//	 Transf(a);
	 IO(a);//PCS Command_config Register
//--------------------
	IO("~0 mac1_config_reg_rd:0x80;");//считываем PCS Control Register
	Transf("PCS1 status register:\r\n");
	IO("~0 mac1_config_reg_rd:0x81;");//считываем PCS Status Register

	Transf("PCS1 partner_ability register:\r\n");
	IO("~0 mac1_config_reg_rd:0x85;");//считываем PCS partner_ability

	Transf("\r\nMAC1 config\r\n");
	//MAC config Register
Transf("\r\nDESable transmit and receive operations \r\n");

	x=(0<< 0)|//TX_ENA 
	  (0<< 1)|//RX_ENA
	  (0<< 2)|//XON_GEN
	  (0<< 3)|//ETH_SPEED 
	  (0<< 4)|//PROMIS_EN приём всех пакетов!!!
	  (1<< 5)|//PAD_EN
	  (0<< 6)|//CRC_FWD
	  (0<< 7)|//PAUSE_FWD
	  (0<< 8)|//PAUSE_IGNORE
	  (1<< 9)|//TX_ADDR_INS
	  (0<<10)|//HD_ENA
	  (0<<11)|//EXCESS_COL 
	  (0<<12)|//LATE_COL 
	  (1<<13)|//SW_RESET
	  (0<<14)|//MHASH_SEL 
	  (0<<15)|//LOOP_ENA 
	  (0<<16)|//TX_ADDR_SEL
	  (0<<19)|//MAGIC_ENA
	  (0<<20)|//SLEEP
	  (0<<21)|//WAKEUP
	  (0<<22)|//XOFF_GEN
	  (1<<23)|//CNTL_FRM_ENA
	  (0<<24)|//NO_LGTH_CHECK
	  (0<<25)|//ENA_10
	  (0<<26)|//RX_ERR_DISC
	  (0<<27)|//DISABLE_READ_TIMEOUT
	  (0<<28)|//Reserved
	  (0<<31);//CNT_RESET
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac1_config_reg_wr:2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);

	 IO(a);//Command_config Register	
     IO("~0 mac1_config_reg_rd:2;");//Read the TX_ENA and RX_ENA bit is set 0 to ensure TX and RX path is disable

	IO("~0 mac1_config_reg_wr:0x09.0x7f0;");//Tx_section_empty  = Max FIFO size-16   (2048-16) 
	IO("~0 mac1_config_reg_rd:0x09;");
	IO("~0 mac1_config_reg_wr:0x0e.0x3;");//Tx_almost_full = 3
	IO("~0 mac1_config_reg_rd:0x0e;");
	IO("~0 mac1_config_reg_wr:0x0d.0x08;");//Tx_almost_empty = 8
	IO("~0 mac1_config_reg_rd:0x0d;");
	IO("~0 mac1_config_reg_wr:0x07.0x7f0;");//Rx_section_empty = Max FIFO size - 16
	IO("~0 mac1_config_reg_rd:0x07;");
	IO("~0 mac1_config_reg_wr:0x0c.0x8;");//Rx_almost_full = 8
	IO("~0 mac1_config_reg_rd:0x0c;");
	IO("~0 mac1_config_reg_wr:0x0b.0x00;");//Rx_almost_empty = 0 Store and forward 
	IO("~0 mac1_config_reg_rd:0x0b;");
	IO("~0 mac1_config_reg_wr:0x0a.0x10;");//Tx_section_full = 16
	IO("~0 mac1_config_reg_rd:0x0a;");
	IO("~0 mac1_config_reg_wr:0x08.0x00;");//Rx_section_full = 0 Store and forward   
	IO("~0 mac1_config_reg_rd:0x08;");
	////MAC address is 00-1C-23-17-4A-CB
	//mac=0x01c23174acb; 
//	IO("~0 mac1_config_reg_wr:0x03.0x17231C00;");//mac_0 0x17231C00
//	IO("~0 mac1_config_reg_wr:0x04.0x0000CB4A;");//mac_1 0x0000CB4A
	
	 MAC1_adr_write (mac);  //функция записи мак адреса в IP ПЛИС
	
	IO("~0 mac1_config_reg_wr:0x05.0x5dc;");//Maximum Frame Length is 1500 bytes
	IO("~0 mac1_config_reg_rd:0x05;");
	IO("~0 mac1_config_reg_wr:0x17.0x0c;");//Minimum Inter Packet Gap is 12 bytes
	IO("~0 mac1_config_reg_rd:0x17;");
	IO("~0 mac1_config_reg_wr:0x06.0xffff;");//Maximum Pause Quanta Value for Flow Control
	IO("~0 mac1_config_reg_rd:0x06;");

	x=(0<< 0)|//TX_ENA 
	  (0<< 1)|//RX_ENA
	  (0<< 2)|//XON_GEN
	  (0<< 3)|//ETH_SPEED 
	  (0<< 4)|//PROMIS_EN приём всех пакетов!!!
	  (0<< 5)|//PAD_EN
	  (0<< 6)|//CRC_FWD
	  (0<< 7)|//PAUSE_FWD
	  (0<< 8)|//PAUSE_IGNORE
	  (1<< 9)|//TX_ADDR_INS
	  (0<<10)|//HD_ENA
	  (0<<11)|//EXCESS_COL 
	  (0<<12)|//LATE_COL 
	  (0<<13)|//SW_RESET
	  (0<<14)|//MHASH_SEL 
	  (0<<15)|//LOOP_ENA 
	  (0<<16)|//TX_ADDR_SEL
	  (0<<19)|//MAGIC_ENA
	  (0<<20)|//SLEEP
	  (0<<21)|//WAKEUP
	  (0<<22)|//XOFF_GEN
	  (1<<23)|//CNTL_FRM_ENA
	  (0<<24)|//NO_LGTH_CHECK
	  (0<<25)|//ENA_10
	  (0<<26)|//RX_ERR_DISC
	  (0<<27)|//DISABLE_READ_TIMEOUT
	  (0<<28)|//Reserved
	  (0<<31);//CNT_RESET
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac1_config_reg_wr:2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register
     IO("~0 mac1_config_reg_rd:2;");//чтение

	
 Transf("\r\n шлём RESET в MAC1 config Register\r\n");

	 x=x|(1<<13);
	 
	 sprintf (strng,"%x",x);	 
	 strcpy(a,"~0 mac1_config_reg_wr:0x2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
//	 Transf(a);
	 IO(a);//Command_config Register
	 IO("~0 mac1_config_reg_rd:2;");//считываем 	 
//desable local loopback on	 
//Transf("\r\n enable local loopback on\r\n");

     
Transf("\r\nEnable transmit and receive operations \r\n");

	 x=x&(~(1<<13));//сбрасываем бит ресета
	 x=x|0x03;
	  
	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 mac1_config_reg_wr:0x2.0x");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 
//	 Transf(a);
	 
	 IO(a);//Command_config Register		 

//----------------------------------------------------

	 Transf("Считываем конфигурационный регистр\r\n");
	 IO("~0 mac1_config_reg_rd:2;");//считываем 
	 
//----------------------------------------------------	
//записываем в ПЛИС в eth наш IP адрес и номер рабочего порта
//Тут должны быть SPI адреса с одного модуля ETH !!!!

u64   IP_my  = 0x0103013d;//1.3.1.61
u64   IP_dest= 0x01030101;
u16 PORT_my  = 77;
u16 PORT_dest=139;

    ETH1_IP_write   (IP_my  ,IP_dest  );
	ETH1_PORT_write (PORT_my,PORT_dest);
	ETH1_MAC_write  (mac);	 
	 
}
void MAC0_statistic (void)
{
	u32 a=0;
	u32 b=0;
	
	Transf("\r\n");
	Transf("----------\r\n");
	Transf("MAC0 statistic:\r\n");
	
	a=IO("~0 mac0_config_reg_rd:0x18;");//считываем aMacID
	b=IO("~0 mac0_config_reg_rd:0x19;");//считываем aMacID
	
	xn_out("MAC adr:",a);x_out("-",b);
	
	a=IO("~0 mac0_config_reg_rd:0x1a;");//считываем aFramesTransmittedOK
	
	a=IO("~0 mac0_config_reg_rd:0x1b;");//считываем aFramesReceivedOK
	
	a=IO("~0 mac0_config_reg_rd:0x1c;");//считываем aFrameCheckSequenceErrors
	
	a=IO("~0 mac0_config_reg_rd:0x1d;");//считываем aAlignmentErrors
	
	a=IO("~0 mac0_config_reg_rd:0x1e;");//считываем aOctetsTransmittedOK
	
	a=IO("~0 mac0_config_reg_rd:0x1f;");//считываем aOctetsReceivedOK
	
	a=IO("~0 mac0_config_reg_rd:0x20;");//считываем aTxPAUSEMACCtrlFrames
	
	a=IO("~0 mac0_config_reg_rd:0x21;");//считываем aRxPAUSEMACCtrlFrames
	
	a=IO("~0 mac0_config_reg_rd:0x22;");//считываем ifInErrors
	
	a=IO("~0 mac0_config_reg_rd:0x23;");//считываем ifOutErrors
	
	a=IO("~0 mac0_config_reg_rd:0x24;");//считываем ifInUcastPkts
	
	a=IO("~0 mac0_config_reg_rd:0x25;");//считываем ifInMulticastPkts
	
	a=IO("~0 mac0_config_reg_rd:0x26;");//считываем ifInBroadcastPkts
	
	a=IO("~0 mac0_config_reg_rd:0x27;");//считываем ifOutDiscards
	
	a=IO("~0 mac0_config_reg_rd:0x28;");//считываем ifOutUcastPkts
	
	a=IO("~0 mac0_config_reg_rd:0x29;");//считываем ifOutMulticastPkts
}
void MAC1_statistic (void)
{
	u32 a=0;
	u32 b=0;
	
	Transf("\r\n");
	Transf("----------\r\n");
	Transf("MAC1 statistic:\r\n");
	
	a=IO("~0 mac1_config_reg_rd:0x18;");//считываем aMacID
	b=IO("~0 mac1_config_reg_rd:0x19;");//считываем aMacID
	
	xn_out("MAC1 adr:",a);x_out("-",b);
	
	a=IO("~0 mac1_config_reg_rd:0x1a;");//считываем aFramesTransmittedOK
	
	a=IO("~0 mac1_config_reg_rd:0x1b;");//считываем aFramesReceivedOK
	
	a=IO("~0 mac1_config_reg_rd:0x1c;");//считываем aFrameCheckSequenceErrors
	
	a=IO("~0 mac1_config_reg_rd:0x1d;");//считываем aAlignmentErrors
	
	a=IO("~0 mac1_config_reg_rd:0x1e;");//считываем aOctetsTransmittedOK
	
	a=IO("~0 mac1_config_reg_rd:0x1f;");//считываем aOctetsReceivedOK
	
	a=IO("~0 mac1_config_reg_rd:0x20;");//считываем aTxPAUSEMACCtrlFrames
	
	a=IO("~0 mac1_config_reg_rd:0x21;");//считываем aRxPAUSEMACCtrlFrames
	
	a=IO("~0 mac1_config_reg_rd:0x22;");//считываем ifInErrors
	
	a=IO("~0 mac1_config_reg_rd:0x23;");//считываем ifOutErrors
	
	a=IO("~0 mac1_config_reg_rd:0x24;");//считываем ifInUcastPkts
	
	a=IO("~0 mac1_config_reg_rd:0x25;");//считываем ifInMulticastPkts
	
	a=IO("~0 mac1_config_reg_rd:0x26;");//считываем ifInBroadcastPkts
	
	a=IO("~0 mac1_config_reg_rd:0x27;");//считываем ifOutDiscards
	
	a=IO("~0 mac1_config_reg_rd:0x28;");//считываем ifOutUcastPkts
	
	a=IO("~0 mac1_config_reg_rd:0x29;");//считываем ifOutMulticastPkts
}
void sfp_init(void)
{

char a[64];

u32 x=0;
u16 z=0;

Transf("\r\n===========\r\n");
Transf("\r\nsfp init: \r\n");

//program reg 22 used copper register
z=IO("~0 i2c_read:22;");//читаем содержимое регистра 22 , I2C 0xAC;

	 x=z&(~0xff);
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 i2c_write:22.");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 IO(a);//Command_config Register
	 
//program reg 27 used copper register
z=IO("~0 i2c_read:27;");//читаем содержимое регистра 27 , I2C 0xAC;

	 x=z|(0x01<<12);
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 i2c_write:27.");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 IO(a);//Command_config Register

//program reg 0 used copper register
z=IO("~0 i2c_read:0;");//читаем содержимое регистра 0 , I2C 0xAC;

	 x=  (0<<15) //Reset
	    |(0<<14) //Loopback
	    |(0<<13) //Speed Selection (LSB)
	    |(0<<12) //Auto-Negotiation Enable
	    |(0<<11) //Power Down
	    |(0<<10) //Isolate
	    |(0<< 9) //Restart Auto-Negotiation
	    |(1<< 8) //Duplex Mode
	    |(0<< 7) //Collision Test
	    |(1<< 6) //Speed Selection (MSB)
		|(0<< 0);	    
	  
//	 x_out("Command_config Register:",x);
	 
	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 i2c_write:0.");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 IO(a);//Command_config Register	 
//-----------------------
//шлём ресет:
Transf("\r\nшлём ресет\r\n");
	 x=x|(1<<15);

	 sprintf (strng,"%x",x);
	 
	 strcpy(a,"~0 i2c_write:0.");
	 strcat(a,strng);
	 strcat(a,";\r\n");
	 IO(a);//Command_config Register
//-----------------------

}

void UDP_tx_init (void)
{
u16 i=0;
u32 data=0;
u16 adr=0;

u8 d0=0;
u8 d1=0;
u8 d2=0;
u8 d3=0;

u8 k=0;
u32 crc=0;
u8 hash=0;

Transf("\r\n---------\r\n");
Transf("Заполняем МЕМ3>\r\n");

hash=0x01;

for (i=0;i<32;i++)
	{
			hash=hash+hash;
			
		if (k==0) d0=hash;
		if (k==1) d1=hash;
		if (k==2) d2=hash;
		if (k==3) d3=hash;
		
		if (k==3) 
		{			
			data=    (d0<<24)+(d1<<16) + (d2<< 8)+(d3<<0);
			crc=crc+((d0<< 8)+(d1<< 0))+((d2<< 8)+(d3<<0));//число байт данных должно быть чётно!!! чтобы можно было сложить в 16 бит
			FPGA_wSPI (16,38, adr);//записываем адресс памяти
			FPGA_wSPI (32,39,data);//записываем данные памяти	
			
			un_out("",adr);
			Transf(":\r\n");
		    hn_out(data,24);hn_out(data,16);hn_out(data,8);h_out(data,0);
			adr++;
		}		
		
		if (k<3) k++; else k=0;	
		//Transf(".");
	}
	
	FPGA_wSPI (32,44, crc);//записываем CRC данных в памяти
	
Transf("\r\n");	
Transf("\r\n---------\r\n");	
}

void UDP_buf_read(void)
{
u16 i=0;
u32 data=0;

Transf("\r\n---------\r\n");
for (i=0;i<32;i++)
	{
	       FPGA_wSPI (16,34,i);//записываем адресс памяти
	  data=FPGA_rSPI (32,37);  //считываем содержимое
	  hn_out(data,24);hn_out(data,16);hn_out(data,8);h_out(data,0);//выводим сырой код
	}	  
}

extern u8  RX_BUF  [2048];

void UDP_mem_read (u8 n)
{	
	u64 z=0;
	u16 size=0;
	u16 i=0;
	u16 j=0;
	u32 data=0;
	u32 temp_data=0;
	u32 tmp=0;
	//------ETH-----------
	u64 ETH_dest  =0;
	u64 ETH_source=0;
	u16 frame_type=0;
	//------ARP-----------
	u16 Hardware_type=0;//1 - for ethernet
	u16 Protocol_type=0;//0x0800 for IP
	u8  Hardware_len =0;//длинна в байтах Hardware addresses (6 байт для eth )
	u8  Protocol_len =0;//длинна в байтах логического адреса (4 байта для IP)
	u16 ARP_operation=0;//1-request;2=reply;3/4=RARP req/reply
	u64 Sender_MAC   =0;//
	u32 Sender_IP    =0;//
	u64 Dest_MAC     =0;//
	u32 Dest_IP      =0;//
	//-------IP-----------
	u8  Version=0;       //4-IP;5-ST datagram;6-IPv6;8-PIP
	u8  IHL=0;           // Internet Header Length min 5 ( Internet Header Length)
	u8  TOS=0;           // Type of Service (Differentiated Services) 
	u16 Total_length=0;  //Contains the length of the datagram.
	u16 Identification=0;//
	u8  Flags=2;//R, Reserved. 1 bit.Should be set to 0.||DF, Don't fragment. 1 bit. 1 -Do not fragment.||MF, More fragments. 1 bit. 0	This is the last fragment.
	u16 Fragmet_offset=0;//Used to direct the reassembly of a fragmented datagram.
	u8  TTL=0;//A timer field used to track the lifetime of the datagram. When the TTL field is decremented down to zero, the datagram is discarded.
	u8  Protocol=0;//This field specifies the next encapsulated protocol.: 
	//1-ICMP
	//2-IGMP
	//3-GGP
	//4-IP,in IP encapsulation
	//6-TCP
	//17 - UDP,User Datagram Protocol
	//41 - IPv6 over IPv4
	//58 - ICMPv6
	u16 Header_checksum=0;//A 16 bit one's complement checksum of the IP header and IP options.
	u32 Source_IP=0;//IP address of the sender.
	u32 Destination_IP=0;//IP address of the intended receiver.
//	u8  Option=0;//Variable length.
//	u32 Padding=0;// Variable length.Used as a filler to guarantee that the data starts on a 32 bit boundary
	//--------UDP------------
	u16 source_port=0;
	u16 dest_port  =0;
	u16 lenght     =0;
	u16 Checksum   =0;
	//-------ICMP-------------
	u8  type=0;//
	u8  code=0;
	u16 Identifier=0;
	u16 Seq_number=0;
	u16 rcv_size=0;
	
	Transf("\r\n---------\r\n");
	
	if (n==0) z=FPGA_rSPI (48,18 );//считываем служебные данные 
	if (n==1) z=FPGA_rSPI (48,126);//считываем служебные данные 
	
	size=(z>>32)&0xffff;
	
	u_out("size    :",size);
	u_out("zero    :",(z>>30)&0x3);
	x_out("rx_mod  :",(z>>28)&0x3);
	x_out("frm_type:",(z>>24)&0x0f);
	x_out("rx_err  :",(z>>18)&0x3f);
	x_out("err_stat:",(z>>0 )&0x3ffff);
	Transf("\r\n\r\n");
	
	for (i=1;i<(size+1);i++)
	{
		if (n==0)
		{
				 FPGA_wSPI (16,16,i-1);//записываем адресс памяти
			data=FPGA_rSPI (32,17);  //считываем содержимое	
		}
		
		if (n==1)
		{
				 FPGA_wSPI (16,124,i-1);//записываем адресс памяти
			data=FPGA_rSPI (32,125);  //считываем содержимое	
		}
	       

	//-----------eth frame-----------------  
	  if (i== 1) ETH_dest=data;
	  if (i== 2) ETH_dest=(ETH_dest<<16)+(data>>16);	  
	  if (i== 2) ETH_source=data&0xffff;
	  if (i== 3) ETH_source=(ETH_source<<32)+data;	  
	  if (i== 4) frame_type=data>>16;
		  
	  if (frame_type==0x0806)
	  {	 	  
		//------------ARP frame----------------  
		  if (i== 4) Hardware_type=data&0xffff;
		  if (i== 5) Protocol_type=data>>16;
		  if (i== 5) Hardware_len =(data>>8)&0xff;
		  if (i== 5) Protocol_len =(data>>0)&0xff;
		  if (i== 6) ARP_operation=data>>16;
		  if (i== 6) Sender_MAC   =data&0xffff;
		  if (i== 7) Sender_MAC   =(Sender_MAC<<32)+(data>> 0);
		  if (i== 8) Sender_IP    =data;
		  if (i== 9) Dest_MAC     =data;
		  if (i==10) Dest_MAC     =(Dest_MAC<<32)+((data>>16)&0xffff);
		  if (i==10) {Dest_IP     =data&0xffff;                       }
		  if (i==11) {Dest_IP     =(Dest_IP <<16)+((data>>16)&0xffff);}
	   }
	   
	   if (frame_type==0x0800)//ip4 (0x86dd - IPv6)
	   {
		 //------------IP header----------------  
		  if (i== 4) Version=(data>>12)&0x0f;
		  if (i== 4) IHL    =(data>> 8)&0x0f;
		  if (i== 4) TOS    =(data>> 0)&0xff; 
		  
		  if (i== 5) Total_length   =(data>> 16)&0xffff;
		  if (i== 5) Identification = data&0xffff;
		  if (i== 6) Flags          =(data>> 29)&0x07;
		  if (i== 6) Fragmet_offset =(data>> 16)&0x1fff;
		  if (i== 6) TTL            =(data>>  8)&0xff;
		  if (i== 6) Protocol       =(data>>  0)&0xff;
		  if (i== 7) Header_checksum=(data>> 16)&0xffff;
		  if (i== 7) Source_IP      =(data>>  0)&0xffff;
		  if (i== 8) Source_IP      =(Source_IP<<16)+((data>> 16)&0xffff);
		  if (i== 8) Destination_IP =(data>>  0)&0xffff;
		  if (i== 9) Destination_IP =(Destination_IP<<16)+((data>> 16)&0xffff);
		  
		  if (Protocol==17) //UDP
		  {
		//------------UDP header----------------
			  if (i== 9) source_port    =(data>>  0)&0xffff;
			  if (i==10) dest_port      =(data>> 16)&0xffff;
			  if (i==10) lenght         =(data>>  0)&0xffff;
			  if (i==11) 
			  {
				  Checksum       =(data>> 16)&0xffff;
				  temp_data 	 = data;
				  Transf("DATA:\r\n");
			  }
			  
			  if (i >11)
			  {
				tmp           =(temp_data<<16)+(data>>16);//переписываем блок данных из UDP пакета в буфер
				temp_data     =      data;
				
				RX_BUF [4*j+3]=(tmp>>24)&0xff;
				RX_BUF [4*j+2]=(tmp>>16)&0xff;
				RX_BUF [4*j+1]=(tmp>> 8)&0xff;
				RX_BUF [4*j+0]=(tmp>> 0)&0xff;
				
				hn_out(tmp,24);hn_out(tmp,16);hn_out(tmp,8);h_out(tmp,0);//выводим данные пришедшие по UDP  
				j++;
			  }
		  }
		  
		  if (Protocol==1) //ICMP
		  {
			  //------------ICMP message----------------
			  if (i== 9) type      =(data>>  8)&0xff;//8 -Echo Req,0-Echo Reply
			  if (i== 9) code      =(data>>  0)&0xff;//0
			  if (i==10) Checksum  =(data>> 16)&0xffff;
			  if (i==10) Identifier=(data>>  0)&0xffff;
			  if (i==11) Seq_number=(data>> 16)&0xffff;
		  }
		  
		  
	   }

	//  hn_out(data,24);hn_out(data,16);hn_out(data,8);h_out(data,0);//выводим сырой код
	}
	Transf("\r\n--------\r\n");
	xn_out("ETH_dest         :",ETH_dest  >>24);x_out("",ETH_dest  &0xffffff);
	xn_out("ETH_source       :",ETH_source>>24);x_out("",ETH_source&0xffffff);
	 x_out("frame_type       :",frame_type);
	
	if (frame_type==0x0806) 
	{
	Transf("\r\nARP frame:\r\n\r\n");

	 x_out("Hardware_type    :",Hardware_type);
	 x_out("Protocol_type    :",Protocol_type);
	 u_out("Hardware_len     :",Hardware_len);
	 u_out("Protocol_len     :",Protocol_len);
	 x_out("ARP_operation    :",ARP_operation);
	Transf("Sender_MAC       :");
	hn_out(Sender_MAC,40);hn_out(Sender_MAC,32);hn_out(Sender_MAC,24);hn_out(Sender_MAC,16);hn_out(Sender_MAC,8);h_out(Sender_MAC,0);
	Transf("Sender_IP        :");
	in_out(Sender_IP,24);in_out(Sender_IP,16);in_out(Sender_IP,8);i_out(Sender_IP,0);
	Transf("Dest_MAC         :");
	hn_out(Dest_MAC,40);hn_out(Dest_MAC,32);hn_out(Dest_MAC,24);hn_out(Dest_MAC,16);hn_out(Dest_MAC,8);h_out(Dest_MAC,0);
	Transf("Dest_IP          :");
	in_out(Dest_IP,24);in_out(Dest_IP,16);in_out(Dest_IP,8);i_out(Dest_IP,0);
		
	}
	if (frame_type==0x0800) 
	{
		Transf("\r\nIP frame:\r\n\r\n");
		 x_out("Version        :",Version);
		 x_out("IHL            :",IHL);
		 x_out("TOS            :",TOS);
		 u_out("Total_length   :",Total_length);
		 x_out("Identification :",Identification);
		 x_out("Flags          :",Flags);
		 u_out("Fragmet_offset :",Fragmet_offset);
		 x_out("TTL            :",TTL);
		 x_out("Protocol       :",Protocol);
		 x_out("Header_checksum:",Header_checksum);
		Transf("Source_IP      :");
		in_out(Source_IP,24);in_out(Source_IP,16);in_out(Source_IP,8);i_out(Source_IP,0);
		
		Transf("Destination_IP :");
		in_out(Destination_IP,24);in_out(Destination_IP,16);in_out(Destination_IP,8);i_out(Destination_IP,0);
	
		if (Protocol==17)
		{
			Transf("\r\nUDP frame:\r\n\r\n");
			u_out("source_port    :",source_port);
			u_out("dest_port      :",dest_port);
			u_out("lenght         :",lenght);
			x_out("Checksum       :",Checksum);	
		}
			
		
		if (Protocol==1)
		{
			Transf("\r\nICMP frame:\r\n\r\n");
			u_out("type         :",type);
			u_out("code         :",code);			
			x_out("Checksum     :",Checksum);
			x_out("Identifier   :",Identifier);
			u_out("Seq_number   :",Seq_number);	
		}
	}	
	
	if (n==0) FPGA_wSPI (16,16 ,0xffff);//записываем служебное слово
	if (n==1) FPGA_wSPI (16,124,0xffff);//записываем служебное слово
	//---------------------------------------------------------------
	rcv_size=4*j;
	u_out("rcv_size:",rcv_size);
	FRAME_DECODE (RX_BUF,rcv_size);//обрабатываем принятый пакет
	
}

u32 crc_input=0u; 
u32 crc_comp=0u;

u32 IO ( char* str)      // функция обработки протокола обмена
{
	  u32 ccc;
	  char *ch;
 unsigned int i=0;
 u8 tmp1;
 u32 lb;
 u64 v1;
 u32 z1,z2;

  i = lenght;//длинна принятой пачки
  if (lenght==0) i = leng(str);
  lenght = 0;
 
  indexZ = 0;
  
  if ((time_uart>50u)||(SCH_LENGHT_PACKET>MAX_PL))
  {
	  //-------------------
		packet_flag=0; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;
  }
  
  while (i>0u)   //перегрузка принятого пакета в массив обработки
  
  {  

	if ((str[indexZ]==0x7e)&&(packet_flag==0))// обнаружено начало пакета
	  {  
		//-------------------
		packet_flag=1; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;		
	  }

	 InOut[index1]=str[indexZ];
	 SCH_LENGHT_PACKET++;//подсчитываем длинну пакета
		 
	if (( InOut[index1]==';')&&(FLAG_DATA==0u)&&(packet_flag==1))  {packet_flag=0;packet_ok=1u;FLAG_CW=1u;}
    
	if (((InOut[index1]=='=')||(InOut[index1]==':'))&&(data_flag==0)) {data_flag=1u;FLAG_CW=1u;}

	if (( InOut[index1]=='.')&&(data_flag2==0)&&(FLAG_DATA==0))   {data_flag2=1u; FLAG_CW=1u;}
	
	if (( InOut[index1]=='$')&&(FLAG_lenght==0u)) {FLAG_lenght=2u;FLAG_CW=1u;}
    
	if ((index1>2u)&&(InOut[2]==' ')&&(FLAG_CW==0u)&&(FLAG_lenght<2u))  
            {
                             if   (data_flag!=1u) {Word[index_word]=InOut[index1];} // заполняем командное слово
                      
                             if  ((data_flag==1u)&&(data_flag2==0u))     DATA_Word[index_data_word]=InOut[index1];// заполняем  слово данных1
                             if  ((data_flag==1u)&&(data_flag2==1u))     DATA_Word2[index_data_word2]=InOut[index1]; // заполняем  слово данных2
                    
                             if  (data_flag!=1u)
                                     {if (index_word<buf_Word) index_word++;} 
                                   else 
                                            {
                                             if ((data_flag==1u)&&(data_flag2==0u)) if (index_data_word<buf_DATA_Word)  {index_data_word++;sch_lenght_data++;}
                                            
                                             if ((data_flag==1u)&&(data_flag2==1u)) if (index_data_word2<buf_DATA_Word) index_data_word2++;
                                            }
			}
	
		if ((FLAG_lenght==2u)&&(FLAG_CW==0u)) {lenght_data = (u8)(InOut[index1]);FLAG_lenght=1u;} //запоминаем длинну пакета данных после ":"
	
		if ((sch_lenght_data<lenght_data)&&(FLAG_lenght==1u)) FLAG_DATA = 1u; else {FLAG_DATA = 0u;}
	 
		if (index1<BUFFER_SR)  index1++;
		if (indexZ <BUFFER_SR)  indexZ ++;
		i--;
		FLAG_CW=0u;
	
  }
 

if (packet_ok==1u) 
  {    
      if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // проверка первого условия пакета - начало пакета
      if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // проверка второго условия пакета - адресат назначения
 
if (crc_ok==0x3)  //обработка команд адресатом которых является хозяин 
{
	
 if (strcmp(Word,"led_ispr")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял led_ispr:",crc_comp); 
	  FPGA_wSPI (8,48,crc_comp);//записываем адресс памяти

   } else
if (strcmp(Word,"CN0led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN0led:",crc_comp); 
      CN0led(crc_comp);
   } else
if (strcmp(Word,"CN1led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN1led:",crc_comp); 
      CN1led(crc_comp);
   } else
if (strcmp(Word,"CN2led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN2led:",crc_comp); 
      CN2led(crc_comp);
   } else
	   
if (strcmp(Word,"CN3led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN3led:",crc_comp); 
      CN3led(crc_comp);
   } else
if (strcmp(Word,"CN4led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN4led:",crc_comp); 
      CN4led(crc_comp);
   } else
if (strcmp(Word,"CN5led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN5led:",crc_comp); 
      CN5led(crc_comp);
   } else
if (strcmp(Word,"CN6led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN6led:",crc_comp); 
      CN6led(crc_comp);
   } else
if (strcmp(Word,"CN7led")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял CN7led:",crc_comp); 
      CN7led(crc_comp);
   } else
  if (strcmp(Word,"stm32_test")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял stm32_test:",crc_comp); 
      ADC_test ();
   } else	
  if (strcmp(Word,"test_delay")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял test_delay:",crc_comp); 
      test_delay (crc_comp);
   } else
   if (strcmp(Word,"test3_sdram")==0) //принимаем код и пишем 1024 значения по адрессу который тоже приняли
   {
	  crc_comp =strtol(DATA_Word ,&ch,16);//адресс в сдрам 
      Transf ("принял test3_sdram\r\n");
      test3_sdram (crc_comp);
   } else
  if (strcmp(Word,"test2_sdram")==0) //контрольное чтение ранее записаной памяти и сравнение с переданным словом - crc_comp
   {
	  crc_comp =strtol(DATA_Word ,&ch,16);//адресс в сдрам 
      Transf ("принял test2_sdram\r\n");
      test2_SDRAM (crc_comp);
   } else
  if (strcmp(Word,"test_sdram")==0) //принимаем код и пишем в sdram
   {
	  crc_comp =strtol(DATA_Word ,&ch,16);//код который пишем
      Transf ("принял test_sdram\r\n");
      test_SDRAM (crc_comp);
   } else
  if (strcmp(Word,"sdram_wr")==0) //принимаем код и пишем 1024 значения по адрессу который тоже приняли
   {
      crc_comp =strtol(DATA_Word ,&ch,16);//адресс в сдрам 
	  crc_input=strtol(DATA_Word2,&ch,16);//данные 
      x_out ("принял sdram_wr:",crc_comp);
	  x_out ("адресс:",crc_input);
      SDRAM_test_wr (crc_comp,crc_input);
   } else

 if (strcmp(Word,"sdram_rd")==0) //
   {
      crc_comp =atoi  (DATA_Word);
      crc_comp =strtol(DATA_Word ,&ch,16);//адресс в sdram
	  crc_input=strtol(DATA_Word2,&ch,16);//колонка в памяти sdram
	  x_out ("принял col:",crc_input);
	  x_out ("принял adr:",crc_comp);
      crc_comp=SDRAM_test_rd   (crc_comp,crc_input);
      x_out(">",crc_comp);
   } else

	if (strcmp(Word,"dac1_reset")==0) //
   {
	crc_comp =atoi(DATA_Word);
	RESETB_DAC_0;
	u_out ("принял dac1_reset:",crc_comp);
	Delay(100);
	RESETB_DAC_1;
	
   } else
if (strcmp(Word,"jesd_dac1_info")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял jesd_dac1_info  :",crc_comp);
	crc_input=FPGA_rSPI (16,70);
	u_out ("число ошибок DAC1_alarm:",crc_input);
	crc_input=FPGA_rSPI (16,71);
	u_out ("число  SYNC_N          :",crc_input);
	crc_input=FPGA_rSPI (8,7);
	u_out ("сигнал       DAC1_alarm:",(crc_input>>3)&0x01);
	
   } else
if (strcmp(Word,"dac1_info")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac1_info:",crc_comp);
	dac1_info (crc_comp);
   } else
if (strcmp(Word,"dac1_serdes_pll")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac1_serdes_pll:",crc_comp);
	alarm_dac1_serdes_pll (crc_comp);
   } else
	if (strcmp(Word,"dac1_init")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac1_init:",crc_comp);
	dac1_write (1);
   } else
	if (strcmp(Word,"dac1_read_reg")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac1_read_reg:",crc_comp);
	dac1_read_reg ();
   } else     
	if (strcmp(Word,"dac2_read_reg")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac2_read_reg:",crc_comp);
	dac2_read_reg ();
   } else   
	if (strcmp(Word,"dac1_r")==0) //
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял dac1_r:",crc_comp);
  x_out ("data:",spiread_dac1(crc_comp));
   } else
	if (strcmp(Word,"dac2_r")==0) //
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял dac2_r:",crc_comp);
  x_out ("data:",spiread_dac2(crc_comp));
   } else
	if (strcmp(Word,"dac1_pll_cp_adj")==0) //
   {    
    crc_comp =atoi(DATA_Word); //
    u_out ("принял dac1_pll_cp_adj:",crc_comp);
	pll_init (dac1.pll_n,dac1.pll_m,dac1.pll_p,dac1.pll_vco,dac1.pll_vcoitune,crc_comp);
   } else    
   if (strcmp(Word,"dac1_pll_vcoitune")==0) //
   {    
    crc_comp =atoi(DATA_Word); //
    u_out ("принял dac1_pll_vcoitune:",crc_comp);
	pll_init (dac1.pll_n,dac1.pll_m,dac1.pll_p,dac1.pll_vco,crc_comp,dac1.pll_cp_adj);
   } else 
	if (strcmp(Word,"dac1_pll_vco")==0) //
   {    
    crc_comp =atoi(DATA_Word); //
    u_out ("принял dac1_pll_vco:",crc_comp);
	pll_init (dac1.pll_n,dac1.pll_m,dac1.pll_p,crc_comp,dac1.pll_vcoitune,dac1.pll_cp_adj);
   } else 
	if (strcmp(Word,"dac1_pll_p")==0) //
   {    
    crc_comp =atoi(DATA_Word); //
    u_out ("принял dac1_pll_p:",crc_comp);
	pll_init (dac1.pll_n,dac1.pll_m,crc_comp,dac1.pll_vco,dac1.pll_vcoitune,dac1.pll_cp_adj);
   } else 
	 if (strcmp(Word,"dac1_pll_m")==0) //
   {    
    crc_comp =atoi(DATA_Word); //
    u_out ("принял dac1_pll_m:",crc_comp);
	pll_init (dac1.pll_n,crc_comp,dac1.pll_p,dac1.pll_vco,dac1.pll_vcoitune,dac1.pll_cp_adj);
   } else   
    if (strcmp(Word,"dac1_pll_n")==0) //
   {    
    crc_comp =atoi(DATA_Word); //
    u_out ("принял dac1_pll_n:",crc_comp);
	pll_init (crc_comp,dac1.pll_m,dac1.pll_p,dac1.pll_vco,dac1.pll_vcoitune,dac1.pll_cp_adj);
   } else
   if (strcmp(Word,"dac1_w")==0) //
   {
    
    crc_comp =atoi(DATA_Word); //адрес регистра
    crc_input=atoi(DATA_Word2);//данные регистра
    un_out ("принял dac1_w:",crc_comp);
    u_out (".",crc_input);
    lb=((crc_comp&0xff)<<16)+(crc_input&0xffff);
    x_out ("code:",lb);
    spisend_dac1(lb);
   } else

if (strcmp(Word,"lmk_sync")==0) //синхронизация LMK
   {
    SYNC_LMK_1;
    Transf ("принял lmk_sync\n\r");
    SYNC_LMK_0;
   } else
if (strcmp(Word,"lmk_w")==0) //
   {
    u32 lb;
    crc_comp =atoi(DATA_Word); //адрес регистра
    crc_input=atoi(DATA_Word2);//данные регистра
    un_out ("принял lmk_w:",crc_comp);
    u_out (".",crc_input);
    lb=(crc_comp<<8)+(crc_input&0xff);
    x_out ("code:",lb);
    spisend_lmk(lb);
   } else

if (strcmp(Word,"lmk_r")==0) //
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял lmk_r:",crc_comp);
  x_out ("data:",spiread_lmk(crc_comp));
   } else
if (strcmp(Word,"lmk_info")==0) //
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял lmk_info:",crc_comp);
  u_out ("OPT_REG_2:",spiread_lmk(0x17d));
  x_out ("OPT_REG_1:",spiread_lmk(0x17c));
  u8 d1=spiread_lmk(0x184);
  u8 d2=spiread_lmk(0x185);
  x_out ("RB_DAC_VALUE:",(d1<<8)+d2);
  x_out ("RB_PLL2_LD:",((spiread_lmk(0x183))>>1)&0x01);
  x_out ("RB_PLL1_LD:",((spiread_lmk(0x182))>>1)&0x01);
  
   } else
if (strcmp(Word,"lmk_CP")==0) //
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял lmk_CP:",crc_comp);
  spisend_lmk((0x169<<8)+(crc_comp<<3)+1);
   } else
if (strcmp(Word,"lmk_test")==0) //
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял lmk_test:",crc_comp);
  //переводим второй тестовый вывод из режима индикации ЗАХВАТА!
  if (crc_comp==1) {Transf("Status_LD2 - 0 !\r\n");        spisend_lmk((0x16e<<8)+(0<<3)+3); Transf("PLL_R\r\n"); spisend_lmk((0x15f<<8)+(17<<3)+3);};
  if (crc_comp==2) {Transf("Status_LD2 - PLL2 DLD !\r\n"); spisend_lmk((0x16e<<8)+(2<<3)+3); Transf("PLL_N\r\n"); spisend_lmk((0x15f<<8)+(13<<3)+3);};
  
   } else
if (strcmp(Word,"init_lmk")==0)                     
   { 
     Transf ("принял init_lmk\r");
     Transf("\r");  
     init_lmk(1);
     SYNC_LMK_1;
     Delay(1);
     SYNC_LMK_0;
	 Delay(10);
	 //RESETB_DAC_0;//сброс ЦАП -ов
	 Delay(1);
	 //RESETB_DAC_1;
   } else
if (strcmp(Word,"help")==0)                     
   {
     Transf ("принял help\r"    );
     Transf("\r");  
     Menu1(0);
   } else
if (strcmp(Word,"help2")==0)                     
   {
     Transf ("принял help\r"    );
     Transf("\r");
	 u_out("timer_DMA2:",timer_DMA2);
   } else
if (strcmp(Word,"info")==0)                     
   {
     Transf ("принял info\r"    );
     Transf("\r");  
     info();
   } else

if (strcmp(Word,"menu")==0)                     
   {
     Transf ("принял menu\r"    );
     Transf("\r");  
     Menu1(0);
   } else
	   
if (strcmp(Word,"custom_phy_wr")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
      
  u_out ("принял custom_phy_wr:",crc_comp);
  FPGA_wSPI (16,23,crc_comp);
                       
    } else 
		
if (strcmp(Word,"sync_wr")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
      
  u_out ("принял sync_wr:",crc_comp);
  FPGA_wSPI (8,88,crc_comp);
                       
    } else 

if (strcmp(Word,"fpga_phy_info")==0) //
   {
  	Transf ("принял fpga_phy_info\r\n");
    crc_input=FPGA_rSPI (24,20);
	x_out("hex:",crc_input);
	x_out("rx_syncstatus      :", crc_input&0x3);
	x_out("rx_errdetect       :",(crc_input>> 2)&0x3);
	x_out("rx_runningdisp     :",(crc_input>> 4)&0x3);
	x_out("rx_disperr         :",(crc_input>> 6)&0x3);
	x_out("rx_patterndetect   :",(crc_input>> 8)&0x3);
	x_out("pll_locked         :",(crc_input>>10)&0x1);
	x_out("rx_signaldetect    :",(crc_input>>11)&0x1);
	x_out("rx_is_lockedtodata :",(crc_input>>12)&0x1);
	x_out("rx_is_lockedtoref  :",(crc_input>>13)&0x1);
	x_out("rx_ready           :",(crc_input>>14)&0x1);
	x_out("tx_ready           :",(crc_input>>15)&0x1);
	x_out("onet1              :",(crc_input>>16)&0x1);
	x_out("onet2              :",(crc_input>>17)&0x1);
	x_out("pll125_locked      :",(crc_input>>18)&0x1);
	x_out("tx_cal_busy        :",(crc_input>>19)&0x1);
	x_out("rx_cal_busy        :",(crc_input>>20)&0x1);
	x_out("pll96_locked       :",(crc_input>>21)&0x1);
	
	crc_input=FPGA_rSPI (16,22);
	
	x_out("rx_data:",crc_input);
	
   } else 
//-------------DAC1-----------------------	   
	   
 if (strcmp(Word,"dac1_dsp_init")==0) //
   {
  crc_comp =atoi(DATA_Word);
      
  u_out ("принял dac1_dsp_init:",crc_comp);
  DAC1_dsp_init(crc_comp);
                       
    } else 
if (strcmp(Word,"dac1_phy_wr")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
      
  u_out ("принял dac1_phy_wr:",crc_comp);
  FPGA_wSPI (32,41,crc_comp);
                       
    } else 	   
if (strcmp(Word,"dac1_phy_info")==0) //
   {
  	Transf ("принял dac1_phy_info\r\n");
	Transf ("---------\r\n");
    crc_input=FPGA_rSPI (8,40);
	//x_out("hex:",crc_input);
	x_out("dac1_datak_align   :", crc_input&0x0f);
	x_out("JESD_DAC1_sync_n   :",(crc_input>> 4)&0x1);
	x_out("tx_ready           :",(crc_input>> 5)&0x1);
	x_out("dac1_pll_locked    :",(crc_input>> 6)&0x1);
	x_out("clk96_locked_D1    :",(crc_input>> 7)&0x1);
	u_out("D1_ALARM:",HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8));
   } else
	   
if (strcmp(Word,"fpga_r")==0) //
   {
  crc_comp =atoi(DATA_Word);//сколько бит
  crc_input=atoi(DATA_Word2);//адресс чтения
  un_out ("принял fpga_r:",crc_comp);
  u_out (".",crc_input);
  x_out ("data:",FPGA_rSPI (crc_comp,crc_input));
   } else
	   
//-------------DAC2-----------------------	   
if (strcmp(Word,"jesd_dac2_info")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял jesd_dac2_info  :",crc_comp);
	crc_input=FPGA_rSPI (16,72);
	u_out ("число ошибок DAC2_alarm:",crc_input);
	crc_input=FPGA_rSPI (16,73);
	u_out ("число  SYNC_N          :",crc_input);
	crc_input=FPGA_rSPI (8,7);
	u_out ("сигнал       DAC2_alarm:",(crc_input>>2)&0x01);
   } else
if (strcmp(Word,"dac2_info")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac2_info:",crc_comp);
	dac2_info (crc_comp);
   } else	   
 if (strcmp(Word,"dac2_dsp_init")==0) //
   {
  crc_comp =atoi(DATA_Word);
      
  u_out ("принял dac2_dsp_init:",crc_comp);
  DAC2_dsp_init(crc_comp);
                       
    } else 
if (strcmp(Word,"dac2_phy_wr")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
      
  u_out ("принял dac2_phy_wr:",crc_comp);
  FPGA_wSPI (32,43,crc_comp);
                       
    } else 	   
if (strcmp(Word,"dac2_phy_info")==0) //
   {
  	Transf ("принял dac2_phy_info\r\n");
	Transf ("---------\r\n");
    crc_input=FPGA_rSPI (8,42);
	//x_out("hex:",crc_input);
	x_out("dac2_datak_align   :", crc_input&0x0f);
	x_out("JESD_DAC2_sync_n   :",(crc_input>> 4)&0x1);
	x_out("tx_ready           :",(crc_input>> 5)&0x1);
	x_out("dac2_pll_locked    :",(crc_input>> 6)&0x1);
	x_out("clk96_locked_D2    :",(crc_input>> 7)&0x1);
	u_out("D2_ALARM:",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
   } else
if (strcmp(Word,"dac2_init")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял dac2_init:",crc_comp);
	dac2_write (1);
   } else
//------------------------------------------------------   
if (strcmp(Word,"dds")==0) //
   {
  u64 code_dds;
  
  crc_comp =atoi(DATA_Word);//
  u_out ("принял dds:",crc_comp);
  code_dds=crc_comp*4294967296/96000000;
  u_out ("code dds:",code_dds);  
  FPGA_wSPI (32,77,code_dds);
   } else
if (strcmp(Word,"fpga_sdram_w")==0) //
   {
  crc_comp =  atoi(DATA_Word); //адресс
  crc_input=strtol(DATA_Word2 ,&ch,16);//strtol принимает строку с базой 16 и конвертит её в int
  u_out ("принял fpga_sdram_w:",crc_comp);
  x_out ("",crc_input);
  FPGA_wSPI (32,crc_comp,crc_input);
                       
    } else
if (strcmp(Word,"fpga_w")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
      
  u_out ("принял fpga_w:",crc_comp);
  u_out ("",crc_input);
  FPGA_wSPI (crc_comp,crc_input,0xaabbccdd);
                       
    } else
if (strcmp(Word,"DAC2_coarse_dac")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял DAC2_coarse_dac:",crc_comp);
	DAC2_coarse_dac (crc_comp);                      
    } else		
if (strcmp(Word,"DAC1_coarse_dac")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял DAC1_coarse_dac:",crc_comp);
	DAC1_coarse_dac (crc_comp);                      
    } else
if (strcmp(Word,"DAC1_QMC_gain")==0) //
   {
	crc_comp =atoi(DATA_Word);
	u_out ("принял DAC1_QMC_gain:",crc_comp);
	DAC1_QMC_gain (crc_comp);                      
    } else
if (strcmp(Word,"DAC1_mixer_gain")==0) //
   {
	crc_comp =atoi(DATA_Word);
    crc_comp=(crc_comp&0x01);
	u_out ("принял DAC1_mixer_gain:",crc_comp);
	DAC1_mixer_gain (crc_comp);                      
    } else
if (strcmp(Word,"DAC2_mixer_gain")==0) //
   {
	crc_comp =atoi(DATA_Word);
    crc_comp=(crc_comp&0x01);
	u_out ("принял DAC2_mixer_gain:",crc_comp);
	DAC2_mixer_gain (crc_comp);                      
    } else
if (strcmp(Word,"DAC1_PWRDN")==0) //
   {
	crc_comp =atoi(DATA_Word);
    PWRDN_DAC1=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял DAC1_PWRDN:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else 
if (strcmp(Word,"DAC2_PWRDN")==0) //
   {
	crc_comp =atoi(DATA_Word);
    PWRDN_DAC2=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял DAC2_PWRDN:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"ADC1_PWRDN")==0) //
   {
	crc_comp =atoi(DATA_Word);
    PWRDN_ADC1=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял ADC1_PWRDN:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"ADC2_PWRDN")==0) //
   {
	crc_comp =atoi(DATA_Word);
    PWRDN_ADC2=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял ADC2_PWRDN:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"DAC1_RST")==0) //
   {
	crc_comp =atoi(DATA_Word);
    RESET_DAC1=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял DAC1_RST:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"DAC2_RST")==0) //
   {
	crc_comp =atoi(DATA_Word);
    RESET_DAC2=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял DAC2_RST:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"ADC1_RST")==0) //
   {
	crc_comp =atoi(DATA_Word);
    RESET_ADC1=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял ADC1_RST:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"ADC2_RST")==0) //
   {
	crc_comp =atoi(DATA_Word);
    RESET_ADC2=(crc_comp&0x01);
	tmp1=TMP_func();
	u_out ("принял ADC2_RST:",crc_comp);
	FPGA_wSPI (8,127,tmp1);                      
    } else
if (strcmp(Word,"upr_switch")==0) //
   {
  crc_comp =atoi(DATA_Word);
    
  u_out ("принял upr_switch:",crc_comp);
  FPGA_wSPI (8,5,crc_comp);
                       
    } else 	
if (strcmp(Word,"upr_at1")==0) //
   {
  crc_comp =atoi(DATA_Word);
    
  u_out ("принял upr_at1:",crc_comp);
  FPGA_wSPI (8,1,crc_comp);                       
    } else
if (strcmp(Word,"upr_at2")==0) //
   {
  crc_comp =atoi(DATA_Word);
    
  u_out ("принял upr_at2:",crc_comp);
  FPGA_wSPI (8,2,crc_comp);                       
    } else
if (strcmp(Word,"upr_at3")==0) //
   {
  crc_comp =atoi(DATA_Word);
    
  u_out ("принял upr_at3:",crc_comp);
  FPGA_wSPI (8,3,crc_comp);                       
    } else
if (strcmp(Word,"upr_at4")==0) //
   {
  crc_comp =atoi(DATA_Word);
    
  u_out ("принял upr_at4:",crc_comp);
  FPGA_wSPI (8,4,crc_comp);                       
    } else		
             
if (strcmp(Word,"test_spi_read")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
        
  u_out ("принял test_spi_read:",crc_comp);
  test_spi_read(crc_comp);
                       
   } else  
//-------------------ADC2------------------------------	   
if (strcmp(Word,"adc2_read_reg")==0) //
   {
	u_out ("принял adc2_read_reg:",0);
	adc2_read_reg();
                       
   } else 
if (strcmp(Word,"adc2_init")==0) //
   {
	u_out ("принял adc2_init:",0);
	adc2_write (0);
                       
   } else 
if (strcmp(Word,"adc2_test")==0) //управляет режимом выдачи АЦП - тест или сигнал
   {
    crc_comp =atoi(DATA_Word);
	u_out ("принял adc2_test:",crc_comp);
	adc2_test(crc_comp);                      
   } else   
if (strcmp(Word,"adc2_fifo_wr_en")==0) //управляет заполнением fifo ацп-плис
   {
    crc_comp =atoi(DATA_Word);
	u_out ("принял adc2_fifo_wr_en:",crc_comp);
	FPGA_wSPI (8,59,crc_comp); 
    if (crc_comp==0) 
	{ 
		adc2_fifo_read(1,0);
		adc2_fifo_read(1,0);
		adc2_fifo_read(1,0);
		adc2_fifo_read(0,0);
		adc2_fifo_read(0,0);
		adc2_fifo_read(0,0);
		FPGA_wSPI (8,59,1);
		delay_us(50000);
		FPGA_wSPI (8,59,0);		
    }
   } else
if (strcmp(Word,"adc2_fifo_read_burst")==0) //
   {
    crc_comp =atoi(DATA_Word);
	//u_out ("принял adc2_fifo_read_burst:",crc_comp);
	adc2_fifo_read(crc_comp,1);                       
   } else
if (strcmp(Word,"adc2_fifo_read")==0) //
   {
	u8 adr_adc=0;   
	crc_comp =atoi(DATA_Word);   
	u_out ("принял adc2_fifo_read:",crc_comp);
	if (crc_comp==0) adr_adc=57; else if (crc_comp==1) adr_adc=58;
	crc_input=FPGA_rSPI (16,adr_adc);
	x_out("fifo:",crc_input);
                       
   } else
if (strcmp(Word,"adc2_phy_info")==0) //
   {
	Transf ("\r\n");
  	Transf ("принял adc2_phy_info\r\n");
    crc_input=FPGA_rSPI (32,50);
	//x_out("hex:",crc_input);
	x_out("rx_runningdisp_adc2:", crc_input>>24);
	x_out("rx_datak_adc2      :",(crc_input>>16)&0xff);
	x_out("rx_disperr_adc2    :",(crc_input>> 8)&0xff);
	x_out("rx_errdetect_adc2  :",(crc_input>> 0)&0xff);

	crc_input=FPGA_rSPI (24,54);	
	x_out("align_ok_adc2        :",(crc_input>>19)&0x0f);	
	x_out("rx_ready_adc2        :",(crc_input>>18)&0x01);
	x_out("sync_n_adc2          :",(crc_input>>17)&0x01);
	x_out("clk240_locked        :",(crc_input>>16)&0x01);
	x_out("rx_patterndetect_adc2:",(crc_input>> 8)&0xff);
	x_out("rx_syncstatus_adc2   :",(crc_input>> 0)&0xff);
	
	crc_input=FPGA_rSPI ( 8,65);
	x_out("align_adc2           :",crc_input);
	
	crc_input=FPGA_rSPI (16,60);
	x_out("error_adc2           :",crc_input);
	
	crc_input=FPGA_rSPI (16,61);
	x_out("error_sysref_adc2    :",crc_input);

	crc_input=FPGA_rSPI (24,55);	
	x_out("bitslipboundary:",crc_input);

	
	v1=FPGA_rSPI (64,53);
	z1=v1>>32;
	z2=v1;
	x_out("adc2_data_tst1[63:32]:",z1);
	x_out("adc2_data_tst1[31: 0]:",z2);
	
	v1=FPGA_rSPI (64,56);	
	z1=v1>>32;
	z2=v1;
	x_out("adc2_data_tst2[63:32]:",z1);
	x_out("adc2_data_tst2[31: 0]:",z2);
	Transf ("\r\n");  
	  } else
//-------------------ADC1------------------------------	   
if (strcmp(Word,"adc1_read_reg")==0) //
   {
	u_out ("принял adc1_read_reg:",0);
	adc1_read_reg();
                       
   } else 
if (strcmp(Word,"adc1_init")==0) //
   {
	u_out ("принял adc1_init:",0);
	adc1_write (0);
                       
   } else 
if (strcmp(Word,"adc1_test")==0) //управляет режимом выдачи АЦП - тест или сигнал
   {
    crc_comp =atoi(DATA_Word);
	u_out ("принял adc1_test:",crc_comp);
	adc1_test(crc_comp);                      
   } else 
if (strcmp(Word,"adc1_fifo_uart_en")==0) //управляет выводом данных из fifo в уарт плис
   {
    crc_comp =atoi(DATA_Word);
	u_out ("принял adc1_fifo_uart_en:",crc_comp);
	FPGA_wSPI (8,111,crc_comp); 
   } else
if (strcmp(Word,"adc1_fifo_upr")==0) //управляет выводом данных из fifo в уарт плис или SPI mk
   {
    crc_comp =atoi(DATA_Word);
	u_out ("принял adc1_fifo_upr:",crc_comp);//0x10(16) - adc0_0 ;0x11(17) - adc0_1;0x12(18) - adc1_0;0x13(19) - adc1_1;
	FPGA_wSPI (8,114,crc_comp); 
   } else
	   
if (strcmp(Word,"adc1_fifo_wr_en")==0) //управляет заполнением fifo ацп-плис
   {
    crc_comp =atoi(DATA_Word);
	u_out ("принял adc1_fifo_wr_en:",crc_comp);
	FPGA_wSPI (8,109,crc_comp); 
    if (crc_comp==0) 
	{ 
		adc1_fifo_read(1,0);
		adc1_fifo_read(1,0);
		adc1_fifo_read(1,0);
		adc1_fifo_read(0,0);
		adc1_fifo_read(0,0);
		adc1_fifo_read(0,0);
		FPGA_wSPI (8,109,1);
		delay_us(5000);
		FPGA_wSPI (8,109,0);
		
    }
   } else
if (strcmp(Word,"adc1_fifo_read_burst")==0) //
   {
    crc_comp =atoi(DATA_Word);
	//u_out ("принял adc1_fifo_read_burst:",crc_comp);
	adc1_fifo_read(crc_comp,1);                       
   } else
if (strcmp(Word,"adc1_fifo_read")==0) //
   {
	u8 adr_adc=0;   
	crc_comp =atoi(DATA_Word);   
	u_out ("принял adc1_fifo_read:",crc_comp);
	if (crc_comp==0) adr_adc=107; else if (crc_comp==1) adr_adc=108;
	crc_input=FPGA_rSPI (16,adr_adc);
	x_out("fifo:",crc_input);
                       
   } else
if (strcmp(Word,"adc1_phy_info")==0) //
   {
	Transf ("\r\n");
  	Transf ("принял adc1_phy_info\r\n");
    crc_input=FPGA_rSPI (32,100);
	//x_out("hex:",crc_input);
	x_out("rx_runningdisp_adc1:", crc_input>>24);
	x_out("rx_datak_adc1      :",(crc_input>>16)&0xff);
	x_out("rx_disperr_adc1    :",(crc_input>> 8)&0xff);
	x_out("rx_errdetect_adc1  :",(crc_input>> 0)&0xff);

	crc_input=FPGA_rSPI (24,104);
	x_out("align_ok_adc1        :",(crc_input>>19)&0x0f);	
	x_out("rx_ready_adc1        :",(crc_input>>18)&0x01);
	x_out("sync_n_adc1          :",(crc_input>>17)&0x01);
	x_out("clk240_locked        :",(crc_input>>16)&0x01);
	x_out("rx_patterndetect_adc1:",(crc_input>> 8)&0xff);
	x_out("rx_syncstatus_adc1   :",(crc_input>> 0)&0xff);
	
	crc_input=FPGA_rSPI (8,113);
	x_out("align_adc1           :",crc_input);
	
	crc_input=FPGA_rSPI (16,110);
	x_out("error_adc1           :",crc_input);
	
	crc_input=FPGA_rSPI (16,112);
	x_out("error_sysref_adc1    :",crc_input);

	crc_input=FPGA_rSPI (24,105);	
	x_out("bitslipboundary:",crc_input);

	
	v1=FPGA_rSPI (64,103);
	z1=v1>>32;
	z2=v1;
	x_out("adc1_data_tst1[63:32]:",z1);
	x_out("adc1_data_tst1[31: 0]:",z2);
	
	v1=FPGA_rSPI (64,106);	
	z1=v1>>32;
	z2=v1;
	x_out("adc1_data_tst2[63:32]:",z1);
	x_out("adc1_data_tst2[31: 0]:",z2);
	Transf ("\r\n");
   } else

if (strcmp(Word,"adc0_start")==0) //
   {    
	u_out ("принял adc0_start:",crc_input);
	adc_init (0);	
   }else
      
if (strcmp(Word,"test_spi_write")==0) //
   {
  crc_comp =atoi(DATA_Word);
  crc_input=atoi(DATA_Word2);
       
  u_out ("принял test_spi_write:",crc_comp);
  test_spi_write(crc_comp);
                       
   } else
//----------------i2c SFP--------------------
if (strcmp(Word,"i2c_write")==0) // ~0 i2c_write:adr.data;
   {
  crc_comp =atoi(DATA_Word);
  crc_input=strtol(DATA_Word2 ,&ch,16);;
  un_out ("принял i2c_write:",crc_comp);
  x_out (".",crc_input);
  //ccc=(0u<<31)|(((0x5600u|(crc_comp&0xff))&0xffff)<<16)|(crc_input&0xffff);//0 - WRITE 1 - READ
  //x_out("ccc:",ccc);
  //FPGA_wSPI (32,10,ccc);//записываем в блок i2c {бит записи|адрес регистра|байт данных}  
    i2c_write (crc_comp,crc_input);                   
   } else
if (strcmp(Word,"i2c_read")==0) // ~0 i2c_read:adr.data;
   {
  crc_comp =atoi(DATA_Word);
  u_out ("принял i2c_read:",crc_comp); 
//  ccc=(1u<<31)|(((0x5600u|(crc_comp&0xff))&0xffff)<<16)|(0x0000&0xffff);//0 - WRITE 1 - READ
//  x_out("ccc:",ccc);
//  FPGA_wSPI (32,10,ccc);//считываем из блока i2c слово{бит записи|адрес регистра|байт данных} 
//  Delay(10);
//  crc_input=FPGA_rSPI (24,11);
//  x_out("i2c data:",crc_input);
     crc_input=i2c_read (crc_comp);                   
   } else
if (strcmp(Word,"sfp_read")==0) //чтение содержимого памяти модуля SFP
   {
	  Transf("принял sfp_read:\r\n");
	  Transf("---------------\r\n");
	  SFP_read();
	  
   } else
if (strcmp(Word,"sfp_reset")==0) //чтение содержимого памяти модуля SFP
   {
	  Transf("принял sfp_reset:\r\n");
	  
	  ccc=IO("~0 i2c_read:0.0;");//читаем содержимое регистра 0 , I2C 0xAC;
//	  x_out("reg0:",ccc);
	  ccc=(0<<31)+((0x5600+(0x00&0xff))<<16)+((ccc|0x8000|0x1200)&0xffff);//0 - WRITE 1 - READ
//	  x_out("ccc:",ccc);
	  FPGA_wSPI (32,10,ccc);
	  	  
   } else
if (strcmp(Word,"sfp_init")==0) //чтение содержимого памяти модуля SFP
   {
	  Transf("принял sfp_init:\r\n");
	     sfp_init();
	  	  
   } else	

//-------------ETH---------------------
if (strcmp(Word,"mac0_config_reg_rd")==0) //чтение содержимого памяти MAC
   {
	   
//	  crc_comp =atoi(DATA_Word);
	  crc_comp =strtol(DATA_Word,&ch,16);//strtol принимает строку с базой 16 и конвертит её в int
      x_out ("принял (hex) mac0_config_reg_rd:",crc_comp);
	          FPGA_wSPI (8,13,crc_comp);//записываем адресс конфигурационного регистра
	crc_input=FPGA_rSPI (32,14);//считываем содержимое конфигурационного регистра
		x_out(">",crc_input);
	   
   } else	
if (strcmp(Word,"mac1_config_reg_rd")==0) //чтение содержимого памяти MAC
   {
	   
//	  crc_comp =atoi(DATA_Word);
	  crc_comp =strtol(DATA_Word,&ch,16);//strtol принимает строку с базой 16 и конвертит её в int
      x_out ("принял (hex) mac1_config_reg_rd:",crc_comp);
	          FPGA_wSPI (8,122,crc_comp);//записываем адресс конфигурационного регистра
	crc_input=FPGA_rSPI (32,120);//считываем содержимое конфигурационного регистра
		x_out(">",crc_input);
	   
   } else
if (strcmp(Word,"mac0_config_reg_wr")==0) //чтение содержимого памяти модуля SFP
   {
	   crc_comp =strtol(DATA_Word ,&ch,16);//strtol принимает строку с базой 16 и конвертит её в int
       crc_input=strtol(DATA_Word2,&ch,16);
	   xn_out ("принял (hex) mac0_config_reg_wr:",crc_comp);
        x_out (".",crc_input);
	    FPGA_wSPI ( 8,13,crc_comp);//записываем адресс конфигурационного регистра
		FPGA_wSPI (32,12,crc_input);//записываем конфигурационный регистр
		   
   } else
if (strcmp(Word,"mac1_config_reg_wr")==0) //чтение содержимого памяти модуля SFP
   {
	   crc_comp =strtol(DATA_Word ,&ch,16);//strtol принимает строку с базой 16 и конвертит её в int
     crc_input=strtol(DATA_Word2,&ch,16);
	   xn_out ("принял (hex) mac1_config_reg_wr:",crc_comp);
        x_out (".",crc_input);
	    FPGA_wSPI ( 8,122,crc_comp);//записываем адресс конфигурационного регистра
		FPGA_wSPI (32,121,crc_input);//записываем конфигурационный регистр
		   
   } else
if (strcmp(Word,"mac_signal")==0) //чтение содержимого памяти модуля SFP
   {
	  Transf("принял mac_signal:\r\n");
	    crc_input=FPGA_rSPI (16,9);//считываем содержимое 
		x_out("DATA:",crc_input);   
   } else
if (strcmp(Word,"arp_test")==0) //чтение содержимого памяти модуля SFP
   {
	  Transf("принял arp_test:\r\n");
	    crc_input=FPGA_rSPI (32,9);//считываем содержимое 
		x_out("DATA:",crc_input);   
   } else	   
if (strcmp(Word,"mac_init")==0) //инициализация МАС
   {
	  x_out("принял mac_init:",crc_comp);
	  crc_comp =strtol(DATA_Word ,&ch,16);
	  if (crc_comp==0) MAC0_init(); 
	  if (crc_comp==1) MAC1_init();	  
   } else	   
if (strcmp(Word,"mac_statistic")==0) //инициализация МАС
   {
	  x_out("принял mac_statistic:",crc_comp);
	   crc_comp =strtol(DATA_Word ,&ch,16);
	   if (crc_comp==0) MAC0_statistic();
	   if (crc_comp==1) MAC1_statistic();
   } else
if (strcmp(Word,"pcs_status")==0) //инициализация МАС
   {
	  x_out("принял pcs_status:",crc_comp);
	  crc_comp =strtol(DATA_Word ,&ch,16);
	  if (crc_comp==0)  PCS0_status();// 
	  if (crc_comp==1)  PCS1_status();// 
   } else
if (strcmp(Word,"init_MAC0_data")==0) //инициализация МАС
   {
	   crc_comp =atoi(DATA_Word);
	  x_out("принял init_MAC0_data:",crc_comp);
	  FPGA_wSPI (8,15,crc_comp);
   } else
if (strcmp(Word,"init_MAC1_data")==0) //инициализация МАС
   {
	   crc_comp =atoi(DATA_Word);
	  x_out("принял init_MAC1_data:",crc_comp);
	  FPGA_wSPI (8,123,crc_comp);
   } else
if (strcmp(Word,"udp_rcv")==0) //
   {
	   crc_comp =atoi(DATA_Word);
	  x_out("принял udp_rcv:",crc_comp);
	  UDP_mem_read(crc_comp);
   } else
if (strcmp(Word,"udp_buf")==0) //
   {
	   crc_comp =atoi(DATA_Word);
	  x_out("принял udp_buf:",crc_comp);
	  UDP_buf_read();
   } else
if (strcmp(Word,"udp_tx_init")==0) //
   {
	   crc_comp =atoi(DATA_Word);
	  x_out("принял udp_tx_init:",crc_comp);
	  UDP_tx_init();
   } else
//-------------------------------------------
if (strcmp(Word,"temp_fpga")==0) //
   {
	int temp;
	temp=temp_fpga ();        
	d_out ("принял temp_fpga:",temp);                 
   } 
      } 
	  for (i=0u;i<buf_Word;i++)               Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)   DATA_Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)  DATA_Word2[i]     =0x0;  
      for (i=0u;i<BUFFER_SR;i++)  
      {
        InOut[i]     =0x0;
      }  
      
	  time_uart=0;  //обнуление счётчика тайм аута
      packet_flag=0; 
      index1=0u; 
      crc_ok=0; 
      i=0;
      packet_ok=0; 
      index_word=0u; 
      index_data_word=0u;
      data_flag=0;
      index_data_word2=0u;
      data_flag2	 =0;
      indexZ 		 =0u;
      FLAG_lenght    =0u;
      lenght_data    =0u;
      sch_lenght_data=0u;
      FLAG_CW   = 0u; //флаг управляющего байта, снимается сразу после исполнения
      FLAG_DATA = 0u;	  
      	  
      DATA_Word [0]=' ';
      DATA_Word2[0]=' ';
	  SCH_LENGHT_PACKET=0;
  }

  if ((packet_ok==1)&&(crc_ok==0x1))     //обработка команд адресатом которых является слейв

  {
    
    if (Master_flag==0)

      {            
         
      }
  }  
  return  crc_input;         
} 

void Menu1(char a) 
 {
//***************************************************************************
    int i;  
 
  for (i=0;i<20;i++) Transf("\r");    // очистка терминала
  for (i=0; i<20; i++) Transf ("-");  // вывод приветствия
  Transf("\r");
  Transf("..........Terminal Тестовой платы.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("Расшифровка структуры команды:\r");
  Transf("~ - стартовый байт\r");
  Transf("1 - адрес абонента\r");
  Transf(";- конец пачки \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("IP  :192.168.1.163 - IP адрес    блока\r");
  Transf("PORT:2054          - номер порта блока\r");
  Transf("~0 help; - текущее меню\r");
  Transf("~0 info; - информация \r");
  Transf("~0 dac1_init; - \r");
  Transf("~0 dac1_r:0;   - чтение регистра\r");
  Transf("~0 dac1_w:0.0; - запись регистра\r");
  Transf("~0 dac1_serdes_pll:1; - очистка регистра сигнала захвата PLL Serdes\r");
  Transf("~0 dac1_info:0; \r");
  Transf("~0 dac1_init:0; \r");
  Transf("~0 dac1_phy_wr:0; \r");
  Transf("~0 dac1_phy_info; \r");
  Transf("~0 lmk_sync; - sync на LMK\r");
  Transf("~0 init_lmk; - init на LMK\r");
  Transf("-------------------------------------------\r");
  Transf("\r");
  Transf("\r");
  Transf("++++++++++++++++++++\r");
  Transf("\r");
  Transf("\r");
  //for (i=0; i<64; i++) zputs ("*",1);  // вывод приветствия
  //for (i=0;i<10;i++) puts("\r",1);  // очистка терминала
  Transf("\r");
  //*******************************************************************************
}


char getchar1(void)
{
   uint8_t data;
   while (rx_counter1 == 0);
   data = rx_buffer1[ rx_rd_index1++ ];
   if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1 = 0;
    --rx_counter1;
    return data;
}
      
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 //-------------------------  
   rx_buffer1[rx_wr_index1++]= (uint8_t) (RX_uBUF[0]& 0xFF); //считываем данные в буфер, инкрементируя хвост буфера
   if ( rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0; //идем по кругу
   	 
	  if (++rx_counter1 == RX_BUFFER_SIZE1) //переполнение буфера
      {
        rx_counter1=0; //начинаем сначала (удаляем все данные)
        rx_buffer_overflow1=1;  //сообщаем о переполнении
      }
	  
 //--------------------------  
   HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
}



void UART_conrol (void)
{
 u16 i=0;
 u16 j=0;

  if (rx_counter1!=0u)
    {   
      if (rx_counter1<BUFFER_SR) j = rx_counter1; else j=BUFFER_SR;

      for (i=0u;i<j; i++) 
         {
           sr[i]=getchar1();
           lenght=i+1;  
           if (sr[i]==';') {break;}
          }
            sr[lenght]=0x00;
            IO (sr);
        };
}

void SDRAM_test_wr (u32 adr,u16 a) //пишет 1024 значения по адресу
{
	u16 i=0;
	u32 z=0;
	
	for (i=0;i<1023;i++)
	{
		z=(i<<16)+a;
		FPGA2_wSPI (32,33,z);//записываем адресс и данные в память МЕМ , перед SDRAM
	}
	
    FPGA2_wSPI (32,20,adr<<10);//записываем адресс в SDRAM, куда пойдут данные
	
//  Delay(2);//1mc
	WR(1);//посылаем сигнал записи из МЕМ в SDRAM (в ПЛИС)
	delay_us(10);
	WR(0);
}


u16 SDRAM_test_rd (u32 adr,u16 col) //adr верхняя часть адреса 
{
	u16 z=0;
	
	FPGA2_wSPI (32,20,adr<<10);//записываем адресс в SDRAM откуда читаем 1024 отсчёта
	
	delay_us(50);
	RD(1);//посылаем сигнал чтения из SDRAM в МЕМ (в ПЛИС)
	delay_us(10);
	RD(0);
	delay_us(50);
	  FPGA2_wSPI (32,20,col);//записываем адресс для памяти МЕМ , куда скопированы данные из SDRAM
	z=FPGA2_rSPI (16,32);    //считываем содержимое МЕМ по вышезаписанному адрессу
	return z;
}

#define Array 300
#define RAM_size 1023

void test_SDRAM (u16 test)
{
 u32 i=0;//page+bank
 u32 j=0;//col
 u32 k=0;
 u32 flag=0;
 u16 data=0;
 u16 data_err=0;
 u32 error=0;
 u32 l1=0;
 u32 l2=0;
 
 Transf("\r\nПишем  SDRAM\r\n");	
 
 for (i=0;i<Array;i++)
 {
	SDRAM_test_wr (i,test);
	Transf(".");
	UART_DMA_TX();
 }
 
 Delay(5);//задержка
 Transf("\r\nЧитаем SDRAM\r\n");

  for (i=0;i<Array;i++)
 {
	FPGA2_wSPI (32,20,i<<10);//записываем адресс в SDRAM откуда читаем 1024 отсчёта
	
//	delay_us(50);
	RD(1);//посылаем сигнал чтения из SDRAM в МЕМ (в ПЛИС)
//	delay_us(10);
	RD(0);
//	Delay(2);
	Transf(".");
	for (j=0;j<RAM_size;j++)
	{
	       FPGA2_wSPI (32,20,j);  //записываем адресс для памяти МЕМ , куда скопированы данные из SDRAM
	  data=FPGA2_rSPI (16,32);    //считываем содержимое МЕМ по вышезаписанному адрессу
	  
	  if ((data!=test)) //&&(j>16)
	  {
		  
		  if ((j==(k+1))&&(flag==0)) 
		  {
			  Transf(" ... ");
			  flag=1;
		  } else
			  if ((j==(k+1))&&(flag==1))
			  {
				  
			  }else
				  if (j!=(k+1))
				  {
				   Transf("\r");
				   xn_out("i:",i);
				   Transf(" ");
				   xn_out("j:",j);
				   xn_out(" ",data);
				   
				   flag=0;
				  }
	   
	   data_err=data;		  
	   error++;
	   k=j;
	  } else
	  {
		  if (flag==1)
		  {
			  if (j!=0) l2=j-1; else {l2=RAM_size;l1=i-1;}
			  xn_out("i:",l1);
			  Transf(" ");
			  xn_out("j:",l2);
			  x_out(" ",data_err);
			 // Transf("\r");  
			  flag=0;
		  }
	  }
	  UART_DMA_TX();
	}	
 } 
 u_out("\r\nЧисло ошибок             :",error); 
}



void test2_SDRAM (u16 test)
{
 u32 i=0;//page+bank
 u32 j=0;//col
 u16 data=0;
 u16 data_err=0;
 u32 error=0;
 u32 k=0;
 u32 l1=0;
 u32 l2=0;
 u32 flag=0;
  
 Transf("\r\nЧитаем SDRAM\r\n");

  for (i=0;i<Array;i++)
 {
	FPGA2_wSPI (32,20,i<<10);//записываем адресс в SDRAM откуда читаем 1024 отсчёта
	
//	delay_us(50);
	RD(1);//посылаем сигнал чтения из SDRAM в МЕМ (в ПЛИС)
//	delay_us(10);
	RD(0);
//	Delay(2);
	Transf(".");
	for (j=0;j<RAM_size;j++)
	{
	       FPGA2_wSPI (32,20,j);  //записываем адресс для памяти МЕМ , куда скопированы данные из SDRAM
	  data=FPGA2_rSPI (16,32);    //считываем содержимое МЕМ по вышезаписанному адрессу
	  
	  if ((data!=test)) //&&(j>16)
	  {
		  
		  if ((j==(k+1))&&(flag==0)) 
		  {
			  Transf(" ... ");
			  flag=1;
		  } else
			  if ((j==(k+1))&&(flag==1))
			  {
				  
			  }else
				  if (j!=(k+1))
				  {
				   Transf("\r");
				   xn_out("i:",i);
				   Transf(" ");
				   xn_out("j:",j);
				   xn_out(" 0x",data);  
				   flag=0;				
				  }
				  
	   data_err=data;
	   error++;
	   k=j;
	  } else
	  {
		  if (flag==1)
		  {
			  if (j!=0) l2=j-1; else {l2=RAM_size;l1=i-1;}
			  xn_out("i:",l1);
			  Transf(" ");
			  xn_out("j:",l2);
			  x_out(" ",data_err);
		   // Transf("\r");  
			  flag=0;
		  }
	  }
	  UART_DMA_TX();
	}	
 } 
 u_out("\r\nЧисло ошибок             :",error);  
}

void test3_sdram(u16 test)
{

}


void LED (void)
{
	static u8 z=1;
	
	if ((TIMER1<100)&&(FLAG_T1==0)) 
	{
		LED1_0;
		LED2_1;
		LED3_0;
		
		FLAG_T1=1;
		FLAG_T2=0;
		if (z!=0) z=z<<1; else z=1;
	}
	
	if ((TIMER1>200)&&(FLAG_T2==0)) 
	{
		LED1_0;
		LED2_0;
		LED3_1;
		
		FLAG_T2=1;
		FLAG_T1=0;
		sch_rx_byte=0;
	}
	
	if ((TIMER1>400))
	{
		LED1_1;
		LED2_0;
		LED3_0;
	}

//	 CN0led((z>>0)&0x01);
//	 CN1led((z>>1)&0x01);
//   CN2led((z>>2)&0x01);
//   CN3led((z>>3)&0x01);
//   CN4led((z>>4)&0x01);
//   CN5led((z>>5)&0x01);
//   CN6led((z>>6)&0x01);
//   CN7led((z>>7)&0x01);

	if (TIMER1>500)	TIMER1=0;
	
}

void FAPCH_INIT (void)
{
 if ((timer_INIT_FAPCH1>1000)&&(FLAG_FAPCH_ON==0))  {init_FAPCH (1);FLAG_FAPCH_ON=1;}
    else
    {
      if (FLAG_FAPCH_ON==0)
      {
       //LED1_off;
      }
    }	
}

void ATT_upr (u8 adr,u8 atten)
{
	u8 Adress_ATT=0;
	
	if (adr==1) Adress_ATT=1;
	if (adr==2) Adress_ATT=2;
	if (adr==3) Adress_ATT=3;
	if (adr==4) Adress_ATT=4;
	
nu_out("ATT",adr);
 u_out(":",atten);	
	
spisend_FPGA (Adress_ATT,atten);
}

u8 PIN_control (void)
{
  static u8 pn_old;
  u8 pn;
  u8 flag;
  pn=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
  if (pn_old!=pn) {pn_old=pn;flag=1;} else flag=0;
  return flag;
}

u8 PIN_control_PA8 (void)
{
  static u8 pn_old;
  u8 pn;
  u8 flag;
  pn=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
  if (pn_old!=pn) {pn_old=pn;flag=1;} else flag=0;
  return flag;
}

void test_delay (u32 a)
{
	Transf("Начали!\r\n");
	Delay(a);
	Transf("Конец!\r\n");
}

volatile  u32 adc_cntl=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)

{
	adc_cntl++;
//	HAL_ADC_Start_IT(hadc1);
}

float DBm[335];

void Massiv_dbm(void)
{
u16 i;
float e;
 DBm[  9]=-25;
 DBm[ 10]=-24;
 DBm[ 11]=-23;
 DBm[ 12]=-22;
 DBm[ 13]=-21;
 DBm[ 15]=-20;
 DBm[ 24]=-15;
 DBm[ 41]=-10;
 DBm[ 70]=-5;
 DBm[130]= 0;
 DBm[230]= 5;
 DBm[330]= 8;

for (i=  0;i<  9;i++) { e=-600+(i- 9)*6  ;   DBm[i]= e;}
for (i=  9;i< 16;i++) { e=-250+(i- 9)*8.3;   DBm[i]= e;}
for (i= 15;i< 25;i++) { e=-200+(i-15)* 5;    DBm[i]= e;}
for (i= 24;i< 42;i++) { e=-150+(i-24)*2.7;   DBm[i]= e;}
for (i= 41;i< 71;i++) { e=-100+(i-41)*1.7;   DBm[i]= e;}
for (i= 70;i<131;i++) { e=-50 +(i- 70)*0.83; DBm[i]= e;}
for (i=130;i<231;i++) { e=-0 +(i-130)*0.5;   DBm[i]= e;}
for (i=230;i<334;i++) { e= 50 +(i-230)*0.29; DBm[i]= e;}

for (i=0;i<334;i++) 
         {
           DBm[i]= DBm[i]/10;
//		   un_out("DBm[",i);
//	       f_out("]=",DBm[i]);
         }
}

double ADC_Temp(double t)
{
double TCelsius;	
TCelsius = ((t - 0.760) / 0.0025) + 25.0 ;
return TCelsius;
} 

void ADC_test (void)
{
//	u32 i=0;

   int z=0;
   double d=0;
	
/*	
	Transf("\r\n---------\r\n");
	u_out("adc0_2.5V     :",adcBuffer[0 ]);
	u_out("adc3_1.8V     :",adcBuffer[1 ]);
	u_out("adc4_0.9V     :",adcBuffer[2 ]);
	u_out("adc5_1.8V     :",adcBuffer[3 ]);
	u_out("adc6_1.8V     :",adcBuffer[4 ]);
//	u_out("adc7_xxV      :",adcBuffer[5 ]);//error
//	u_out("adc9_xxV      :",adcBuffer[6 ]);//error
	u_out("adc10_D1_ATEST:",adcBuffer[5 ]);
	u_out("adc11_D2_ATEST:",adcBuffer[6 ]);
	u_out("adc12_REF     :",adcBuffer[7]);
	u_out("adc13_2.5V    :",adcBuffer[8]);
	u_out("adc14_3.3V    :",adcBuffer[9]);
	u_out("adc15_5.0V    :",adcBuffer[10]);
	u_out("temp_sens     :",adcBuffer[11]);	
//	u_out("adc_cntl      :",adc_cntl);

*/
	adc_cntl=0;	
	
	adc_ch[ 0] = adcBuffer[ 0]*3300*2/1024;
	adc_ch[ 1] = adcBuffer[ 1]*3300*1/1024;
	adc_ch[ 2] = adcBuffer[ 2]*3300*1/1024;
	adc_ch[ 3] = adcBuffer[ 3]*3300*1/1024;
	adc_ch[ 4] = adcBuffer[ 4]*3300*1/1024;
	adc_ch[ 5] = adcBuffer[ 5]*3300*1/1024;
	adc_ch[ 6] = adcBuffer[ 6]*3300*1/1024;
	adc_ch[ 7] = adcBuffer[ 7]*3300*1/1024;
	adc_ch[ 8] = adcBuffer[ 8]*3300*2/1024;
	adc_ch[ 9] = adcBuffer[ 9]*3300*2/1024;
	adc_ch[10] = adcBuffer[10]*3300*2/1024;
	adc_ch[11] = adcBuffer[11]*3300*1/1024;
	
	adc_ch[ 0] = adc_ch[ 0]/1000;
	adc_ch[ 1] = adc_ch[ 1]/1000;
	adc_ch[ 2] = adc_ch[ 2]/1000;
	adc_ch[ 3] = adc_ch[ 3]/1000;
	adc_ch[ 4] = adc_ch[ 4]/1000;
	adc_ch[ 5] = adc_ch[ 5]/1000;
	adc_ch[ 6] = adc_ch[ 6]/1000;
	adc_ch[ 7] = adc_ch[ 7]/1000;
	adc_ch[ 8] = adc_ch[ 8]/1000;
	adc_ch[ 9] = adc_ch[ 9]/1000;
	adc_ch[10] = adc_ch[10]/1000;
	adc_ch[11] = adc_ch[11]/1000;
	
	Transf("\r\n---------\r\n");
	f_out("adc0_5.0V     :",adc_ch[0 ]);
	f_out("adc3_1.8V     :",adc_ch[1 ]);
	f_out("adc4_0.9V     :",adc_ch[2 ]);
	f_out("adc5_1.8V     :",adc_ch[3 ]);
	f_out("adc6_1.8V     :",adc_ch[4 ]);
//	f_out("adc7_xxV      :",adc_ch[5 ]);//error
//	f_out("adc9_xxV      :",adc_ch[6 ]);//error
	f_out("adc10_D1_ATEST:",adc_ch[5 ]);
	f_out("adc11_D2_ATEST:",adc_ch[6 ]);
	f_out("adc12_REF     :",adc_ch[7]);
	f_out("adc13_2.5V    :",adc_ch[8]);
	f_out("adc14_3.3V    :",adc_ch[9]);
	f_out("adc15_5.0V    :",adc_ch[10]);
	f_out("temp_sens(V)  :",adc_ch[11]);
	f_out("temp_sens(С)  :",ADC_Temp(adc_ch[11]));
	
	d = adc_ch[7]*100;
	z=(int) (d);
	
//	f_out("d:",d);
//	u_out("z:",z);
	if (z>333) z=333;
	else 
		if (z<0) z=0;
	
	f_out("REF (Дб):",DBm[z]);
	
//	for (i=0;i<12;i++) adcBuffer[i]=0;

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  Adress=0x30; //адресс кассеты
  
  Adress_ATT1=1;
  Adress_ATT2=2;
  Adress_ATT3=3;
  Adress_ATT4=4;
  
  Adress_SWITCH =5;
  Adress_KONTROL_WIRE=6;
  Adress_KONTROL_1=7;
  Adress_KONTROL_2=8;  
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
 timer_INIT_FAPCH1=0;
  FLAG_FAPCH_ON=0;
     
  Delay(1000);
  
  Transf("-------------\r\n");
  Transf("    Б072У\r\n");
  Transf("-------------\r\n");
  
  //IO("~0 init_dds;");
  
//  ATT_upr(1,0xff);
//  ATT_upr(2,0xff);
//  ATT_upr(3,0xff);
//  ATT_upr(4,0xff);


    CS_LMK_1;
   FPGA_CS_1;
  CS_FPGA2_1;
  CS_FPGA1_1;
   CS_adc1_1;
   CS_adc2_1;
   CS_DAC1_1;
   CS_DAC2_1;
  CS_FLASH_1;
  
 LED1_1;
 LED2_1;
 LED3_1;
 
 CN0led(1);
 CN1led(1);
 CN2led(1);
 CN3led(1);
 CN4led(1);
 CN5led(1);
 CN6led(1);
 CN7led(1);
 
 PD0_0;      //сигнал SYNC для LMK
 RESET_ADC_0;//сигнал RESET на АЦП снят
 RESETB_DAC_0;
 
 PWRDN_DAC1=1;
 PWRDN_DAC2=1;
 PWRDN_ADC1=1;
 PWRDN_ADC2=1;
 
 Massiv_dbm();
 
 HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
 HAL_ADC_Start_DMA  (&hadc1,(uint32_t*)&adcBuffer,12); // Start ADC in DMA 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	if (PIN_control     ()==1) Transf ("Event_PA12!\r\n");
  //if (PIN_control_PA8 ()==1) u_out ("SYNC_N:",HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8));

	if (EVENT_INT2==1) {EVENT_INT2=0;  Transf("event 2!\r");UDP_mem_read (1);}
	if (EVENT_INT7==1) {EVENT_INT7=0;  Transf("event 7!\r");UDP_mem_read (0);}
	if (EVENT_INT1==1) {EVENT_INT1=0;/*Transf("event 1!\r");*/}
	
	
	LED();
	UART_conrol();
	UART_DMA_TX();
   //UART_IT_TX ();//отправляем данные , если есть.
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
