/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  *  >>> THINGS YOU MUST VERIFY FOR YOUR BOARD <<<
  *    1. Buffers MUST live in DMA-accessible RAM (AXI SRAM / SRAM1..3),
  *       NOT DTCM. See the ".axisram" section note below.
  *    2. TIM2_UP / DMA_REQUEST_TIM2_UP / DMA1 streams exist on your part.
  *    3. If D-Cache is ON, the SCB_InvalidateDCache_by_Addr() calls are needed
  *       (already included). aligned(32) + size multiple of 32 is required.
  *    4. EXTI1/EXTI3 IRQHandlers must not also be defined in stm32h7xx_it.c
  *       (remove them there if duplicated).
  *  ==========================================================================
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t data_port_e;
	uint16_t data_port_f;
	uint16_t data_port_g;
} sample_t;

typedef struct {
	int16_t ns_adc;
	int16_t ew_adc;
	uint32_t ns_t;
	uint32_t ew_t;
	uint32_t counter;
} data_record_t;

typedef struct {
	int16_t max_ns_adc;
	int16_t max_ew_adc;
	uint32_t max_ns_t;
	uint32_t max_ew_t;
	float max_ns_inp;
	float max_ew_inp;
} max_sample_t;

typedef struct {
	int16_t min_ns_adc;
	int16_t min_ew_adc;
	uint32_t min_ns_t;
	uint32_t min_ew_t;
} min_sample_t;

typedef struct {
	uint8_t ns_rec_type;
	int16_t ns_first_adc;
	int16_t ns_max_adc;
	int16_t ns_min_adc;
	int16_t ew_first_adc;
	int16_t ew_max_adc;
	int16_t ew_min_adc;
	uint32_t ns_max_t;
	uint32_t ns_min_t;
	uint32_t ew_max_t;
	uint32_t ew_min_t;
	uint32_t trigger_t;
	int16_t ns_favg;
	int16_t ew_favg;
} record_t;

#define POL_RX_SIZE     4
#define POL_TIMEOUT_MS  1000
#define SPI_TX_FRAME_SIZE  128

uint8_t spiTxFrame[SPI_TX_FRAME_SIZE];

typedef enum { POL_NEGATIVE = 0,
			   POL_POSITIVE = 1,
			   POL_UNKNOWN  = 2      /* UNKNOWN */
} polarity_t;

uint8_t polRxBuf[POL_RX_SIZE];
uint8_t polarity = POL_UNKNOWN;  /* = 2 */

volatile uint32_t trig_pos     = 0;   // DMA write index latched at trigger moment
volatile uint8_t  trigger_flag = 0;
uint32_t captured_count = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINT_DATA_ENABLE
#define DEBUG_SYSTEM_ENABLE

//#define WATCHDOG_DISABLE

#define SETTING_USE_DATASHEET_PARAM
//#define SETTING_INVERT_EW_CHANNEL

#define DELAY_SEC						1
#define MAX_BUF_SIZE					500

#define TB_MODE_0						0
#define TB_MODE_1						~(TB_MODE_0)
#define TB_MODE							TB_MODE_0

#define SAMPLE_READ_MAX					1000
#define ADC_R1         				 	10000.0
#define ADC_R2          				4320.0
#define BUFFER_SIZE 					2000

/* --- DMA sampling configuration ---------------------------------------------
 * TIM2 kernel clock on this clock tree = 240 MHz
 *   (HCLK = 480/2 = 240 MHz, APB1 /2 -> PCLK1 120 MHz, timer x2 -> 240 MHz)
 * ARR = (TIM2_CLK / SAMPLE_RATE_HZ) - 1
 *   1 MHz  -> ARR 239   (1.0 us / sample)
 *   2 MHz  -> ARR 119   (0.5 us / sample)
 *   ~3.3MHz-> ARR  71   (matches old read_adc ~300 ns)
 * Pick a rate the AHB bus + 4 parallel DMA can sustain; start safe, then push.
 */
#define TIM2_CLK_HZ						240000000UL
#define SAMPLE_RATE_HZ					1000000UL
#define TIM2_ARR						((TIM2_CLK_HZ / SAMPLE_RATE_HZ) - 1U)

#ifdef SETTING_USE_DATASHEET_PARAM
	#define ADC_VREF_P						2.048
	#define ADC_VREF_N						0.988
	#define ADC_OFFSET						1.52
#else
	#define ADC_VREF_P						2.003
	#define ADC_VREF_N						0.994
	#define ADC_OFFSET						1.55
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* DMA target buffers — MUST be in AXI SRAM (DMA cannot reach DTCM).
 * If your linker has no ".axisram" output section, either:
 *   (a) add one mapping to RAM_D1 (0x24000000) in your .ld, OR
 *   (b) remove the section attribute IF your project's default RAM is AXI SRAM
 *       (many CubeIDE H7 projects default to DTCM — do NOT leave it there).
 * aligned(32) + (BUFFER_SIZE*2) multiple of 32 -> safe for D-Cache maintenance.
 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IWDG_HandleTypeDef hiwdg1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef *esp_uart;
UART_HandleTypeDef *debug_uart;


sample_t sample[SAMPLE_READ_MAX];
sample_t ordered[SAMPLE_READ_MAX];   // <<< เพิ่มบรรทัดนี้
record_t rec;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void find_max_min();
void send_data();
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void collect_samples(void);
void clear_buffer(void);
void reorder_samples(uint32_t n);
void send_data_spi1(uint8_t pol);
void receive_polarity(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_sec(int sec){
	int count_500ms = 0;
	do{
		if (HAL_IWDG_Refresh(&hiwdg1) != HAL_OK){
			Error_Handler();
		}
		HAL_Delay(500);
		count_500ms++;
	}while(count_500ms < (sec<<1));
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // -- UART Mapping -- //
  esp_uart = &huart2;
  debug_uart = &huart4;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Temp for debug
#ifdef WATCHDOG_DISABLE
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif

  printf("Last reset cause : ");

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST))
  {
      printf("IWDG1 timer");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
  {
      printf("Reset Pin");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
  {
      printf("Power On Reset");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
  {
      printf("Software Reset");
  }
  else
  {
      printf("Other reason");
  }

  printf("\n");

  /* Clear reset flags */
  __HAL_RCC_CLEAR_RESET_FLAGS();

//  printf("Setup :\n");

 // printf("Main program started (DMA capture @ %lu Hz, ARR=%lu)\n",
 //        (unsigned long)SAMPLE_RATE_HZ, (unsigned long)TIM2_ARR);
  HAL_GPIO_WritePin(USER_LED_BLUE_GPIO_Port, USER_LED_BLUE_Pin, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* USER CODE END WHILE */
		    HAL_IWDG_Refresh(&hiwdg1);

		    collect_samples();          // เก็บจนกว่าจะ trig หรือเต็ม

		    if (trigger_flag)           // ถ้ามี trig จริง -> วิเคราะห์
		    {
		    	reorder_samples(captured_count);
		    	receive_polarity();
		        find_max_min();
		        send_data_spi1(polarity);
		        // send_data();
		        clear_buffer();
	    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg1.Init.Window = 4000;
  hiwdg1.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_BLUE_GPIO_Port, USER_LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CT2_Pin CT3_Pin CT4_Pin CT5_Pin
                           CT6_Pin CT7_Pin CT8_Pin CT9_Pin
                           CT10_Pin CT11_Pin CT12_Pin CT13_Pin
                           CT14_Pin CT15_Pin CT0_Pin CT1_Pin */
  GPIO_InitStruct.Pin = CT2_Pin|CT3_Pin|CT4_Pin|CT5_Pin
                          |CT6_Pin|CT7_Pin|CT8_Pin|CT9_Pin
                          |CT10_Pin|CT11_Pin|CT12_Pin|CT13_Pin
                          |CT14_Pin|CT15_Pin|CT0_Pin|CT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_USER_SW_Pin */
  GPIO_InitStruct.Pin = GPIO_USER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_USER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D0A_Pin D1A_Pin D2A_Pin D3A_Pin
                           D4A_Pin D5A_Pin D6A_Pin D7A_Pin
                           D8A_Pin D9A_Pin CT22_Pin CT23_Pin */
  GPIO_InitStruct.Pin = D0A_Pin|D1A_Pin|D2A_Pin|D3A_Pin
                          |D4A_Pin|D5A_Pin|D6A_Pin|D7A_Pin
                          |D8A_Pin|D9A_Pin|CT22_Pin|CT23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : P0B_Pin P1B_Pin D2B_Pin D3B_Pin
                           D4B_Pin D5B_Pin D6B_Pin D7B_Pin
                           D8B_Pin D9B_Pin CT16_Pin CT17_Pin
                           CT18_Pin CT19_Pin CT20_Pin CT21_Pin */
  GPIO_InitStruct.Pin = P0B_Pin|P1B_Pin|D2B_Pin|D3B_Pin
                          |D4B_Pin|D5B_Pin|D6B_Pin|D7B_Pin
                          |D8B_Pin|D9B_Pin|CT16_Pin|CT17_Pin
                          |CT18_Pin|CT19_Pin|CT20_Pin|CT21_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_BLUE_Pin */
  GPIO_InitStruct.Pin = USER_LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int16_t convert_number(uint16_t data){
	int16_t value;
#if TB_MODE == TB_MODE_0
		if((data & 0x0200) > 0){
			// Positive
			value = data & 0x01ff;
		}else{
			// Negative
			value = -(((~data + 1) & 0x01ff));
		}
#else
		if((data & 0x0200) > 0){
			// Negative
			value = -(~data + 0x0001);
		} else{
			// Positive
			value = data & 0x01ff;
		}
#endif
	return value;
}


void send_data(){
	char send_buf[MAX_BUF_SIZE];
	uint8_t chksum = 0xa5;			// Temp, the checksum should be a calculated one.

	sprintf(send_buf, "$STMFIELD,%d,%d,%lu,%d,%lu,%d,%lu,%d,%lu,%d,%lu*%02X\r\n", rec.ns_first_adc, rec.ew_first_adc,
			rec.trigger_t, rec.ns_max_adc, rec.ns_max_t, rec.ns_min_adc, rec.ns_min_t, rec.ew_max_adc, rec.ew_max_t, rec.ew_min_adc,rec.ew_min_t, chksum);
	HAL_UART_Transmit(esp_uart, (uint8_t*)send_buf, strlen(send_buf), HAL_MAX_DELAY);
#ifdef DEBUG_SYSTEM_ENABLE
	printf("\nSend update to ESP32 = %s\n",send_buf);
#endif
}

float cal_voltage_adc(int16_t adc_val){
	return ((ADC_VREF_P - ADC_VREF_N) * adc_val / 512) + ADC_OFFSET;
}

float cal_voltage_inp(float vol_adc){
	return ((vol_adc - 1.5) / (ADC_R2 / (ADC_R1 + ADC_R2)));
}

void receive_polarity(void)
{
    HAL_StatusTypeDef st = HAL_SPI_Receive(&hspi2, polRxBuf, POL_RX_SIZE, POL_TIMEOUT_MS);

    if (st == HAL_OK)
    {
        polarity = polRxBuf[0];
    }
    else
    {
        HAL_SPI_Abort(&hspi2);
        __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
        polarity = POL_UNKNOWN;      /* ← เปลี่ยนเป็น 2 */
    }
}
static uint8_t nmea_checksum(const char *s)
{
    uint8_t c = 0;
    while (*s) c ^= (uint8_t)*s++;
    return c;
}
void send_data_spi1(uint8_t pol)
{
    char body[112];
    char line[SPI_TX_FRAME_SIZE];

    /* body = ทุกอย่างระหว่าง $ กับ * */
    snprintf(body, sizeof(body),
        "STMFIELD,%d,%d,%d,%lu,%d,%lu,%d,%lu,%d,%lu,%d",
        rec.ns_favg,
        rec.ew_favg,
        rec.ns_max_adc, (unsigned long)rec.ns_max_t,
        rec.ns_min_adc, (unsigned long)rec.ns_min_t,
        rec.ew_max_adc, (unsigned long)rec.ew_max_t,
        rec.ew_min_adc, (unsigned long)rec.ew_min_t,
        pol);

    uint8_t ck  = nmea_checksum(body);
    int     len = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, ck);

    if (len < 0) return;
    if (len > SPI_TX_FRAME_SIZE) len = SPI_TX_FRAME_SIZE;

    /* pad ศูนย์ให้เต็มเฟรมคงที่ */
    memset(spiTxFrame, 0, SPI_TX_FRAME_SIZE);
    memcpy(spiTxFrame, line, len);

    HAL_SPI_Transmit(&hspi1, spiTxFrame, SPI_TX_FRAME_SIZE, 100);
}

void reorder_samples(uint32_t n)
		{
    if (n >= SAMPLE_READ_MAX) {
        memcpy(ordered, sample, sizeof(sample));
        return;
    }
    uint32_t idx = 0;
    for (uint32_t i = n; i < SAMPLE_READ_MAX; i++)   // ส่วนเก่า
        ordered[idx++] = sample[i];
    for (uint32_t i = 0; i < n; i++)                 // ส่วนใหม่
        ordered[idx++] = sample[i];
		}
void find_max_min(){

	int count;
	sample_t *p_sample;
	uint16_t adc_data_a, adc_data_b;
	int16_t adc_value_a, adc_value_b;
	uint32_t counter = 0;

	rec.ns_favg = 0;
	rec.ns_max_adc = 0;
	rec.ns_min_adc = 0;
	rec.ew_favg = 0;
	rec.ew_max_adc = 0;
	rec.ew_min_adc = 0;
	rec.ns_max_t = 10000000;
	rec.ns_min_t = 10000000;
	rec.ew_max_t = 10000000;
	rec.ew_min_t = 10000000;
	rec.trigger_t = 0;

	count = 0;
	p_sample = ordered;;

	do{
		// Read ADC
		adc_data_a = p_sample->data_port_f & 0x03ff; // D0-D9 (PF0-PF9)
		adc_data_b = p_sample->data_port_g & 0x03ff; // D0-D9 (PG0-PG9)

		// Read Counter
		counter = 0;
		counter |= (uint32_t)(p_sample->data_port_e & 0xFFFF);                    // C0-C15  (PE0-PE15)
		counter |= (uint32_t)((p_sample->data_port_g >> 10) & 0x003F) << 16;      // C16-C21 (PG10-PG15)
		counter |= (uint32_t)((p_sample->data_port_f >> 10) & 0x0003) << 22;      // C22-C23 (PF10-PF11)

		// Convert the adc_data to adc_value
		adc_value_a = convert_number(adc_data_a);

#ifdef SETTING_INVERT_EW_CHANNEL
		adc_value_b = -(convert_number(adc_data_b));			// *Invert to negative value due to the hardware bug
#else
		adc_value_b = convert_number(adc_data_b);
#endif

		// Find Max/Min of N-S
		if(adc_value_a > rec.ns_max_adc && adc_value_a <= 450){
			rec.ns_max_adc = adc_value_a;
			rec.ns_max_t = counter;
		}else if (adc_value_a < rec.ns_min_adc && adc_value_a >= -450){
			rec.ns_min_adc = adc_value_a;
			rec.ns_min_t = counter;
		}
		// Find Max/Min of E-W
		if(adc_value_b > rec.ew_max_adc && adc_value_b <= 450){
			rec.ew_max_adc = adc_value_b;
			rec.ew_max_t = counter;
		}else if (adc_value_b < rec.ew_min_adc && adc_value_b >= -450){
			rec.ew_min_adc = adc_value_b;
			rec.ew_min_t = counter;
		}

		p_sample++;
#ifdef DEBUG_PRINT_DATA_ENABLE

		printf("%d, %lu, %d, %d\n", count, counter, adc_value_a, adc_value_b);
#endif
	} while(++count < SAMPLE_READ_MAX);
	printf("--- End of Counter Data ---\n\n");
}

void collect_samples(void)
{
    uint32_t n = 0;
    sample_t *p = sample;

    trigger_flag = 0;   // เคลียร์ก่อนเริ่มเก็บรอบใหม่

    // เก็บไปเรื่อย ๆ จนกว่าจะ trig หรือครบ 1000
    while (n < SAMPLE_READ_MAX)
    {
        p->data_port_e = GPIOE->IDR;
        p->data_port_f = GPIOF->IDR;
        p->data_port_g = GPIOG->IDR;
        p++;
        n++;

        if (trigger_flag)   // trig เข้า -> หยุดทันที (เช่นหยุดที่ 500)
            break;
    }

    captured_count = n;   // จำเลขไว้ให้ find_max_min ใช้
}

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1)
    {
        trigger_flag = 1;
    }
    else if (GPIO_Pin == GPIO_USER_SW_Pin)
    {
        __NOP();   // user switch — add handling if needed
    }
}
// Overwite the _write function that defined in syscall.c
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(debug_uart, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
void clear_buffer(void)
{
    memset(sample, 0, sizeof(sample));
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
