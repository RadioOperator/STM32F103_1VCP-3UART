/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

const uint8_t vcp_print_info[] =
{
"\r\n\
====================================\r\n\
Uart-1:  Rx-PA10   Tx-PA9\r\n\
Uart-2:  Rx-PA3    -\r\n\
Uart-3:  Rx-PB11   -\r\n\r\n\
short PB7-PB8, print in Hex mode, or\r\n\
short PB8-PB9, print in ASCII mode\r\n\r\n\
Uart speed = USB-VCP speed.\r\n\
====================================\r\n"
};

#define USE_UART_PREFIX         1 //0 - disable, 1 - VCP received "Uart-1: " as UART1 Rx message prefix.

#define UARTx_PREFIX_LEN        10
#define UART1_PREFIX            "\r\nUart-1: "
#define UART2_PREFIX            "\r\nUart-2: "
#define UART3_PREFIX            "\r\nUart-3: "

const uint8_t uart1_prefix[16] = {UART1_PREFIX};
const uint8_t uart2_prefix[16] = {UART2_PREFIX};
const uint8_t uart3_prefix[16] = {UART3_PREFIX};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
/* USER CODE BEGIN PV */
volatile uint32_t u32LEDcounter;
uint32_t u32LEDblinkEnable;
uint32_t pause_for_line_open = 1;
uint32_t print_hex_flag = 0;      //0-ASCII, 1-HEX print out
const char HEX[] = "0123456789ABCDEF";
static uint8_t u8CDC_Tx_Buf[1024] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
ctx_t ctx;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CDC_Tx_HexString(uint8_t *data, uint8_t length)
{
  int len = length;
  
  while(len--)
  {
    *(u8CDC_Tx_Buf+3*len+2) = ' '; //space
    *(u8CDC_Tx_Buf+3*len+1) = HEX[(*(data+len))&0x0f];
    *(u8CDC_Tx_Buf+3*len)   = HEX[(*(data+len))>>4];    
  }
  while (CDC_Transmit_FS(u8CDC_Tx_Buf, 3*length) == USBD_BUSY) //print out
  {
  }
}

void CDC_Tx_AscString(uint8_t *data, uint8_t length)
{
  while (CDC_Transmit_FS(data, length) == USBD_BUSY) //print out
  {
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

//  SCB->VTOR = 0x08004000; //uncomment this line for YAB use, set IROM1 Start address coordinately.
  
  int i;
  uart_ctx_t * uart_ctx;
  memset(&ctx, 0, sizeof(ctx_t));
  uint8_t * prefix;
    
  ctx.uart1.name = "UART1";
  ctx.uart1.huart = &huart1;
  ctx.uart1.hdma_rx = &hdma_usart1_rx;
  ctx.uart1.hdma_tx = &hdma_usart1_tx;
  ctx.uart1.prefix = (uint8_t*)&uart1_prefix;
  ctx.uart1.irq_num = USART1_IRQn;
  
  ctx.uart2.name = "UART2";
  ctx.uart2.huart = &huart2;
  ctx.uart2.hdma_rx = &hdma_usart2_rx;
  ctx.uart2.hdma_tx = &hdma_usart2_tx;
  ctx.uart2.prefix = (uint8_t*)&uart2_prefix;
  ctx.uart2.irq_num = USART2_IRQn;
  
  ctx.uart3.name = "UART3";
  ctx.uart3.huart = &huart3;
  ctx.uart3.hdma_rx = &hdma_usart3_rx;
  ctx.uart3.hdma_tx = &hdma_usart3_tx;
  ctx.uart3.prefix = (uint8_t*)&uart3_prefix;
  ctx.uart3.irq_num = USART3_IRQn;
  
  ctx.memcpy_dma = &hdma_memtomem_dma1_channel1;
  
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  while(pause_for_line_open)
  {
    HAL_Delay(100);
  }
  HAL_Delay(1000);
  //send info
  CDC_Transmit_FS((uint8_t *)vcp_print_info, sizeof(vcp_print_info)-1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // __wfe();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    for (i = 1; i <= 3; i++)
    {
      if (i == 1)      uart_ctx = &ctx.uart1;
      else if (i == 2) uart_ctx = &ctx.uart2;
      else             uart_ctx = &ctx.uart3;
      
      if (uart_ctx->buf_idx != uart_ctx->buf.idx)
      {
        int buf_idx = uart_ctx->buf_idx;
        
#if USE_UART_PREFIX
        CDC_Transmit_FS(uart_ctx->prefix, UARTx_PREFIX_LEN); //print: "Uart-x: "
#endif //#if USE_UART_PREFIX
        
        if (!print_hex_flag)
             CDC_Tx_AscString((uint8_t *)uart_ctx->buf.data[buf_idx], uart_ctx->buf.len[buf_idx]);
        else CDC_Tx_HexString((uint8_t *)uart_ctx->buf.data[buf_idx], uart_ctx->buf.len[buf_idx]);
        
        uart_ctx->buf_idx = buf_idx ? 0 : 1;
        // SEGGER_RTT_printf(0, "mloop: buf=%d\n", buf_idx);
      }

      if (uart_ctx->buf.rest_len > 0)
      {
        int tx_len = uart_ctx->buf.rest_len;
        uart_ctx->buf.rest_len = 0;
        uart_ctx->buf_idx = 0;
        
#if USE_UART_PREFIX
        CDC_Transmit_FS(uart_ctx->prefix, UARTx_PREFIX_LEN); //print: "UARTx: "
#endif //#if USE_UART_PREFIX
        
        if (!print_hex_flag)
             CDC_Tx_AscString((uint8_t *)uart_ctx->buf.data_rest, tx_len);
        else CDC_Tx_HexString((uint8_t *)uart_ctx->buf.data_rest, tx_len);
        // SEGGER_RTT_printf(0, "rest: len=%d\n", tx_len);
      }
    }
    //LED
    if (u32LEDblinkEnable)
    {
      if (u32LEDcounter <= 50) //~50ms on
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //LED on
      }
      else
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //LED off
        u32LEDblinkEnable = 0;
      }
    }
    else
    {
      if (u32LEDcounter >= 2000) //~2s blink
      {
        u32LEDblinkEnable = 1;
        u32LEDcounter = 1;
        
        if (0 == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) print_hex_flag = 1; //PB8=Low,  HEX mode
        else                                          print_hex_flag = 0; //PB8=High, ASCII mode
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static uart_ctx_t * getUartPort(UART_HandleTypeDef *huart)
{
  uart_ctx_t * uart_ctx;
  
  if      (huart == &huart1) uart_ctx = &ctx.uart1;
  else if (huart == &huart2) uart_ctx = &ctx.uart2;
  else                       uart_ctx = &ctx.uart3;
  
  return uart_ctx;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  uart_ctx_t * uart_ctx = getUartPort(huart);
    
  if (uart_ctx->buf.idx != 0) {
    uart_ctx->buf.idx = 0;
  }

  if (uart_ctx->buf_idx == 1) {
    // SEGGER_RTT_printf(0, "rxhalf: %s; [X]\n", uart_ctx->name);
    return;
  }
  // In Rx Half callback, the length of received data is half length of double buffer.
  uart_ctx->buf.len[0] = DBL_BUF_LEN; 
  // Set index of double buffer to next.
  uart_ctx->buf.idx = 1;
  // SEGGER_RTT_printf(0, "rxhalf: %s; \n", uart_ctx->name);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_ctx_t * uart_ctx = getUartPort(huart);

  if (uart_ctx->buf.idx != 1) {
    uart_ctx->buf.idx = 1;
  }
  
  if (uart_ctx->buf_idx == 0) {
    // SEGGER_RTT_printf(0, "rxcmpl: %s; [X]\n", uart_ctx->name);
    return;
  }
  // In Rx callback, the length of received data is half length of double buffer.
  uart_ctx->buf.len[1] = DBL_BUF_LEN; 
  // Set index of double buffer to next.
  uart_ctx->buf.idx = 0;
  // SEGGER_RTT_printf(0, "rxcmpl: %s; \n", uart_ctx->name);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_ctx_t * uart_ctx = getUartPort(huart);
  
//  SEGGER_RTT_printf(0, "uart error: %s; \n", uart_ctx->name);
  HAL_UART_DMAStop(huart);
  HAL_UART_Receive_DMA(huart, (uint8_t *)uart_ctx->buf.data[0], DBL_BUF_TOTAL_LEN);
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
//    SEGGER_RTT_printf(0, "_Error_Handler: %s #%d\n", file, line);
    HAL_Delay(1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
