/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_uart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

// Version information.
#define MAIN_VERSION  (1)
#define MINOR_VERSION (0)
#define PATCH_VERSION (0)

#define DBL_BUF_LEN (512)
#define DBL_BUF_TOTAL_LEN (2 * DBL_BUF_LEN)

typedef struct _hart_dbl_buf_t {
  uint32_t data[2][DBL_BUF_LEN / 4];
  uint32_t data_rest[DBL_BUF_LEN / 4 * 2];
  int len[2];
  int rest_len;
  int idx;
} uart_dbl_buf_t;

typedef struct _uart_ctx_t {
  const char *name;
  UART_HandleTypeDef *huart;
  IRQn_Type irq_num;
  DMA_HandleTypeDef *hdma_rx;
  DMA_HandleTypeDef *hdma_tx;
  uart_dbl_buf_t buf;
  int buf_idx;
  uint8_t *prefix;
} uart_ctx_t;

typedef struct _ctx_t {
  uart_ctx_t uart1;
  uart_ctx_t uart2;
  uart_ctx_t uart3;
  DMA_HandleTypeDef *memcpy_dma;
} ctx_t;

extern ctx_t ctx;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
