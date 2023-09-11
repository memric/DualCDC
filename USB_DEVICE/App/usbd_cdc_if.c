/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define UART_RX_BUF_LEN     1024
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

uint8_t uart_rx_buf[2][UART_RX_BUF_LEN];
uint32_t uart_rx_ind[2] = {0, 0};
uint32_t uart_rx_tick[2] = {0, 0};
uint32_t uart_rx_timeout[2] = {10, 10};
uint32_t de_pending = 0;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len, uint16_t index);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length, uint16_t index)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
    {
        UART_HandleTypeDef *huart;

        if (index == 2)
        {
            huart = &huart2;
        }
        else
        {
            huart = &huart1;
        }

        USBD_CDC_LineCodingTypeDef *line_coding = (USBD_CDC_LineCodingTypeDef *)pbuf;
        if (line_coding->bitrate == 0 || line_coding->datatype == 0) {
            break;
        }

        HAL_UART_AbortReceive_IT(huart);

//        __HAL_UART_DISABLE(huart);

        /*
         * The maping between USBD_CDC_LineCodingTypeDef and line coding structure.
         *    dwDTERate   -> line_coding->bitrate
         *    bCharFormat -> line_coding->format
         *    bParityType -> line_coding->paritytype
         *    bDataBits   -> line_coding->datatype
         */
        huart->Init.BaudRate = line_coding->bitrate;
        huart->Init.WordLength = (line_coding->datatype == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
        huart->Init.StopBits = (line_coding->format == 0) ? UART_STOPBITS_1 : UART_STOPBITS_2;
        huart->Init.Parity = (line_coding->paritytype == 0) ? UART_PARITY_NONE : (line_coding->paritytype == 1) ? UART_PARITY_ODD : UART_PARITY_EVEN;
        huart->Init.Mode = UART_MODE_TX_RX;
        huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart->Init.OverSampling = UART_OVERSAMPLING_16;

        if (HAL_UART_Init(huart) != HAL_OK)
        {
            Error_Handler();
        }

        uart_rx_timeout[index / 1] = 1000 / (line_coding->bitrate / (line_coding->datatype + 2));
        if (uart_rx_timeout[index / 1] == 0)
        {
            uart_rx_timeout[index / 1] = 1;
        }

        uart_rx_ind[index / 1] = 0;
        HAL_UART_Receive_IT(huart, &uart_rx_buf[index / 1][uart_rx_ind[index / 1]], 1);

//        __HAL_UART_ENABLE(huart);
//        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

//        NVIC_ClearPendingIRQ(uart_ctx->irq_num);

//        HAL_UART_DMAStop(huart);
//        HAL_UART_Receive_DMA(uart_ctx->huart, (uint8_t *)uart_ctx->buf.data[0], DBL_BUF_TOTAL_LEN);
    }
    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len, uint16_t index)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS, index);

#ifdef LOOPBACK
  CDC_Transmit_FS(Buf, *Len, index);
#else
  if (index < 2)
  {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  }

  HAL_UART_Transmit_DMA((index < 2) ? &huart1 : &huart2, Buf, *Len);
#endif

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len, uint16_t index)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS, index);

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

//  uint16_t rest_len = Len;
//  uint32_t i;
//  for (i = 0; result == USBD_OK && i <= Len; rest_len = Len - i) {
//
//      if (rest_len >= USB_FS_MAX_PACKET_SIZE) {
//          USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &Buf[i], USB_FS_MAX_PACKET_SIZE);
//          i += USB_FS_MAX_PACKET_SIZE;
//          do {
//              result = USBD_CDC_TransmitPacket(&hUsbDeviceFS, index);
//          } while (result == USBD_BUSY);
//
//      } else if (rest_len == 0) {
//          // It's necessary to send zero-length packet to compliance USB protocol.
//          USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &Buf[0], 0);
//          do {
//              result = USBD_CDC_TransmitPacket(&hUsbDeviceFS, index);
//          } while (result == USBD_BUSY);
//          break;
//
//      } else {
//          USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &Buf[i], rest_len);
//          do {
//              result = USBD_CDC_TransmitPacket(&hUsbDeviceFS, index);
//          } while (result == USBD_BUSY);
//          break;
//
//      }
//  }
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t ind = 0;

    if (huart == &huart1)
    {
        ind = 0;
    }
    else if (huart == &huart2)
    {
        ind = 1;
    }
    else
    {
        return;
    }

    uart_rx_tick[ind] = HAL_GetTick();
    uart_rx_ind[ind]++;

    if (uart_rx_ind[ind] >= UART_RX_BUF_LEN)
    {
        CDC_Transmit_FS(uart_rx_buf[ind], uart_rx_ind[ind], ind * 2);
        uart_rx_ind[ind] = 0;
    }

    HAL_UART_Receive_IT(huart, &uart_rx_buf[ind][uart_rx_ind[ind]], 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        de_pending = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint16_t ind = 0;

    HAL_UART_AbortReceive_IT(huart);

    if (huart == &huart1)
    {
        ind = 0;
    }
    else if (huart == &huart2)
    {
        ind = 1;
    }
    else
    {
        return;
    }

    uart_rx_ind[ind] = 0;
    HAL_UART_Receive_IT(huart, &uart_rx_buf[ind][uart_rx_ind[ind]], 1);
}

void UART_Poll(void)
{
    uint16_t ind;

    if (de_pending)
    {
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        de_pending = 0;
    }

    for (ind = 0; ind < 2; ind++)
    {
        if (((HAL_GetTick() - uart_rx_tick[ind]) > uart_rx_timeout[ind]) && (uart_rx_ind[ind] > 0))
        {
            HAL_UART_AbortReceive_IT(ind == 0 ? &huart1 : &huart2);

            CDC_Transmit_FS(uart_rx_buf[ind], uart_rx_ind[ind], ind * 2);
            uart_rx_ind[ind] = 0;

            HAL_UART_Receive_IT(ind == 0 ? &huart1 : &huart2, &uart_rx_buf[ind][uart_rx_ind[ind]], 1);
        }
    }
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
