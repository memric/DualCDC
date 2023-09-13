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
#include "trace.h"
#include <inttypes.h>
#include "ring_buf.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define UART_RX_BUF_LEN     512
#define UART_TX_BUF_LEN     512
#define USB_TX_BUF_LEN      256
#define UART_BYTES2WAIT     3.5
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/**
 * @brief   UART instance type
 *
 */
typedef struct {
        const char *name;
        UART_HandleTypeDef *huart;
        uint8_t rx_byte;
        ring_buf_t *pRbuf;
        uint32_t last_rx_tick;
        uint32_t rx_timeout;
        uint8_t *pTbuf;
        uint32_t tx_len;
} uart_handle_t;

extern void Lock(void *arg);
extern void Unlock(void *arg);

RING_BUF_DEFINE(uart1_rx, UART_RX_BUF_LEN, uint8_t, Lock, Unlock);
RING_BUF_DEFINE(uart2_rx, UART_RX_BUF_LEN, uint8_t, Lock, Unlock);
uint8_t uart1_tx[UART_TX_BUF_LEN];
uint8_t uart2_tx[UART_TX_BUF_LEN];

uart_handle_t hUART[] = {
{"UART1", &huart1, 0, pRING_BUF(uart1_rx), 0, 10, uart1_tx, 0},
{"UART2", &huart2, 0, pRING_BUF(uart2_rx), 0, 10, uart2_tx, 0}
};

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
static inline uart_handle_t *get_uart_handle(UART_HandleTypeDef *huart)
{
    uart_handle_t *uart = NULL;

    if (huart == &huart1)
    {
        uart = &hUART[0];
    }
    else if (huart == &huart2)
    {
        uart = &hUART[1];
    }

#if 0
    for (ind = 0; ind < sizeof(hUART) / sizeof(uart_handle_t); ind++)
    {
        if (hUART[ind].huart == huart)
        {
            uart = &hUART[ind];
            break;
        }
    }
#endif

    return uart;
}
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
        uart_handle_t *uart = &hUART[index / 2];

        USBD_CDC_LineCodingTypeDef *line_coding = (USBD_CDC_LineCodingTypeDef*) pbuf;
        if (line_coding->bitrate == 0 || line_coding->datatype == 0)
        {
            break;
        }

        TRACE_DEBUG("Set Linecoding. Ind. %"PRIu16"\r\n", index);

        /*
         * The maping between USBD_CDC_LineCodingTypeDef and line coding structure.
         *    dwDTERate   -> line_coding->bitrate
         *    bCharFormat -> line_coding->format
         *    bParityType -> line_coding->paritytype
         *    bDataBits   -> line_coding->datatype
         */
        uint32_t baudRate = line_coding->bitrate;
        uint32_t wordLength = (line_coding->datatype == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
        uint32_t stopBits = (line_coding->format == 0) ? UART_STOPBITS_1 : UART_STOPBITS_2;
        uint32_t parity = (line_coding->paritytype == 0) ? UART_PARITY_NONE : (line_coding->paritytype == 1) ? UART_PARITY_ODD : UART_PARITY_EVEN;

        if ((uart->huart->Init.BaudRate != baudRate) ||
            (uart->huart->Init.WordLength != wordLength) ||
            (uart->huart->Init.StopBits != stopBits) ||
            (uart->huart->Init.Parity != parity))
        {
            /* Update settings */
            TRACE_DEBUG("Params. %"PRIu32":%u:%u:%u\r\n",
                        line_coding->bitrate,
                        line_coding->format,
                        line_coding->paritytype,
                        line_coding->datatype);

            HAL_UART_AbortReceive_IT(uart->huart);
            HAL_UART_AbortTransmit_IT(uart->huart);

            /*
             * The maping between USBD_CDC_LineCodingTypeDef and line coding structure.
             *    dwDTERate   -> line_coding->bitrate
             *    bCharFormat -> line_coding->format
             *    bParityType -> line_coding->paritytype
             *    bDataBits   -> line_coding->datatype
             */
            uart->huart->Init.BaudRate = line_coding->bitrate;
            uart->huart->Init.WordLength = (line_coding->datatype == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
            uart->huart->Init.StopBits = (line_coding->format == 0) ? UART_STOPBITS_1 : UART_STOPBITS_2;
            uart->huart->Init.Parity = (line_coding->paritytype == 0) ? UART_PARITY_NONE : (line_coding->paritytype == 1) ? UART_PARITY_ODD : UART_PARITY_EVEN;
            uart->huart->Init.Mode = UART_MODE_TX_RX;
            uart->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
            uart->huart->Init.OverSampling = UART_OVERSAMPLING_16;

            if (HAL_UART_Init(uart->huart) != HAL_OK)
            {
                Error_Handler();
            }

            /* Set ring buffer reset state */
            RingBuf_Init(uart->pRbuf);

            /* Calculate timeout */
            uart->rx_timeout = UART_BYTES2WAIT * 1000
                / (line_coding->bitrate / (line_coding->datatype + 2));

            if (uart->rx_timeout == 0)
            {
                uart->rx_timeout = 1;
            }

            uart->tx_len = 0;

            TRACE_DEBUG("%s timeout: %"PRIu32" ms\r\n", uart->name, uart->rx_timeout);

        }

        /* Start or continue reception */
        HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1);
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
  uart_handle_t *uart = &hUART[index / 2];

  if (uart->tx_len == 0)
  {
      memcpy(uart->pTbuf, Buf, *Len);
      uart->tx_len = *Len;
  }
  else
  {
      return USBD_FAIL;
  }
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
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_handle_t *uart = get_uart_handle(huart);

    if (uart != NULL)
    {
        /* Save reception tick */
        uart->last_rx_tick = HAL_GetTick();

        /* Save byte to buffer */
        RingBuf_Push(uart->pRbuf, &uart->rx_byte, 1);

        /* Continue reception */
        HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_handle_t *uart = get_uart_handle(huart);

    if (uart != NULL)
    {
        uart->tx_len = 0;

        if (huart == &huart1)
        {
            HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET); //TODO Use us timer
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_handle_t *uart = get_uart_handle(huart);

    if (uart != NULL)
    {
        HAL_UART_AbortReceive_IT(huart);

        TRACE_ERR("%s err %"PRIu32"\r\n", uart->name, HAL_UART_GetError(huart));

        /* Continue reception */
        HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1);
    }
}

void UART_Poll(void)
{
    uint16_t ind;
    uart_handle_t *uart = NULL;
    uint8_t usb_tx_buf[USB_TX_BUF_LEN];
    uint8_t err;

    for (ind = 0; ind < sizeof(hUART) / sizeof(uart_handle_t); ind++)
    {
        uart = &hUART[ind];

        /* Check for Tx first */
        if ((uart->tx_len > 0)
            && ((HAL_UART_GetState(uart->huart) & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_READY))
        {
            if (uart->huart == &huart1)
            {
                /* Set DE signal UP for RS485 */
                HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET); //TODO Use us timer
            }

            TRACE_DEBUG("Tx data ready. %s; Size %"PRIu32"\r\n", uart->name, uart->tx_len);

            /* Have pending data for Tx. Start DMA transfer */
            HAL_UART_Transmit_DMA(uart->huart, uart->pTbuf, uart->tx_len);
        }

        /* Check for Rx data */
        if (((HAL_GetTick() - uart->last_rx_tick) > uart->rx_timeout)
            && (RingBuf_GetNum(uart->pRbuf) > 0))
        {
            uint16_t len = RingBuf_Pop(uart->pRbuf, usb_tx_buf, USB_TX_BUF_LEN);

            TRACE_DEBUG("Rx data ready. %s; Size %"PRIu16"\r\n", uart->name, len);

            uint32_t i;
            for (i = 0; i < len; i++)
            {
                TRACE_DEBUG("%02X ", usb_tx_buf[i]);
            }
            TRACE_DEBUG("\r\n");

            do
            {
                err = CDC_Transmit_FS(usb_tx_buf, len, ind * 2);
            }
            while (err != USBD_OK);

            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
