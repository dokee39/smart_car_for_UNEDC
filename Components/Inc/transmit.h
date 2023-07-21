/**
 * @file transmit.h
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _TRANSMIT_H_
#define _TRANSMIT_H_

#include "ring.h"

#define StrBuf_SIZE 256
#define TxBuf_SIZE 256
#if IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
#define TxMainBuf_SIZE 4096
#else
#define TxMainBuf_SIZE 512
#endif

typedef enum
{
    TRANSMIT_SUCCESS,
    TRANSMIT_FAILURE,
} TRANSMIT_STATUS_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;

    uint8_t StrBuf[StrBuf_SIZE];
    uint8_t TxBuf[TxBuf_SIZE];
    uint8_t MainBuf[TxMainBuf_SIZE];
    ring_t *pring_MainBuf;

    uint8_t is_data_available; // MainBuf 中是否还有需要传的数据
    uint8_t is_data_overflow;
} uart_transmit_t;

extern uart_transmit_t uart_transmit_for_debug;

void Transmit_Init(uart_transmit_t *puart_transmit, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);
TRANSMIT_STATUS_t Transmit_printf(uart_transmit_t *puart_transmit, char *format, ...);

#endif //!_TRANSMIT_H_
