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

// 要与 RING_STATUS 的相同
typedef enum
{
    TRANSMIT_SUCCESS = 0u,
    TRANSMIT_OVERFLOW = 1u,
    TRANSMIT_FAILURE = 2u, 
} TRANSMIT_STATUS_t;

typedef struct
{
    UART_HandleTypeDef *huart;

    void *StrBuf_H; // 这个缓存区是用来存中断嵌套时高优先级的打印数据, 正常的打印数据要新建数组
    int16_t StrBuf_H_Printed_Size; // 表示 StrBuf_H 接收到的打印数据的字节数
    uint16_t StrBuf_SIZE;

    void *TxBuf;
    uint16_t TxBuf_SIZE;

    ring_t *pring_TxMainBuf;

    uint8_t is_data_available; // MainBuf 中是否还有需要传的数据
    uint8_t is_sending; // 表示正在 Transmit_Send() 函数中，以防止重复进入
    uint8_t is_printing; // 表示正在 Transmit_Printf() 函数中，以防止重入 printf
    uint8_t is_data_H_available; // StrBuf_H 是否有待打印数据
} uart_transmit_t;

// TODO 可变 Size
// TODO hdma 直接引用

/* 在此加入 uart_transmit 的外部声明 BEGIN */
extern uart_transmit_t uart_transmit_for_debug;
/* 在此加入 uart_transmit 的外部声明 END */

void Transmit_Init(void);
TRANSMIT_STATUS_t Transmit_printf(uart_transmit_t *puart_transmit, char *format, ...);

#endif //!_TRANSMIT_H_
