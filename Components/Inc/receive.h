/**
 * @file receive.h
 * @author dokee (dokee.39@gmail.com)
 * @brief 使用 IDLE 中断和 DMA 进行串口接收以及对接收的字符串进行处理
 * @version 0.1
 * @date 2023-07-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __RECEIVE_H__
#define __RECEIVE_H__

#include "usart.h"

/* Define the Size Here */
#define RxBuf_SIZE 64
#define RxMainBuf_SIZE 128

typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t RxBuf[RxBuf_SIZE];
    uint8_t MainBuf[RxMainBuf_SIZE];

    // position used by callback function
    uint16_t oldPos;
    uint16_t newPos;

    // mark the beginning and end of valid data
    uint16_t head;
    uint16_t tail;

    uint8_t is_data_available;
    uint8_t is_data_overwritten;
} uart_receive_t;

typedef enum 
{
    RECEIVE_SUCCESS,
    RECEIVE_FAILURE
} RECEIVE_STATUS_t;

/* 在此加入 uart_receive 的外部声明 BEGIN */
// C 文件中对应的也要修改
extern uart_receive_t uart_receive_for_debug;
extern uart_receive_t uart_receive_with_K210;
/* 在此加入 uart_receive 的外部声明 END */

void Receive_Init(uart_receive_t *puart_receive, UART_HandleTypeDef *huart);
void Receive_Reset(uart_receive_t *puart_receive);
RECEIVE_STATUS_t Receive_FindFirstVaildString(uart_receive_t *puart_receive, char *cmd_start, char *cmd_end, char *pdata);




#endif // !__RECEIVE_H__
