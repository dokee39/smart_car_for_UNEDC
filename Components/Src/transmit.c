/**
 * @file transmit.c
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "usart.h"
#include "transmit.h"
#include "ring.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 在此加入 uart, 并将其放入地址列表中 BEGIN */
// 对应的 extern 也要修改
// 记得把对应的 dma extern 出去
uart_transmit_t uart_transmit_for_debug;

ring_t ring_MainBuf;

uart_transmit_t *(uart_transmit_list[]) = {&uart_transmit_for_debug};
uint8_t num_of_uart_transmits = 1;
/* 在此加入 uart, 并将其放入地址列表中 END */

void Transmit_Init(uart_transmit_t *puart_transmit, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    puart_transmit->huart = huart;
    puart_transmit->hdma = hdma;
    puart_transmit->pring_MainBuf = &ring_MainBuf;

    memset(puart_transmit->StrBuf, '\0', StrBuf_SIZE);
    memset(puart_transmit->TxBuf, '\0', TxBuf_SIZE);
    ring_init(puart_transmit->pring_MainBuf, puart_transmit->MainBuf, TxMainBuf_SIZE);

    puart_transmit->is_data_available = 0;
    puart_transmit->is_data_overflow = 0;
}

static TRANSMIT_STATUS_t Transmit_Send(uart_transmit_t *puart_transmit)
{
    uint16_t data_size = ring_usedspace_get(puart_transmit->pring_MainBuf);
    if (data_size == 0)
    {
        puart_transmit->huart->gState = HAL_UART_STATE_READY;
        puart_transmit->is_data_available = 0;
    }
    else if (data_size <= TxBuf_SIZE)
    {
        ring_fetch(puart_transmit->pring_MainBuf, puart_transmit->TxBuf, data_size);
        HAL_UART_Transmit_DMA(puart_transmit->huart, puart_transmit->TxBuf, data_size);
        __HAL_DMA_DISABLE_IT(puart_transmit->hdma, DMA_IT_HT);
        puart_transmit->is_data_available = 0;
    }
    else
    {
        ring_fetch(puart_transmit->pring_MainBuf, puart_transmit->TxBuf, TxBuf_SIZE);
        HAL_UART_Transmit_DMA(puart_transmit->huart, puart_transmit->TxBuf, TxBuf_SIZE);
        __HAL_DMA_DISABLE_IT(puart_transmit->hdma, DMA_IT_HT);
    }
    // TODO 考虑不成功的情况
    return TRANSMIT_SUCCESS;
}

TRANSMIT_STATUS_t Transmit_printf(uart_transmit_t *puart_transmit, char *format, ...)
{
    // 格式化字符串到 StrBuf
    va_list args;
    uint16_t Size;

    va_start(args, format);
    Size = vsprintf((char *)puart_transmit->StrBuf, format, args);
    va_end(args);

    puart_transmit->is_data_available = 1;

    // TODO 溢出了要处理些什么呢
    if (ring_add(puart_transmit->pring_MainBuf, puart_transmit->StrBuf, Size) == RING_OVERFLOW)
        puart_transmit->is_data_overflow = 1;

    // 判断到 DMA 没有发送任务, 则进行发送
    if (HAL_DMA_GetState(puart_transmit->hdma) == HAL_DMA_STATE_READY)
        Transmit_Send(puart_transmit);

    // TODO return 不成功的情况
    return TRANSMIT_SUCCESS;
}
// TODO 考虑不同优先级导致的修改

// TODO 甚至不要中断都能跑，他真的我哭死
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 找到 huart 对应的 uart_transmit
    uart_transmit_t *puart_transmit = NULL;
    for (uint8_t i = 0; i < num_of_uart_transmits; i++)
    {
        if (uart_transmit_list[i]->huart == huart)
        {
            puart_transmit = uart_transmit_list[i];
            break;
        }
    }

    if (puart_transmit != NULL)
    {
        // TODO 注意在这个函数中是否要改变串口或 DMA 的标志位
        puart_transmit->huart->gState = HAL_UART_STATE_READY;
        // TODO 判断是否还有数据要传
        Transmit_Send(puart_transmit);
    }
}
