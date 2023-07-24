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
#include "debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 在此加入 uart_transmit, 并将其放入地址列表中 BEGIN */
// uart_transmit 要 extern 出去
uart_transmit_t uart_transmit_for_debug;

uart_transmit_t *(uart_transmit_list[]) = {&uart_transmit_for_debug};
uint8_t num_of_uart_transmits = 1;
/* 在此加入 uart_transmit, 并将其放入地址列表中 END */

/* 在此声明串口发送需要的缓存区和环状数组 BEGIN */
// 缓存区 StrBuf, TxBuf, TxMainBuf 共三个
// 环状数组 ring_MainBuf 共一个
#define StrBuf_for_debug_SIZE 256
uint8_t StrBuf_for_debug[StrBuf_for_debug_SIZE];
#define TxBuf_for_debug_SIZE 256
uint8_t TxBuf_for_debug[TxBuf_for_debug_SIZE];
#if IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
#define TxMainBuf_for_debug_SIZE 4096
#else
#define TxMainBuf_for_debug_SIZE 512
#endif
uint8_t TxMainBuf_for_debug[TxMainBuf_for_debug_SIZE];
ring_t ring_TxMainBuf_for_debug;

/* 在此声明串口发送需要的缓存区和环状数组 END */

/**
 * @brief Transmit 串口初始化内部调用的函数
 *
 * @param puart_transmit
 * @param huart
 * @param StrBuf_H
 * @param StrBuf_SIZE
 * @param TxBuf
 * @param TxBuf_SIZE
 * @param TxMainBuf
 * @param TxMainBuf_SIZE
 * @param pring_TxMainBuf
 */
static void Transmit_Init_Internal(uart_transmit_t *puart_transmit,
                                   UART_HandleTypeDef *huart,
                                   void *StrBuf_H,
                                   uint16_t StrBuf_SIZE,
                                   void *TxBuf,
                                   uint16_t TxBuf_SIZE,
                                   void *TxMainBuf,
                                   uint16_t TxMainBuf_SIZE,
                                   ring_t *pring_TxMainBuf)
{
    puart_transmit->huart = huart;

    puart_transmit->StrBuf_H = StrBuf_H;
    puart_transmit->StrBuf_H_Printed_Size = 0;
    puart_transmit->StrBuf_SIZE = StrBuf_SIZE;
    memset(puart_transmit->StrBuf_H, '\0', puart_transmit->StrBuf_SIZE);

    puart_transmit->TxBuf = TxBuf;
    puart_transmit->TxBuf_SIZE = TxBuf_SIZE;
    memset(puart_transmit->TxBuf, '\0', puart_transmit->TxBuf_SIZE);

    puart_transmit->pring_TxMainBuf = pring_TxMainBuf;
    ring_init(puart_transmit->pring_TxMainBuf, TxMainBuf, TxMainBuf_SIZE);

    puart_transmit->is_data_available = 0;
    puart_transmit->is_sending = 0;
    puart_transmit->is_printing = 0;
    puart_transmit->is_data_H_available = 0;
}

/**
 * @brief Transmit 串口初始化
 *
 * @note 若添加 uart_transmit, 这里也要对应添加
 *
 */
void Transmit_Init(void)
{
    /* 在这里为每个 uart_trnsmit 初始化 BEGIN */
    Transmit_Init_Internal(&uart_transmit_for_debug,
                           &huart_for_debug,
                           StrBuf_for_debug,
                           StrBuf_for_debug_SIZE,
                           TxBuf_for_debug,
                           TxBuf_for_debug_SIZE,
                           TxMainBuf_for_debug,
                           TxMainBuf_for_debug_SIZE,
                           &ring_TxMainBuf_for_debug);
    /* 在这里为每个 uart_trnsmit 初始化 END */
}

/**
 * @brief 判断是否有内容要发送, 如果有则拷贝到发送缓存区并打开 DMA 串口发送
 *
 * @note 注意此函数不能重复进入, 进入该函数时会置标志位 puart_transmit->is_sending
 *
 * @param puart_transmit
 * @return TRANSMIT_STATUS_t
 * @return 可能为 TRANSMIT_SUCCESS, TRANSMIT_FAILURE
 * @return 实际使用时并没有处理该函数的返回值, 因为正常情况下都是成功的
 */
static TRANSMIT_STATUS_t Transmit_Send(uart_transmit_t *puart_transmit)
{
    TRANSMIT_STATUS_t TRANSMIT_STATUS = TRANSMIT_SUCCESS;

    if (puart_transmit->is_sending == 0)
    {
        puart_transmit->is_sending = 1;
        uint16_t data_size = ring_usedspace_get(puart_transmit->pring_TxMainBuf);
        if (data_size == 0)
        {
            puart_transmit->huart->gState = HAL_UART_STATE_READY;
            puart_transmit->is_data_available = 0;
        }
        else if (data_size <= puart_transmit->TxBuf_SIZE)
        {
            TRANSMIT_STATUS = (TRANSMIT_STATUS_t)ring_fetch(puart_transmit->pring_TxMainBuf, puart_transmit->TxBuf, data_size);
            if (TRANSMIT_STATUS == TRANSMIT_SUCCESS)
            {
                HAL_UART_Transmit_DMA(puart_transmit->huart, puart_transmit->TxBuf, data_size);
                __HAL_DMA_DISABLE_IT(puart_transmit->huart->hdmatx, DMA_IT_HT);
                puart_transmit->is_data_available = 0;
            }
        }
        else
        {
            TRANSMIT_STATUS = (TRANSMIT_STATUS_t)ring_fetch(puart_transmit->pring_TxMainBuf, puart_transmit->TxBuf, puart_transmit->TxBuf_SIZE);
            if (TRANSMIT_STATUS == TRANSMIT_SUCCESS)
            {
                HAL_UART_Transmit_DMA(puart_transmit->huart, puart_transmit->TxBuf, puart_transmit->TxBuf_SIZE);
                __HAL_DMA_DISABLE_IT(puart_transmit->huart->hdmatx, DMA_IT_HT);
            }
        }
        puart_transmit->is_sending = 0;
    }
    return TRANSMIT_STATUS;
}

/**
 * @brief 带缓存区的串口 DMA 格式化 printf, 第一个参数是调用 uart_transmit, 后面和 printf 一样使用
 *
 * @note 该函数将要发送的字符串送至对应的串口 DMA 发送缓存区
 * @note 可以连续调用, 但如果缓存区装不下了则会省略掉超出的内容
 * @note 可以根据返回值进行缓存区溢出判断, 以防止忽略重要的命令
 *
 * @param puart_transmit
 * @param format
 * @param ...
 * @return TRANSMIT_STATUS_t
 * @return 可能为 TRANSMIT_SUCCESS, TRANSMIT_OVERFLOW, TRANSMIT_FAILURE
 */
TRANSMIT_STATUS_t Transmit_printf(uart_transmit_t *puart_transmit, char *format, ...)
{
    TRANSMIT_STATUS_t TRANSMIT_STATUS = TRANSMIT_SUCCESS;

    if (puart_transmit->is_data_H_available == 1)
    {
        TRANSMIT_STATUS = (TRANSMIT_STATUS_t)ring_append(puart_transmit->pring_TxMainBuf, puart_transmit->StrBuf_H, (uint16_t)puart_transmit->StrBuf_H_Printed_Size);
        puart_transmit->is_data_H_available = 0;
    }

    if (TRANSMIT_STATUS == TRANSMIT_SUCCESS)
    {
        if (puart_transmit->is_printing == 0)
        {
            puart_transmit->is_printing = 1;
            // 格式化字符串到 StrBuf
            va_list args;
            int16_t Size;
            uint8_t StrBuf[puart_transmit->StrBuf_SIZE];

            va_start(args, format);
            Size = vsprintf((char *)StrBuf, format, args);
            va_end(args);

            if (Size <= 0)
            {
                TRANSMIT_STATUS = TRANSMIT_FAILURE;
            }
            else
            {
                puart_transmit->is_data_available = 1;

                // 这里包含了溢出判断
                TRANSMIT_STATUS = (TRANSMIT_STATUS_t)ring_append(puart_transmit->pring_TxMainBuf, StrBuf, (uint16_t)Size);
                if (TRANSMIT_STATUS != TRANSMIT_FAILURE)
                {
                    // 判断到 DMA 没有发送任务, 则进行发送
                    if (HAL_DMA_GetState(puart_transmit->huart->hdmatx) == HAL_DMA_STATE_READY)
                        Transmit_Send(puart_transmit);
                }
            }

            if (TRANSMIT_STATUS == TRANSMIT_SUCCESS && puart_transmit->is_data_H_available == 1)
            {
                TRANSMIT_STATUS = (TRANSMIT_STATUS_t)ring_append(puart_transmit->pring_TxMainBuf, puart_transmit->StrBuf_H, (uint16_t)puart_transmit->StrBuf_H_Printed_Size);
                puart_transmit->is_data_H_available = 0;
            }

            puart_transmit->is_printing = 0; // TODO 这玩意要移到最后吗还是？
        }
        else
        {
            // TODO 如果在这里被打断怎么办呢
            // TODO 这里标志位怎么办呢

            va_list args_H;

            va_start(args_H, format);
            puart_transmit->StrBuf_H_Printed_Size = vsprintf((char *)puart_transmit->StrBuf_H, format, args_H);
            va_end(args_H);

            if (puart_transmit->StrBuf_H_Printed_Size <= 0)
            {
                TRANSMIT_STATUS = TRANSMIT_FAILURE;
            }
            else
            {
                puart_transmit->is_data_H_available = 1;
            }
        }
    }

    return TRANSMIT_STATUS;
}
// TODO 考虑不同优先级导致的修改

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
        // 要改变 UART 的标志位为 READY (这可能是 HAL 库的 BUG)
        puart_transmit->huart->gState = HAL_UART_STATE_READY;

        Transmit_Send(puart_transmit);
    }
}
