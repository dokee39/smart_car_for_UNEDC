/**
 * @file receive.c
 * @author dokee (dokee.39@gmail.com)
 * @brief 使用 IDLE 中断和 DMA 进行串口接收以及对接收的字符串进行处理
 * @version 0.2
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 * @note 增加了对多个串口的支持
 */

#include "stm32f1xx_hal.h"
#include "string.h"
#include "receive.h"
#include "usart.h"
#include "main.h"
#include "dma.h"

/* 在此加入 uart, 并将其放入地址列表中 BEGIN */
// 对应的 extern 也要修改
// 记得把对应的 dma extern 出去
uart_receive_t uart_receive_for_debug;
uart_receive_t uart_receive_with_K210;

uart_receive_t *(uart_receive_list[]) = {&uart_receive_for_debug, &uart_receive_with_K210};
uint8_t num_of_uart_receives = 2;
/* 在此加入 uart, 并将其放入地址列表中 END */

/**
 * @brief
 *
 * @param puart_receive
 * @param phuart
 * @param phdma
 */
void Receive_Init(uart_receive_t *puart_receive, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    puart_receive->huart = huart;
    puart_receive->hdma = hdma;

    memset(puart_receive->RxBuf, '\0', RxBuf_SIZE);
    memset(puart_receive->MainBuf, '\0', RxMainBuf_SIZE);

    puart_receive->head = 0;
    puart_receive->tail = 0;
    puart_receive->oldPos = 0;
    puart_receive->newPos = 0;
    puart_receive->is_data_available = 0;
    puart_receive->is_data_overwritten = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(puart_receive->huart, puart_receive->RxBuf, RxBuf_SIZE);
    // When we enable DMA transfer using HAL, all the interrupts associated with it are also enabled.
    // As we don’t need the Half Transfer interrupt, we will disable it.
    __HAL_DMA_DISABLE_IT(puart_receive->hdma, DMA_IT_HT);
}

static void Receive_FlagReset(uart_receive_t *puart_receive)
{
    puart_receive->head = 0;
    puart_receive->tail = 0;
    puart_receive->oldPos = 0;
    puart_receive->newPos = 0;
    puart_receive->is_data_available = 0;
    puart_receive->is_data_overwritten = 0;
}

/**
 * @brief
 *
 * @param puart_receive
 */
void Receive_Reset(uart_receive_t *puart_receive)
{
    memset(puart_receive->MainBuf, '\0', RxMainBuf_SIZE);
    memset(puart_receive->RxBuf, '\0', RxBuf_SIZE);
    Receive_FlagReset(puart_receive);
}

/**
 * @brief 从 MainBuf 中找到被起始和终止命令包裹的字符串并拷贝到 pdata 中
 * 
 * @param puart_receive 
 * @param cmd_start 
 * @param cmd_end 
 * @param pdata 长度必须大于等于 RxMainBuf_SIZE
 * @return RECEIVE_STATUS_t 
 */
RECEIVE_STATUS_t Receive_FindFirstVaildString(uart_receive_t *puart_receive, char *cmd_start, char *cmd_end, char *pdata)
{
    puart_receive->is_data_overwritten = 0;
    if (puart_receive->is_data_available == 0)
    {
        return RECEIVE_FAILURE;
    }
    else
    {
        uint16_t tail_copy = puart_receive->tail; // 在回调中改变 tail, 在其他函数中就不能用 tail
        uint16_t indx_cmd = 0;
        uint16_t indx_buf = puart_receive->head;
        uint16_t cmd_start_len = strlen(cmd_start);
        uint16_t cmd_end_len = strlen(cmd_end);
        uint16_t indx_buf_temp = 0;
        uint16_t data_start_pos = 0;
        uint16_t data_end_pos = 0;

    repeat1:
        while (puart_receive->MainBuf[indx_buf] != cmd_start[indx_cmd])
        {
            indx_buf++;
            if (indx_buf >= RxMainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset(puart_receive);
                return RECEIVE_FAILURE;
            }
        }

        indx_buf_temp = indx_buf;

        for (indx_cmd = 0; indx_cmd < cmd_start_len; indx_cmd++)
        {
            if (indx_buf >= RxMainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset(puart_receive);
                return RECEIVE_FAILURE;
            }
            if (puart_receive->MainBuf[indx_buf] != cmd_start[indx_cmd])
            {
                indx_buf = indx_buf_temp + 1;
                indx_cmd = 0;
                goto repeat1;
            }
            indx_buf++;
        }

        // 到此已经找到了 cmd_start, 记录
        data_start_pos = indx_buf;

        indx_cmd = 0;

    repeat2:
        while (puart_receive->MainBuf[indx_buf] != cmd_end[indx_cmd])
        {
            indx_buf++;
            if (indx_buf >= RxMainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset(puart_receive);
                return RECEIVE_FAILURE;
            }
        }

        indx_buf_temp = indx_buf;

        for (indx_cmd = 0; indx_cmd < cmd_end_len; indx_cmd++)
        {
            // 这两个判断要放在前面, 否则当命令包尾正好时 MainBuf 的 tail 时会出错
            if (indx_buf >= RxMainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset(puart_receive);
                return RECEIVE_FAILURE;
            }
            if (puart_receive->MainBuf[indx_buf] != cmd_end[indx_cmd])
            {
                indx_buf = indx_buf_temp + 1;
                indx_cmd = 0;
                goto repeat2;
            }
            indx_buf++;
        }

        // 到此已经找到了 cmd_end, 记录
        data_end_pos = indx_buf - cmd_end_len;
        uint16_t data_len = data_end_pos - data_start_pos;

        // copy
        if (puart_receive->head <= puart_receive->tail)
        {
            memcpy((void *)pdata, (void *)(puart_receive->MainBuf + data_start_pos), data_len);
            puart_receive->head = indx_buf;
        }
        else
        {
            uint16_t data_in_tail_len = 0;
            data_in_tail_len = RxMainBuf_SIZE - data_start_pos;
            memcpy((void *)pdata, (void *)(puart_receive->MainBuf + data_start_pos), data_in_tail_len);
            memcpy((void *)(pdata + data_in_tail_len), (void *)puart_receive->MainBuf, data_end_pos);
            puart_receive->head = indx_buf;
        }

        // 结尾加上 '\0'
        pdata[data_len] = '\0';

        // 不必清空 MainBuf, 但如果没数据可以归零
        if (puart_receive->head == puart_receive->tail)
            Receive_FlagReset(puart_receive);

        // 如果发生覆盖则舍弃已经拷贝的
        if (puart_receive->is_data_overwritten)
        {
            puart_receive->is_data_overwritten = 0;
            return RECEIVE_FAILURE;
        }
        else
        {
            return RECEIVE_SUCCESS;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 找到 huart 对应的 uart_receive
    uart_receive_t *puart_receive = NULL;
    for (uint8_t i = 0; i < num_of_uart_receives; i++)
    {
        if (uart_receive_list[i]->huart == huart)
        {
            puart_receive = uart_receive_list[i];
            break;
        }
    }

    if (puart_receive != NULL)
    {
        puart_receive->oldPos = puart_receive->newPos; // Update the last position before copying new data

        uint16_t space_remaining; // 记录 MainBuf 剩余空间
        if (puart_receive->head <= puart_receive->tail)
            space_remaining = RxMainBuf_SIZE - (puart_receive->tail - puart_receive->head);
        else
            space_remaining = puart_receive->head - puart_receive->tail;

        // 拷贝数据 (如有溢出拷贝在前面，即形成环式的数据)
        if (puart_receive->oldPos + Size > RxMainBuf_SIZE) // If the current position + new data size is greater than the main buffer
        {
            uint16_t datatocopy = RxMainBuf_SIZE - puart_receive->oldPos;                   // find out how much space is left in the main buffer
            memcpy((void *)(puart_receive->MainBuf + puart_receive->oldPos), (void *)puart_receive->RxBuf, datatocopy); // copy data in that remaining space
            puart_receive->oldPos = 0;                                                                 // point to the start of the buffer
            memcpy((void *)puart_receive->MainBuf, (void *)(puart_receive->RxBuf + datatocopy), (Size - datatocopy)); // copy the remaining data
            puart_receive->newPos = (Size - datatocopy);                                               // update the position
        }
        else
        {
            memcpy((void *)(puart_receive->MainBuf + puart_receive->oldPos), (void *)puart_receive->RxBuf, Size);
            puart_receive->newPos = Size + puart_receive->oldPos;
        }

        puart_receive->tail = puart_receive->newPos;
        if (space_remaining < Size) // 发生覆盖时, 置标志位并改变 head 值
        {
            puart_receive->is_data_overwritten = 1;
            puart_receive->head = puart_receive->tail + 1;
            if (puart_receive->head == RxMainBuf_SIZE)
                puart_receive->head = 0;
        }

        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(puart_receive->huart, (uint8_t *)puart_receive->RxBuf, RxBuf_SIZE);
        __HAL_DMA_DISABLE_IT(puart_receive->hdma, DMA_IT_HT);

        puart_receive->is_data_available = 1;
    }
}
