/**
 * @file receive.c
 * @author dokee (dokee.39@gmail.com)
 * @brief 使用 IDLE 中断和 DMA 进行串口接收以及对接收的字符串进行处理
 * @version 0.1
 * @date 2023-07-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "stm32f1xx_hal.h"
#include "string.h"
#include "receive.h"
#include "usart.h"
#include "main.h"
#include "dma.h"

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];

// position used by callback function
uint16_t oldPos = 0;
uint16_t newPos = 0;

// mark the beginning and end of valid data
uint16_t head = 0;
uint16_t tail = 0;

uint8_t is_data_available = 0;

uint8_t is_data_overwritten = 0;

/* time in milliseconds with systick */
int32_t receive_time_ref = 0;

/**
 * @brief Initialize the Ring Buffer
 *
 */
void Receive_BufInit(void)
{
    memset(RxBuf, '\0', RxBuf_SIZE);
    memset(MainBuf, '\0', MainBuf_SIZE);

    head = tail = 0;
    oldPos = 0;
    newPos = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart_receive, RxBuf, RxBuf_SIZE);
    // When we enable DMA transfer using HAL, all the interrupts associated with it are also enabled.
    // As we don’t need the Half Transfer interrupt, we will disable it.
    __HAL_DMA_DISABLE_IT(&hdma_usart_receive_rx, DMA_IT_HT);
}

static void Receive_FlagReset(void)
{
    tail = 0;
    head = 0;
    oldPos = 0;
    newPos = 0;
    is_data_available = 0;
    is_data_overwritten = 0;
}

/**
 * @brief Resets the Ring buffer
 *
 */
void Receive_BufReset(void)
{
    memset(MainBuf, '\0', MainBuf_SIZE);
    memset(RxBuf, '\0', RxBuf_SIZE);
    Receive_FlagReset();
}



/**
 * @brief 从 MainBuf 中找到被起始和终止命令包裹的字符串并拷贝到 pdata 中
 * 
 * @param cmd_start 
 * @param cmd_end 
 * @param pdata 长度必须大于等于 MainBuf_SIZE
 * @return receive_state_t 
 */
RECEIVE_STATUS_t Receive_FindFirstVaildString(char *cmd_start, char *cmd_end, char *pdata)
{
    is_data_overwritten = 0;
    if (is_data_available == 0)
    {
        return RECEIVE_FAILURE;
    }
    else
    {
        uint16_t tail_copy = tail; // 在回调中改变 tail, 在其他函数中就不能用 tail
        uint16_t indx_cmd = 0;
        uint16_t indx_buf = head;
        uint16_t cmd_start_len = strlen(cmd_start);
        uint16_t cmd_end_len = strlen(cmd_end);
        uint16_t indx_buf_temp = 0;
        uint16_t data_start_pos = 0;
        uint16_t data_end_pos = 0;

    repeat1:
        while (MainBuf[indx_buf] != cmd_start[indx_cmd])
        {
            indx_buf++;
            if (indx_buf >= MainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset();
                return RECEIVE_FAILURE;
            }
        }

        indx_buf_temp = indx_buf;

        for (indx_cmd = 0; indx_cmd < cmd_start_len; indx_cmd++)
        {
            if (indx_buf >= MainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset();
                return RECEIVE_FAILURE;
            }
            if (MainBuf[indx_buf] != cmd_start[indx_cmd])
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
        while (MainBuf[indx_buf] != cmd_end[indx_cmd])
        {
            indx_buf++;
            if (indx_buf >= MainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset();
                return RECEIVE_FAILURE;
            }
        }

        indx_buf_temp = indx_buf;

        for (indx_cmd = 0; indx_cmd < cmd_end_len; indx_cmd++)
        {
            // 这两个判断要放在前面, 否则当命令包尾正好时 MainBuf 的 tail 时会出错
            if (indx_buf >= MainBuf_SIZE)
                indx_buf = 0;
            if (indx_buf == tail_copy)
            {
                Receive_FlagReset();
                return RECEIVE_FAILURE;
            }
            if (MainBuf[indx_buf] != cmd_end[indx_cmd])
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
        if (head <= tail)
        {
            memcpy((void *)pdata, (void *)(MainBuf + data_start_pos), data_len);
            head = indx_buf;
        }
        else 
        {
            uint16_t data_in_tail_len = 0;
            data_in_tail_len = MainBuf_SIZE - data_start_pos;
            memcpy((void *)pdata, (void *)(MainBuf + data_start_pos), data_in_tail_len);
            memcpy((void *)(pdata + data_in_tail_len), (void *)MainBuf, data_end_pos);
            head = indx_buf;
        }

        // 结尾加上 '\0'
        pdata[data_len] = '\0';

        // 不必清空 MainBuf, 但如果没数据可以归零
        if (head == tail)
            Receive_FlagReset();

        // 如果发生覆盖则舍弃已经拷贝的
        if (is_data_overwritten)
        {
            is_data_overwritten = 0;
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
    oldPos = newPos; // Update the last position before copying new data

    uint16_t space_remaining; // 记录 MainBuf 剩余空间
    if (head <= tail)
        space_remaining = MainBuf_SIZE - (tail - head);
    else
        space_remaining = head - tail;

    // 拷贝数据 (如有溢出拷贝在前面，即形成环式的数据)
    if (oldPos + Size > MainBuf_SIZE) // If the current position + new data size is greater than the main buffer
    {
        uint16_t datatocopy = MainBuf_SIZE - oldPos;                 // find out how much space is left in the main buffer
        memcpy((void *)(MainBuf + oldPos), (void *)RxBuf, datatocopy); // copy data in that remaining space
        // (uint8_t *) ?
        oldPos = 0;                                                               // point to the start of the buffer
        memcpy((void *)MainBuf, (void *)(RxBuf + datatocopy), (Size - datatocopy)); // copy the remaining data
        newPos = (Size - datatocopy);                                             // update the position
    }
    else
    {
        memcpy((void *)(MainBuf + oldPos), (void *)RxBuf, Size);
        newPos = Size + oldPos;
    }

    tail = newPos;
    if (space_remaining < Size) // 发生覆盖时, 置标志位并改变 head 值
    {
        is_data_overwritten = 1;
        head = tail + 1;
        if (head == MainBuf_SIZE)
            head = 0;
    }

    /* start the DMA again */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_receive, (uint8_t *)RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart_receive_rx, DMA_IT_HT);

    is_data_available = 1;
}
