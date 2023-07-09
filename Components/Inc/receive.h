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

/* Define the Size Here */
#define RxBuf_SIZE 64
#define MainBuf_SIZE 128

typedef enum 
{
    RECEIVE_SUCCESS,
    RECEIVE_FAILURE
} RECEIVE_STATUS_t;

/* Time in milliseconds with systick*/
extern int32_t receive_time_ref;

void Receive_BufInit(void);
void Receive_BufReset(void);
RECEIVE_STATUS_t Receive_FindFirstVaildString(char *cmd_start, char *cmd_end, char *pdata);




#endif // !__RECEIVE_H__
