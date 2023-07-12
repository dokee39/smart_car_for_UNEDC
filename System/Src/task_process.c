/**
 * @file task_process.c
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "main.h"
#include "control.h"
#include "timesilce_task.h"
#include "debug.h"
#include "tim.h"

// 任务计数周期 (TIM_TASK_INTERVAL 的倍数)
#define TASK_CNT_DEBUG_RECEIVE (50u / TIM_TASK_INTERVAL)
#define TASK_CNT_K210_RECEIVE (20u / TIM_TASK_INTERVAL)
#define TASK_CNT_DEBUG_LED (640u / TIM_TASK_INTERVAL)

TimesilceTaskObj task_K210_receive;

#if IS_DEBUG_ON
#if IS_DEBUG_UART_ON
TimesilceTaskObj task_debug_receive;
#endif // !IS_DEBUG_UART_ON
#if IS_DEBUG_LED_RUN_ON
TimesilceTaskObj task_LED_run;
#endif // !IS_DEBUG_LED_RUN_ON
#endif // !IS_DEBUG_ON

void TaskProcess_Init(void)
{

    // timeslice_task_init(&task_K210_receive, task2_hdl, 1, TASK_CNT_K210_RECEIVE); // TODO 
    // timeslice_task_add(&task_K210_receive);
#if IS_DEBUG_ON
#if IS_DEBUG_UART_ON
    timeslice_task_init(&task_debug_receive, Debug_SetPID_basedon_Receive, 2, TASK_CNT_DEBUG_RECEIVE);
    timeslice_task_add(&task_debug_receive);
#endif // !IS_DEBUG_UART_ON
#if IS_DEBUG_LED_RUN_ON
    timeslice_task_init(&task_LED_run, Debug_LED_run_Toggle, 3, TASK_CNT_DEBUG_LED);
    timeslice_task_add(&task_LED_run);
#endif // !IS_DEBUG_LED_RUN_ON
#endif // !IS_DEBUG_ON
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim_PID_Interval)) // TIM7 用于产生 PID 执行时间间隔, 每 20ms 进入一次
    {
        Control_Task();
    }

    if (htim == (&htim_Task_Interval)) // TIM6 的任务处理中断, 每 10ms 进入一次
    {
        timeslice_tick();
        timeslice_exec();
    }
}
