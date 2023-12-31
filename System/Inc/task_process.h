/**
 * @file task_process.h
 * @author dokee (dokee.39@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _TASK_PROCESS_H_
#define _TASK_PROCESS_H_

// 任务计数周期 (TIM_TASK_INTERVAL 的倍数)
#define TASK_CNT_K210_RECEIVE (20u / TIM_TASK_INTERVAL)
#define TASK_CNT_DEBUG_RECEIVE (50u / TIM_TASK_INTERVAL)
#define TASK_CNT_DEBUG_LED (640u / TIM_TASK_INTERVAL)

void TaskProcess_Init(void);

#endif
