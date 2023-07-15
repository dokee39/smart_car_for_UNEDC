/**
 * @file control.h
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "motor.h"
#include "pid.h"
#include "debug.h"
#include "task_process.h"

/**************** 硬件信息 **************************/
#define ENCODER_RESOLUTION 13.0f                                                  // 编码器物理分辨率 (磁极数)
#define REDUCTION_RATIO 30.0f                                                     // 电机减速比
#define PULSE_PER_REVOLUTION (float)(ENCODER_RESOLUTION * REDUCTION_RATIO * 4.0f) // 电机每转产生的脉冲数
#define WHEEL_DIAMETER 4.8f                                                    // 轮子直径
#define JOURNEY_PER_REVOLUTION (float)(WHEEL_DIAMETER * 3.1416f)               // 轮子每转走的路程

/**************** 软件设置 **************************/
#define CONTROL_LOCAATION_DIV 2 // 每隔多少次进行一次位置控制

#define TARGET_SPEED_MAX 100.0f
#define DELTA_SPEED_MAX 30.0f

#define MOTOR_DIR_ERR_VALID_MAX 1.0f // 从 K210 接收到的合法值范围
#define STEER_COMPENSATION_VALID_TIME (1000u / TASK_CNT_K210_RECEIVE / TIM_TASK_INTERVAL) // 超过这个时间就把转向补偿 ban 掉
#define STEER_COMPENSATION_DELAY 3u

#if IS_DEBUG_UART_ON && IS_DEBUG_ON
// pid 环名称
typedef enum
{
    PID_LOOP_SPEED,
    PID_LOOP_LOCATION,
    PID_LOOP_STEER_COMPENSATION
} PID_LOOP_t;

// 一个分电机的 pid 的结构体,
typedef struct
{
    pid_t motor1;
    pid_t motor2;
} pid_loop_t;

// 包含所有 pid 的结构体, 分为速度环、位置环和转向补偿
typedef struct
{
    pid_loop_t speed;
    pid_t location;
    pid_t steer_compensation;
} pids_t;

extern pids_t pids;
#endif // !IS_DEBUG_UART_ON && IS_DEBUG_ON

void Control_PID_Init(void);
void Control_SetSteerCompensation_basedon_Receive(void);
void Control_Task(void);

#if IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
void Control_PIDs_Get(float *ppids);
#endif // !IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON

#endif
