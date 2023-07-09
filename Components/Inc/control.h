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

/**************** 硬件信息 **************************/
#define ENCODER_RESOLUTION 13                                           // 编码器物理分辨率 (磁极数)
#define REDUCTION_RATIO 30                                              // 电机减速比
#define PULSE_PER_REVOLUTION (float)(ENCODER_RESOLUTION * REDUCTION_RATIO * 4) // 电机每转产生的脉冲数
#define WHEEL_DIAMETER 4.5f                                             // 轮子直径
#define JOURNEY_PER_REVOLUTION (float)(WHEEL_DIAMETER * 3.1416f)               // 轮子每转走的路程

/**************** 软件设置 **************************/
#define PID_PERIOD 20           // tim_PID_Interval 的定时周期 (ms)
#define CONTROL_LOCAATION_DIV 2 // 每隔多少次进行一次位置控制
#define TARGET_SPEED_MAX 200

#if IS_DEBUG_UART_ON && IS_DEBUG_ON
// pid 环名称
typedef enum
{
    PID_LOOP_SPEED,
    PID_LOOP_LOCATION,
} PID_LOOP_t;

// 包含一个环的 pid 的结构体, 分电机
typedef struct
{
    pid_t motor1;
    pid_t motor2;
} pid_loop_t;

// 包含所有 pid 的结构体, 分为速度环和位置环
typedef struct
{
    pid_loop_t speed;
    pid_loop_t location;
} pids_t;

extern pids_t pids;
#endif

void Control_PID_Init(void);

#if IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
void Control_PIDs_Get(float *ppids);
#endif // !IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON

#endif
