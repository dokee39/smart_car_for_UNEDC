/**
 * @file debug.h
 * @author dokee (dokee.39@gmail.com)
 * @brief 用于调试
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "main.h"

/**
 * 调试开关总览：
 * |---IS_DEBUG_ON -> 调试模式总开关
 *     |---IS_DEBUG_LED_RUN_ON -> LED_run 调试 (每秒翻转一次，表示没有卡住)
 *     |---IS_DEBUG_UART_ON    -> 串口调试
 *     |   |---IS_DEBUG_UART_REAL_TIME_MONITOR_ON    -> 串口实时监控
 *     |   |---IS_DEBUG_UART_TIME_FEEDBACK_ON        -> 显示 PID 计算及串口发送耗时
 *     |   |---IS_DEBUG_UART_CMD_FEEDBACK_ON         -> 串口调试时返回收到的命令
 *     |   |---IS_DEBUG_UART_PID_FEEDBACK_ON         -> 串口调试时返回当前全部的 PID 设置
 *     |   |---IS_DEBUG_UART_PID_LOOP_SPEED          -> 调试速度环时打开此开关
 *     |   |---IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED -> 调试位置速度环时打开此开关
 *     |---IS_DEBUG_IN_MAIN_C_ON  -> 除上述以外需要存放在 main.c 的调试代码
 */

// 串口调试命令格式:
// <!motor1 speed P_I_D set: %f, %f, %f OK?>!        -> kp, ki, kd
// <!motor1 speed err_limit set: %f, %f, %f OK?>!    -> input_max_err, input_min_err, integral_separate_err
// <!motor1 speed out_limit set: %f, %f OK?>!        -> max_out, integral_limit
// <!motor1 speed target set: %f OK?>!               -> set
// .......2.speed................................
// ..motor..location.............................
// ..motor..steer_compensation...................

/* 调试模式总开关 */
#define IS_DEBUG_ON 1
#if IS_DEBUG_ON

/* LED_run 调试 (每秒翻转一次，表示没有卡住) */
#define IS_DEBUG_LED_RUN_ON 1
#if IS_DEBUG_LED_RUN_ON

void Debug_LED_run_Toggle(void);

#endif // !IS_DEBUG_LED_RUN_ON

/* 串口调试 */
#define IS_DEBUG_UART_ON 1
#define IS_DEBUG_UART_REAL_TIME_MONITOR_ON 1 // 串口实时监控
#define IS_DEBUG_UART_TIME_FEEDBACK_ON 0     // 显示 PID 计算及串口发送耗时
#define IS_DEBUG_UART_CMD_FEEDBACK_ON 0      // 串口调试时返回收到的命令
#define IS_DEBUG_UART_PID_FEEDBACK_ON 0      // 串口调试时返回当前全部的 PID 设置
// 下面分别调试 pid 各环的不能同时打开
#define IS_DEBUG_UART_PID_LOOP_SPEED 0          // 调试速度环时打开此开关
#define IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED 1 // 调试位置速度环时打开此开关
#if IS_DEBUG_UART_ON

// 接收到信息的最大字符数
#define DEBUG_UART_MESSAGE_SIZE_MAX 256

extern uint8_t is_UART_working;

void Debug_SetPID_basedon_Receive(void);

#endif // !IS_DEBUG_UART_ON

#define IS_DEBUG_IN_MAIN_C_ON 0

#endif // !IS_DEBUG_ON

#endif // !__DEBUG_H__

