/**
 * @file control.c
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "control.h"
#include "main.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "receive.h"

#if IS_DEBUG_UART_ON && IS_DEBUG_ON
static float debug_motor1_voltage = 0.0f; // 真实电压值
static float debug_motor2_voltage = 0.0f;
pids_t pids;
#else
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

static pids_t pids;
#endif

static float motor1_speed = 0.0f;
static float motor2_speed = 0.0f;
static float motor1_speed_set = 0.0f; // 位置环的输出值
static float motor2_speed_set = 0.0f;

static float motor_speed_difference = 0.0f;

static float motor1_location = 0.0f;
static float motor2_location = 0.0f;
static float motor_location_average = 0.0f;
static float motor_location_set = 0.0f; // 位置环设置的目标值

static float motor1_voltage = 0.0f;     // 速度环的输出值
static float motor2_voltage = 0.0f;     // 不是实际电压值, 只是确定 PWM 占空比的比较值

static char *cmd_start = "<!"; // K210 命令包头
static char *cmd_end = ">!";   // K210 命令包尾
static char cmd[MainBuf_SIZE]; // 用于存放从 K210 收到的命令

void Control_PID_Init(void)
{
    // &pid, input_max_err, input_min_err, integral_separate_err, maxout, intergral_limit, kp, ki, kd
    pid_struct_init(&pids.speed.motor1, 0, 0.5f, 0, MOTOR_DUTY_MAX, 2000, 1.7f, 0.8f, 0);
    pid_struct_init(&pids.speed.motor2, 0, 0.5f, 0, MOTOR_DUTY_MAX, 2000, 1.75f, 0.95f, 0);
    pid_struct_init(&pids.location, 0, 0.2f, 0, TARGET_SPEED_MAX, 0, 10, 0, 0);
    // TODO 初始化转向补偿 PID
}

#if IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON

/**
 * @brief 获取当前一份 pid 的值放进数组 ppid (1*9) 中
 *
 * @param pid
 * @param ppid
 */
static void pid_get(pid_t *pid, float *ppid)
{
    ppid[0] = pid->param.p;
    ppid[1] = pid->param.i;
    ppid[2] = pid->param.d;

    ppid[3] = pid->param.input_max_err;
    ppid[4] = pid->param.input_min_err;
    ppid[5] = pid->param.integral_separate_err;

    ppid[6] = pid->param.max_out;
    ppid[7] = pid->param.integral_limit;

    ppid[8] = pid->set;
}

/**
 * @brief 获取当前所有 pid 的值放进数组 ppids (3*9=27) 中
 *
 * @param ppids
 */
void Control_PIDs_Get(float *ppids)
{
    pid_get(&pids.speed.motor1, &ppids[0]);
    pid_get(&pids.speed.motor2, &ppids[9]);
    pid_get(&pids.location, &ppids[18]);
}
#endif // !IS_DEBUG_UART_PID_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON

static float Control_Speed(MOTOR_t MOTOR)
{
    float control_val = 0.0f; // 当前控制值

// 调试速度环时, 保持速度环目标值不变
#if IS_DEBUG_UART_PID_LOOP_SPEED
    if (MOTOR == MOTOR1)
        control_val = pid_calculate(&pids.speed.motor1, motor1_speed, pids.speed.motor1.set); // 进行 PID 计算
    if (MOTOR == MOTOR2)
        control_val = pid_calculate(&pids.speed.motor2, motor2_speed, pids.speed.motor2.set); // 进行 PID 计算
#else
    if (MOTOR == MOTOR1)
        control_val = pid_calculate(&pids.speed.motor1, motor1_speed, motor1_speed_set); // 进行 PID 计算
    if (MOTOR == MOTOR2)
        control_val = pid_calculate(&pids.speed.motor2, motor2_speed, motor2_speed_set); // 进行 PID 计算
#endif

    return control_val;
}

static float Control_Location(void)
{
    float control_val = 0.0f; // 当前控制值

// 调试位置速度环时, 保持位置目标值不变
#if IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED
    control_val = pid_calculate(&pids.location, motor_location_average, pids.location.set);
#else
    control_val = pid_calculate(&pids.location, motor_location_average, motor_location_set);
#endif

    /* 目标速度上限处理 */
    if (control_val > TARGET_SPEED_MAX)
        control_val = TARGET_SPEED_MAX;
    else if (control_val < -TARGET_SPEED_MAX)
        control_val = -TARGET_SPEED_MAX;

    return control_val;
}

void Control_SetSteerCompensation_basedon_Receive(void)
{
//     memset(cmd, '\0', MainBuf_SIZE);
//     while (receive_time_ref > 0)
//     {
//         if (Receive_FindFirstVaildString(&uart_for_debug, cmd_start, cmd_end, cmd) == RECEIVE_SUCCESS)
//         {
//             if (cmd_information_find() == CMD_FIND_SUCCESS)
//             {
//                 is_UART_working = 1;

// #if IS_DEBUG_UART_CMD_FEEDBACK_ON
//                 cmd_feedback(timeout - receive_time_ref);
// #endif // !IS_DEBUG_UART_CMD_FEEDBACK_ON

//                 pid_set(); // TODO
//                 printf("sent after %dms -->  PID_SET_OK!\r\n", timeout - receive_time_ref);

// #if IS_DEBUG_UART_PID_FEEDBACK_ON
//                 pid_feedback(timeout - receive_time_ref);
// #endif // !IS_DEBUG_UART_PID_FEEDBACK_ON

//                 is_UART_working = 0;
//                 break;
//             }
//             else
//             {
//                 printf("sent after %dms -->  COMMAND_ERROR!\r\n", timeout - receive_time_ref);
//             }
//         }
//     }
}

// TODO 计算 delta v 的函数
static void Control_SteerCompensation(void)
{

}

static void Control_LocationSpeed(void) // TODO 改名字，以及加入转向补偿部分
{
    static uint8_t control_location_count = 0;
    if (is_motor1_en == 1 || is_motor2_en == 1) // 电机在使能状态下才进行控制处理
    {
        control_location_count++;
        if (control_location_count >= 2)
        {
            control_location_count = 0;
            motor1_speed_set = Control_Location();
            motor2_speed_set = motor1_speed_set;
        }

// 调试速度环时, 保持速度环目标值不变
#if !IS_DEBUG_UART_PID_LOOP_SPEED
        pid_set_target(&pids.speed.motor1, motor1_speed_set);
        pid_set_target(&pids.speed.motor2, motor2_speed_set);
#endif

        motor1_voltage = Control_Speed(MOTOR1);
        motor2_voltage = Control_Speed(MOTOR2);

#if IS_DEBUG_UART_REAL_TIME_MONITOR_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
        motor1_voltage = (motor1_voltage > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : motor1_voltage;
        debug_motor1_voltage = (motor1_voltage / PWM_COUNTER_PERIOD) * MOTOR_VOLTAGE;
        motor2_voltage = (motor2_voltage > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : motor2_voltage;
        debug_motor2_voltage = (motor2_voltage / PWM_COUNTER_PERIOD) * MOTOR_VOLTAGE;
#endif
    }
}

void Control_Task(void)
{
            // TODO 在合适的位置取得补偿值和设置标志位等
#if IS_DEBUG_UART_TIME_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
        int32_t time = receive_time_ref;
#endif // !IS_DEBUG_UART_TIME_FEEDBACK_ON
        Encoder_PulseGet();
        motor1_speed = ((float)encoder_motor1_pulsenum * 1000.0 * 60.0) / (PULSE_PER_REVOLUTION * TIM_PID_INTERVAL);
        motor2_speed = ((float)encoder_motor2_pulsenum * 1000.0 * 60.0) / (PULSE_PER_REVOLUTION * TIM_PID_INTERVAL);
        motor1_location = ((float)encoder_motor1_pulsenum_sum / PULSE_PER_REVOLUTION) * JOURNEY_PER_REVOLUTION;
        motor2_location = ((float)encoder_motor2_pulsenum_sum / PULSE_PER_REVOLUTION) * JOURNEY_PER_REVOLUTION;
        motor_location_average = (motor1_location + motor2_location) / 2.0f;

// 调试速度环时用
#if IS_DEBUG_UART_PID_LOOP_SPEED && IS_DEBUG_UART_ON && IS_DEBUG_ON
        if (is_motor1_en == 1)
        {
            motor1_voltage = Control_Speed(MOTOR1);
#if IS_DEBUG_UART_REAL_TIME_MONITOR_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
            motor1_voltage = (motor1_voltage > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : motor1_voltage;
            debug_motor1_voltage = (motor1_voltage / PWM_COUNTER_PERIOD) * MOTOR_VOLTAGE;
#endif
        }
        if (is_motor2_en == 1)
        {
            motor2_voltage = Control_Speed(MOTOR2);
#if IS_DEBUG_UART_REAL_TIME_MONITOR_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
            motor2_voltage = (motor2_voltage > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : motor2_voltage;
            debug_motor2_voltage = (motor2_voltage / PWM_COUNTER_PERIOD) * MOTOR_VOLTAGE;
#endif
        }
#endif // !IS_DEBUG_UART_PID_LOOP_SPEED && IS_DEBUG_UART_ON && IS_DEBUG_ON

// 调试位置速度环时用
#if IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED && IS_DEBUG_UART_ON && IS_DEBUG_ON
        Control_LocationSpeed();
#endif // !IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED && IS_DEBUG_UART_ON && IS_DEBUG_ON

        Motor_Output((int16_t)motor1_voltage, (int16_t)motor2_voltage);

#if IS_DEBUG_UART_REAL_TIME_MONITOR_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
        if (is_UART_working == 0)
        {
            printf("motor: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",
                   motor1_speed,
                   motor2_speed,
                   pids.speed.motor1.set,
                   pids.speed.motor2.set,
                   motor1_location,
                   motor2_location,
                   motor_location_average,
                   pids.location.set,
                   debug_motor1_voltage,
                   debug_motor2_voltage);
#if IS_DEBUG_UART_TIME_FEEDBACK_ON
            time -= receive_time_ref;
            printf("sent in %dms\r\n", time);
#endif // !IS_DEBUG_UART_TIME_FEEDBACK_ON
        }
#endif
}
