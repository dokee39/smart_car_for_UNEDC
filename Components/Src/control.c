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
#include <math.h>
#include "debug.h"
#include "stm32f1xx_it.h"
#include "receive.h"

#define PID_SPEED_MOTOR1_P 6.7f
#define PID_SPEED_MOTOR1_I 2.2f
#define PID_SPEED_MOTOR1_D 0.0f

#define PID_SPEED_MOTOR2_P 7.3f
#define PID_SPEED_MOTOR2_I 2.3f
#define PID_SPEED_MOTOR2_D 0.0f

#define PID_LOCATION_P 15.0f
#define PID_LOCATION_I 0.0f
#define PID_LOCATION_D 0.0f

#define PID_STEER_COMPENSATION_P 20.0f
#define PID_STEER_COMPENSATION_I 1.4f
#define PID_STEER_COMPENSATION_D 30.0f

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
static float motor_speed_set = 0.0f; // 位置环的输出值
static float motor1_speed_set = 0.0f;
static float motor2_speed_set = 0.0f;

static float motor_dir_err[STEER_COMPENSATION_DELAY] = {0.0f}; // 从 K210 获取的偏差值, 因为需要延时响应所以多记录几个
static float motor_steer_compensation_ratio = 0.0f;
static float motor_speed_difference_set = 0.0f;

static float motor1_location = 0.0f;
static float motor2_location = 0.0f;
static float motor_location_average = 0.0f;
static float motor_location_set = 0.0f; // 位置环设置的目标值

static float motor1_voltage = 0.0f; // 速度环的输出值
static float motor2_voltage = 0.0f; // 不是实际电压值, 只是确定 PWM 占空比的比较值

static char *cmd_start = "<!"; // K210 命令包头
static char *cmd_end = ">!";   // K210 命令包尾
static char cmd[MainBuf_SIZE]; // 用于存放从 K210 收到的命令

void Control_PID_Init(void)
{
    // &pid, 
    // input_max_err, input_min_err, integral_separate_err, 
    // maxout, intergral_limit, 
    // kp, ki, kd
    pid_struct_init(&pids.speed.motor1,
                    0.0f, 0.5f, 0.0f,
                    MOTOR_DUTY_MAX, 2000,
                    PID_SPEED_MOTOR1_P, PID_SPEED_MOTOR1_I, PID_SPEED_MOTOR1_D);
    pid_struct_init(&pids.speed.motor2,
                    0.0f, 0.5f, 0.0f,
                    MOTOR_DUTY_MAX, 2000,
                    PID_SPEED_MOTOR2_P, PID_SPEED_MOTOR2_I, PID_SPEED_MOTOR2_D);
    pid_struct_init(&pids.location,
                    0.0f, 0.2f, 0.5f,
                    TARGET_SPEED_MAX, 0.0f,
                    PID_LOCATION_P, PID_LOCATION_I, PID_LOCATION_D);
    pid_struct_init(&pids.steer_compensation,
                    0.01f, 0.0002f, 0.0f,
                    MOTOR_DIR_ERR_VALID_MAX, 0.0f,
                    PID_STEER_COMPENSATION_P, PID_STEER_COMPENSATION_I, PID_STEER_COMPENSATION_D);
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
    pid_get(&pids.steer_compensation, &ppids[27]);
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
    return control_val;
}

static float Control_SteerCompensation(void)
{
    float control_val = pid_calculate(&pids.steer_compensation, motor_dir_err[0], 0);
    return control_val;
}

void Control_SetSteerCompensation_basedon_Receive(void)
{
    float motor_dir_err_tmp; // 防止直接用 sscanf 读进去导致发生错误
    static uint8_t is_received_from_K210[STEER_COMPENSATION_DELAY] = {0};
    static uint8_t cnt = 0;

    memset(cmd, '\0', MainBuf_SIZE);
    is_received_from_K210[STEER_COMPENSATION_DELAY - 1] = 0;
    if (Receive_FindFirstVaildString(&uart_with_K210, cmd_start, cmd_end, cmd) == RECEIVE_SUCCESS)
    {
        if (sscanf(cmd, "%f", &motor_dir_err_tmp) != EOF)
        {
            if (motor_dir_err_tmp > -MOTOR_DIR_ERR_VALID_MAX && motor_dir_err_tmp < MOTOR_DIR_ERR_VALID_MAX)
            {
                motor_dir_err[STEER_COMPENSATION_DELAY - 1] = motor_dir_err_tmp;
                is_received_from_K210[STEER_COMPENSATION_DELAY - 1] = 1;
                cnt = 0;
            }
        }
    }

    if (is_received_from_K210[0])
        motor_steer_compensation_ratio = Control_SteerCompensation();

    for (uint8_t i = 1; i < STEER_COMPENSATION_DELAY; i++)
    {
        motor_dir_err[i - 1] = motor_dir_err[i];
        is_received_from_K210[i - 1] = is_received_from_K210[i];
    }

    if (cnt >= STEER_COMPENSATION_VALID_TIME) // 当超过 1000ms 没有读到数据时清空
    {
        cnt = 0;
        pid_clear(&pids.steer_compensation);
        motor_steer_compensation_ratio = 0;
        motor_speed_difference_set = 0;
    }
    cnt++;
}

static void Control_Move(void)
{
    static uint8_t control_location_count = 0;
    float motor1_speed_set_tmp = motor1_speed_set;
    float motor2_speed_set_tmp = motor2_speed_set;
    if (is_motor1_en == 1 || is_motor2_en == 1) // 电机在使能状态下才进行控制处理
    {
        control_location_count++;
        if (control_location_count >= 2)
        {
            control_location_count = 0;
            motor_speed_set = Control_Location();
        }

        // 转向补偿部分
        motor1_speed_set = motor_speed_set * (1.0f + motor_steer_compensation_ratio);
        motor2_speed_set = motor_speed_set * (1.0f - motor_steer_compensation_ratio);

        // 速度变化太快会打滑, 故采用此策略
        if (motor1_speed_set - motor1_speed_set_tmp > DELTA_SPEED_MAX)
            motor1_speed_set = motor1_speed_set_tmp + DELTA_SPEED_MAX;
        else if (motor1_speed_set - motor1_speed_set_tmp < -DELTA_SPEED_MAX)
            motor1_speed_set = motor1_speed_set_tmp - DELTA_SPEED_MAX;
        if (motor2_speed_set - motor2_speed_set_tmp > DELTA_SPEED_MAX)
            motor2_speed_set = motor2_speed_set_tmp + DELTA_SPEED_MAX;
        else if (motor2_speed_set - motor2_speed_set_tmp < -DELTA_SPEED_MAX)
            motor2_speed_set = motor2_speed_set_tmp - DELTA_SPEED_MAX;

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
#if IS_DEBUG_UART_TIME_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
    int32_t time = pid_cal_time_ref;
    int32_t time_start;
    int32_t time_end;
#endif // !IS_DEBUG_UART_TIME_FEEDBACK_ON
    Encoder_PulseGet();
    motor1_speed = ((float)encoder_motor1_pulsenum * 1000.0f * 60.0f) / (PULSE_PER_REVOLUTION * TIM_PID_INTERVAL);
    motor2_speed = ((float)encoder_motor2_pulsenum * 1000.0f * 60.0f) / (PULSE_PER_REVOLUTION * TIM_PID_INTERVAL);
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
    Control_Move();
#endif // !IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED && IS_DEBUG_UART_ON && IS_DEBUG_ON

    Motor_Output((int16_t)motor1_voltage, (int16_t)motor2_voltage);

#if IS_DEBUG_UART_REAL_TIME_MONITOR_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
    if (is_UART_working == 0)
    {
#if IS_DEBUG_UART_TIME_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
        time_start = pid_cal_time_ref - time;
        printf("printf start in %dms\r\n", time_start);
#endif // !IS_DEBUG_UART_TIME_FEEDBACK_ON
        motor_speed_difference_set = motor_steer_compensation_ratio * motor_speed_set;
        printf("motor: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",
               motor1_speed,
               motor2_speed,
               pids.speed.motor1.set,
               pids.speed.motor2.set,
               motor_dir_err[STEER_COMPENSATION_DELAY - 1] * 100,
               motor_speed_difference_set,
               motor1_location,
               motor2_location,
               motor_location_average,
               pids.location.set,
               debug_motor1_voltage,
               debug_motor2_voltage);
#if IS_DEBUG_UART_TIME_FEEDBACK_ON && IS_DEBUG_UART_ON && IS_DEBUG_ON
        time_end = pid_cal_time_ref - time;
        printf("printf end in %dms\r\n", time_end);
#endif // !IS_DEBUG_UART_TIME_FEEDBACK_ON
    }
#endif
}
