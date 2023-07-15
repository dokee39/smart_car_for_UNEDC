/**
 * @file pid.h
 * @author dokee (dokee.39@gmail.com)
 * @brief PID 参数设置与计算
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 * @note 参考 RoboRTS-Firmware
 */

#ifndef __PID_H__
#define __PID_H__

#include "stdint.h"
#include "debug.h"

typedef struct
{
    float p;
    float i;
    float d;

    float input_max_err;
    float input_min_err;
    float integral_separate_err;

    float max_out;
    float integral_limit;
} pid_param_t;

typedef struct pid
{
    pid_param_t param;

    uint8_t enable;

    float set;
    float get;

    float err;
    float last_err;

    float pout;
    float iout;
    float dout;
    float out;

    void (*f_param_init)(struct pid *pid,
                         float input_max_err,
                         float input_min_err,
                         float integral_separate_err,
                         float max_output,
                         float integral_limit,
                         float p,
                         float i,
                         float d);
    void (*f_pid_reset)(struct pid *pid, float p, float i, float d);
} pid_t;

void pid_struct_init(
    struct pid *pid,
    float input_max_err,
    float input_min_err,
    float integral_separate_err,
    float maxout,
    float intergral_limit,

    float kp,
    float ki,
    float kd);

void pid_clear(pid_t *pid);

float pid_calculate(struct pid *pid, float fdb, float ref);

#if IS_DEBUG_UART_ON && IS_DEBUG_ON
void pid_set_P_I_D(struct pid *pid, float kp, float ki, float kd);
void pid_set_err_limit(struct pid *pid, float input_max_err, float input_min_err, float integral_separate_err);
void pid_set_out_limit(struct pid *pid, float max_out, float integral_limit);
#endif

void pid_set_target(struct pid *pid, float target);

#endif // !__PID_H__
