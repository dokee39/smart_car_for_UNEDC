/**
 * @file pid.c
 * @author dokee (dokee.39@gmail.com)
 * @brief PID 参数设置与计算
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 * @note 参考 RoboRTS-Firmware
 */

#include <math.h>
#include "pid.h"
#include "debug.h"

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
    {
        *a = ABS_MAX;
    }
    if (*a < -ABS_MAX)
    {
        *a = -ABS_MAX;
    }
}

static void pid_param_init(
    pid_t *pid,
    float input_max_err,
    float input_min_err,
    float integral_separate_err,
    float maxout,
    float integral_limit,
    float kp,
    float ki,
    float kd)
{
    pid->param.input_max_err = input_max_err;
    pid->param.input_min_err = input_min_err;
    pid->param.integral_separate_err = integral_separate_err;

    pid->param.max_out = maxout;
    pid->param.integral_limit = integral_limit;

    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
}

/**
 * @brief     modify pid parameter when code running
 * @param[in] pid: control pid struct
 * @param[in] p/i/d: pid parameter
 * @retval    none
 */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

void pid_clear(pid_t *pid)
{
    pid->err = 0;
    pid->iout = 0;
}

/**
 * @brief     initialize pid parameter
 * @retval    none
 */
void pid_struct_init(
    struct pid *pid,
    float input_max_error,
    float input_min_error,
    float integral_separate_err,

    float maxout,
    float integral_limit,

    float kp,
    float ki,
    float kd)
{
    pid->enable = 1;
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;

    pid->f_param_init(pid, input_max_error, input_min_error, integral_separate_err, maxout, integral_limit, kp, ki, kd);
    pid->f_pid_reset(pid, kp, ki, kd);
}

#if IS_DEBUG_UART_ON && IS_DEBUG_ON

void pid_set_P_I_D(struct pid *pid, float kp, float ki, float kd)
{
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
}

void pid_set_err_limit(struct pid *pid, float input_max_err, float input_min_err, float integral_separate_err)
{
    pid->param.input_max_err = input_max_err;
    pid->param.input_min_err = input_min_err;
    pid->param.integral_separate_err = integral_separate_err;
}

void pid_set_out_limit(struct pid *pid, float max_out, float integral_limit)
{
    pid->param.max_out = max_out;
    pid->param.integral_limit = integral_limit;
}

#endif

void pid_set_target(struct pid *pid, float target)
{
    pid->set = target;
}

/**
 * @brief     calculate delta PID and position PID
 * @param[in] pid: control pid struct
 * @param[in] get: measure feedback value
 * @param[in] set: target value
 * @retval    pid calculate output
 */
float pid_calculate(struct pid *pid, float get, float set)
{
    pid->last_err = pid->err;
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    if ((pid->param.input_max_err != 0) && (fabs(pid->err) > pid->param.input_max_err))
    {
        return 0;
    }
    if ((pid->param.input_min_err != 0) && (fabs(pid->err) < pid->param.input_min_err))
        pid->err = 0;

    pid->pout = pid->param.p * pid->err;
    if ((pid->param.integral_separate_err != 0) && (fabs(pid->err) > pid->param.integral_separate_err))
    {
        pid->iout = 0;
    }
    else
    {
        pid->iout += pid->param.i * pid->err;
    }
    pid->dout = pid->param.d * (pid->err - pid->last_err);

    abs_limit(&(pid->iout), pid->param.integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->param.max_out);

    if (pid->enable == 0)
    {
        pid->out = 0;
    }

    return pid->out;
}
