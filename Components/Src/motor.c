/**
 * @file motor.c
 * @author dokee (dokee.39@gmail.com)
 * @brief 电机控制
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "main.h"
#include "tim.h"
#include "motor.h"

uint8_t is_motor1_en = 0;
uint8_t is_motor2_en = 0;

static MOTOR_DIR_t motor1_dir = MOTOR_STOP;
static MOTOR_DIR_t motor2_dir = MOTOR_STOP;

static uint16_t motor1_duty = 0;
static uint16_t motor2_duty = 0;

/**
 * @brief 重置并开始电机的 PWM 输出
 *
 * @param MOTOR
 */
void Motor_Enable(MOTOR_t MOTOR)
{
    if (MOTOR == MOTOR1)
    {
        is_motor1_en = 1;
        __MOTOR1_INA_RESET;
        __MOTOR1_INB_RESET;
        __MOTOR1_DUTY_SET(0);
        __MOTOR1_ENABLE;
    }
    else
    {
        is_motor2_en = 1;
        __MOTOR2_INA_RESET;
        __MOTOR2_INB_RESET;
        __MOTOR2_DUTY_SET(0);
        __MOTOR2_ENABLE;
    }
}

/**
 * @brief 重置并关闭电机的 PWM 输出
 *
 * @param MOTOR
 */
void Motor_Disable(MOTOR_t MOTOR)
{
    if (MOTOR == MOTOR1)
    {
        is_motor1_en = 0;
        __MOTOR1_INA_RESET;
        __MOTOR1_INB_RESET;
        __MOTOR1_DISABLE;
        __MOTOR1_DUTY_SET(0);
    }
    else
    {
        is_motor2_en = 0;
        __MOTOR2_INA_RESET;
        __MOTOR2_INB_RESET;
        __MOTOR1_DISABLE;
        __MOTOR2_DUTY_SET(0);
    }
}

/**
 * @brief 设置电机方向
 * 
 * @param MOTOR 
 * @param MOTOR_DIR 
 */
void Motor_DirSet(MOTOR_t MOTOR, MOTOR_DIR_t MOTOR_DIR)
{

    if (MOTOR == MOTOR1)
    {
        motor1_dir = MOTOR_DIR;
        switch (motor1_dir)
        {
        case MOTOR_STOP:
            __MOTOR1_INB_RESET;
            __MOTOR1_INA_RESET;
            break;
        case MOTOR_FWD:
            __MOTOR1_INB_RESET;
            __MOTOR1_INA_SET;
            break;
        case MOTOR_REV:
            __MOTOR1_INA_RESET;
            __MOTOR1_INB_SET;
        }
    }
    else
    {
        motor2_dir = MOTOR_DIR;
        switch (motor2_dir)
        {
        case MOTOR_STOP:
            __MOTOR2_INB_RESET;
            __MOTOR2_INA_RESET;
            break;
        case MOTOR_FWD:
            __MOTOR2_INB_RESET;
            __MOTOR2_INA_SET;
            break;
        case MOTOR_REV:
            __MOTOR2_INA_RESET;
            __MOTOR2_INB_SET;
        }
    }
}

/**
 * @brief 设置电机 PWM 的占空比
 * 
 * @param MOTOR 
 * @param duty 
 */
void Motor_DutySet(MOTOR_t MOTOR, uint16_t duty)
{
    if (MOTOR == MOTOR1)
    {
        motor1_duty = duty;
        __MOTOR1_DUTY_SET(motor1_duty);
    } else {
        motor2_duty = duty;
        __MOTOR2_DUTY_SET(motor2_duty);
    }
}

/**
 * @brief 更新两个电机的占空比
 * 
 * @param motor1_PWM_duty 
 * @param motor2_PWM_duty 
 */
void Motor_Output(int16_t motor1_PWM_duty, int16_t motor2_PWM_duty)
{
    if (motor1_PWM_duty >= 0)
    {
        Motor_DirSet(MOTOR1, MOTOR_FWD);
    }
    else
    {
        motor1_PWM_duty = -motor1_PWM_duty;
        Motor_DirSet(MOTOR1, MOTOR_REV);
    }
    motor1_PWM_duty = (motor1_PWM_duty > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : motor1_PWM_duty; // 速度上限处理

    if (motor2_PWM_duty >= 0) // 判断电机方向
    {
        Motor_DirSet(MOTOR2, MOTOR_FWD); // 正方向要对应
    }
    else
    {
        motor2_PWM_duty = -motor2_PWM_duty;
        Motor_DirSet(MOTOR2, MOTOR_REV); // 正方向要对应
    }

    motor2_PWM_duty = (motor2_PWM_duty > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : motor2_PWM_duty; // 速度上限处理

    Motor_DutySet(MOTOR1, motor1_PWM_duty);  // 设置 PWM 占空比
    Motor_DutySet(MOTOR2, motor2_PWM_duty); // 设置 PWM 占空比
}
