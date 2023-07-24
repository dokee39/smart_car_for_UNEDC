/**
 * @file motor.h
 * @author dokee (dokee.39@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"

/**************** 左边电机 Motor1 配置 **************/
// INA 与 INB 高低电平设置
#define __MOTOR1_INA_SET HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, GPIO_PIN_SET)
#define __MOTOR1_INA_RESET HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, GPIO_PIN_RESET)
#define __MOTOR1_INB_SET HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, GPIO_PIN_SET)
#define __MOTOR1_INB_RESET HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, GPIO_PIN_RESET)

// 设置 PWM 占空比
#define __MOTOR1_DUTY_SET(duty) __HAL_TIM_SET_COMPARE(&htim_Motor_PWM, TIM_CHANNEL_MOTOR1_PWM, duty)

// 使能 PWM 输出
#define __MOTOR1_ENABLE HAL_TIM_PWM_Start(&htim_Motor_PWM, TIM_CHANNEL_MOTOR1_PWM);

// 禁用 PWM 输出
#define __MOTOR1_DISABLE HAL_TIM_PWM_Stop(&htim_Motor_PWM, TIM_CHANNEL_MOTOR1_PWM);

/**************** 右边电机 Motor2 配置 **************/
// INA 与 INB 高低电平设置
#define __MOTOR2_INA_SET HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, GPIO_PIN_SET)
#define __MOTOR2_INA_RESET HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, GPIO_PIN_RESET)
#define __MOTOR2_INB_SET HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, GPIO_PIN_SET)
#define __MOTOR2_INB_RESET HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, GPIO_PIN_RESET)

// 设置 PWM 占空比
#define __MOTOR2_DUTY_SET(duty) __HAL_TIM_SET_COMPARE(&htim_Motor_PWM, TIM_CHANNEL_MOTOR2_PWM, duty)

// 使能 PWM 输出
#define __MOTOR2_ENABLE HAL_TIM_PWM_Start(&htim_Motor_PWM, TIM_CHANNEL_MOTOR2_PWM);

// 禁用 PWM 输出
#define __MOTOR2_DISABLE HAL_TIM_PWM_Stop(&htim_Motor_PWM, TIM_CHANNEL_MOTOR2_PWM);

/**************** PWM 占空比限制 ********************/
#define PWM_COUNTER_PERIOD (2048 - 1)
#define MOTOR_DUTY_MAX (PWM_COUNTER_PERIOD - 30)
#define MOTOR_VOLTAGE 12

// 左电机还是右电机
typedef enum
{
    MOTOR1,
    MOTOR2,
    MOTOR_ALL // 不对应实际的电机, 而表示小车整体
} MOTOR_t;

// 电机方向
typedef enum
{
    MOTOR_STOP,
    MOTOR_FWD,
    MOTOR_REV
} MOTOR_DIR_t;

extern uint8_t is_motor1_en;
extern uint8_t is_motor2_en;

void Motor_Enable(MOTOR_t MOTOR);
void Motor_Disable(MOTOR_t MOTOR);
void Motor_Output(int16_t motor1_PWM_duty, int16_t motor2_PWM_duty);

#endif
