/**
 * @file encoder.c
 * @author dokee (dokee.39@gmail.com)
 * @brief 编码器脉冲数获取
 * @version 0.1
 * @date 2023-07-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "encoder.h"
#include "stm32f1xx.h"
#include "tim.h"
#include "usart.h"

int32_t encoder_motor1_pulsenum_sum = 0; // 电机脉冲数累计值, 完成一次位置控制之后才清除
int32_t encoder_motor2_pulsenum_sum = 0; // 电机脉冲数累计值, 完成一次位置控制之后才清除
int16_t encoder_motor1_pulsenum = 0; // 单次电机脉冲数值
int16_t encoder_motor2_pulsenum = 0; // 单次电机脉冲数值

/**
 * @brief 读取电机脉冲数
 * @note 如果编码器有时候捕获的数值会跳变的话，可以在 CubeMX 的编码器设置里面加上滤波, 可以设置 1~16 的值
 * @note 当两个轮子转向不一致的时候，要同时调这里编码器捕获脉冲的正负和 control.h 里的 PWM 通道宏定义
 */
void Encoder_PulseGet(void)
{
    encoder_motor1_pulsenum = (short)(__HAL_TIM_GET_COUNTER(&htim_Motor1_Encoder)); // 获取计数器值   为什么现在可以两个都是正？

    __HAL_TIM_SET_COUNTER(&htim_Motor1_Encoder, 0); // 计数器清零

    encoder_motor2_pulsenum = (short)(__HAL_TIM_GET_COUNTER(&htim_Motor2_Encoder)); // 获取计数器值
    encoder_motor2_pulsenum = -encoder_motor2_pulsenum;
    __HAL_TIM_SET_COUNTER(&htim_Motor2_Encoder, 0); // 计数器清零

    encoder_motor1_pulsenum_sum += encoder_motor1_pulsenum;
    encoder_motor2_pulsenum_sum += encoder_motor2_pulsenum;
}


