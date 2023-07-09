/**
 * @file encoder.h
 * @author dokee (dokee.39@gmail.com)
 * @brief 编码器脉冲数获取
 * @version 0.1
 * @date 2023-07-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

extern int32_t encoder_motor1_pulsenum_sum; // 电机脉冲数累计值, 完成一次位置控制之后才清除
extern int32_t encoder_motor2_pulsenum_sum; // 电机脉冲数累计值, 完成一次位置控制之后才清除
extern int16_t encoder_motor1_pulsenum; // 单次电机脉冲数值
extern int16_t encoder_motor2_pulsenum; // 单次电机脉冲数值

void Encoder_PulseGet(void);

#endif