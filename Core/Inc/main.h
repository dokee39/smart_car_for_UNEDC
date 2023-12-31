/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "stdint.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define huart_with_K210 huart2
#define hdma_usart_with_K210_rx hdma_usart2_rx
#define hdma_usart_for_debug_rx hdma_usart1_rx
#define huart_for_debug huart1
#define hdma_usart_for_debug_tx hdma_usart1_tx
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOD
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOD
#define USART_with_K210_TX_Pin GPIO_PIN_2
#define USART_with_K210_TX_GPIO_Port GPIOA
#define USART_with_K210_RX_Pin GPIO_PIN_3
#define USART_with_K210_RX_GPIO_Port GPIOA
#define Motor1_INA_Pin GPIO_PIN_0
#define Motor1_INA_GPIO_Port GPIOB
#define Motor1_INB_Pin GPIO_PIN_1
#define Motor1_INB_GPIO_Port GPIOB
#define Motor2_EncoderA_Pin GPIO_PIN_6
#define Motor2_EncoderA_GPIO_Port GPIOC
#define Motor2_EncoderB_Pin GPIO_PIN_7
#define Motor2_EncoderB_GPIO_Port GPIOC
#define Motor2_INA_Pin GPIO_PIN_8
#define Motor2_INA_GPIO_Port GPIOC
#define Motor2_INB_Pin GPIO_PIN_9
#define Motor2_INB_GPIO_Port GPIOC
#define LED_run_Pin GPIO_PIN_8
#define LED_run_GPIO_Port GPIOA
#define USART_for_debug_TX_Pin GPIO_PIN_9
#define USART_for_debug_TX_GPIO_Port GPIOA
#define USART_for_debug_RX_Pin GPIO_PIN_10
#define USART_for_debug_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Motor1_EncoderA_Pin GPIO_PIN_4
#define Motor1_EncoderA_GPIO_Port GPIOB
#define Motor1_EncoderB_Pin GPIO_PIN_5
#define Motor1_EncoderB_GPIO_Port GPIOB
#define Motor1_PWM_Pin GPIO_PIN_8
#define Motor1_PWM_GPIO_Port GPIOB
#define Motor2_PWM_Pin GPIO_PIN_9
#define Motor2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// 定时器马甲
#define htim_Motor1_Encoder htim3
#define htim_Motor2_Encoder htim8
#define htim_Motor_PWM htim4
#define htim_PID_Interval htim7
#define htim_Task_Interval htim6

// 定时器时间间隔
#define TIM_PID_INTERVAL 20.0f
#define TIM_TASK_INTERVAL 10u

// 定时器通道马甲
#define TIM_CHANNEL_ENCODER_1 TIM_CHANNEL_1
#define TIM_CHANNEL_ENCODER_2 TIM_CHANNEL_2
#define TIM_CHANNEL_MOTOR1_PWM TIM_CHANNEL_3
#define TIM_CHANNEL_MOTOR2_PWM TIM_CHANNEL_4

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
