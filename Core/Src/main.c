/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#if IS_DEBUG_UART_ON && IS_DEBUG_ON
#include "receive.h"
#endif
#include "control.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */
    // IN the Generated code, make sure the DMA is initialized before the UART.
    // CubeMX sometimes does the opposite and in that case, DMA won't work

    // Timer Start
    HAL_TIM_Encoder_Start(&htim_Motor1_Encoder, TIM_CHANNEL_ENCODER_1);
    HAL_TIM_Encoder_Start(&htim_Motor1_Encoder, TIM_CHANNEL_ENCODER_2);
    HAL_TIM_Encoder_Start(&htim_Motor2_Encoder, TIM_CHANNEL_ENCODER_1);
    HAL_TIM_Encoder_Start(&htim_Motor2_Encoder, TIM_CHANNEL_ENCODER_2);
    HAL_TIM_Base_Start_IT(&htim_Debug_LED_Interval);
    HAL_TIM_Base_Start_IT(&htim_PID_Interval);

    // PID Param Init
    Control_PID_Init();

#if IS_DEBUG_UART_ON && IS_DEBUG_ON
    Receive_BufInit();

#if IS_DEBUG_UART_PID_LOOP_SPEED || IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED
    Motor_Enable(MOTOR1);
    Motor_Enable(MOTOR2);
#endif // !IS_DEBUG_UART_PID_LOOP_SPEED || IS_DEBUG_UART_PID_LOOP_LOCATION_SPEED
#endif // !IS_DEBUG_UART_ON && IS_DEBUG_ON

#if IS_DEBUG_IN_MAIN_C_ON
    HAL_Delay(5000);
    pid_set_target(&pids.location.motor1, 100);
    pid_set_target(&pids.location.motor2, 100);
#endif // !IS_DEBUG_IN_MAIN_C_ON

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
#if IS_DEBUG_UART_ON && IS_DEBUG_ON
        Debug_SetPIDbasedonReceive(1000);
#endif

#if IS_DEBUG_IN_MAIN_C_ON
        if (pids.speed.motor1.set <= 1 && pids.speed.motor1.set >= -1)
        {
            HAL_Delay(2000);
            pid_set_target(&pids.location.motor1, -pids.location.motor1.set);
            pid_set_target(&pids.location.motor2, -pids.location.motor2.set);
        }
#endif // !IS_DEBUG_IN_MAIN_C_ON
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
