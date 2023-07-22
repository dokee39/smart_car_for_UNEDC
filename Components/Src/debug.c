/**
 * @file debug.c
 * @author dokee (dokee.39@gmail.com)
 * @brief 用于调试
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "receive.h"
#include "transmit.h"
#include "control.h"
#include "stm32f1xx_it.h"

#if IS_DEBUG_ON

/* 串口调试 */
#if IS_DEBUG_UART_ON

uint8_t is_UART_working = 0;

// 标志命令格式是否正确, 为函数 cmd_information_find() 的返回值
typedef enum
{
    CMD_FIND_SUCCESS,
    CMD_FIND_FAILURE,
} CMD_FIND_STATUS_t;

// 命令中数值的类型
typedef enum
{
    PID_VALUE_P_I_D,
    PID_VALUE_ERR_LIMIT,
    PID_VALUE_OUT_LIMIT,
    PID_VALUE_TARGET,
} PID_VALUE_TYPE_t;

// 包含命令传达的所有信息的结构体
typedef struct
{
    MOTOR_t MOTOR;
    PID_LOOP_t PID_LOOP;
    PID_VALUE_TYPE_t PID_VALUE_TYPE;
    float data[3];
} cmd_information_t;

// --> 教训: 不要把外层的 def 放在头文件里给内层的函数调用, 而应该 extern 内层的变量给外层的文件用
//     否则就会出现头文件互相包含的问题导致无法识别 identifier

static char *cmd_start = "<!"; // 命令包头
static char *cmd_end = ">!";   // 命令包尾
static char cmd[RxMainBuf_SIZE]; // 用于存放收到的命令

// 用于存放收到的命令的各部分
char motor_received[8];
char pid_contorller_received[20];
char pid_value_type_received[11];
char check_char_received[5];

#if IS_DEBUG_UART_PID_FEEDBACK_ON
// 用于存放 pid 的当前参数
float current_pids[36];
#endif

static cmd_information_t cmd_information; // 包含命令传达的所有信息的结构体

/**
 * @brief 验证命令是否正确以及将信息写入 cmd_information 结构体中
 *
 * @return cmd_find_status_t
 */
static CMD_FIND_STATUS_t cmd_information_find()
{
    uint8_t data_num = 0;

    int EOF_flag = sscanf(cmd,
                          "%s %s %s set:",
                          motor_received,
                          pid_contorller_received,
                          pid_value_type_received);

    if (EOF_flag == EOF)
        return CMD_FIND_FAILURE;

    if (strncmp(motor_received, "motor1", 6) == 0)
        cmd_information.MOTOR = MOTOR1;
    else if (strncmp(motor_received, "motor2", 6) == 0)
        cmd_information.MOTOR = MOTOR2;
    else if (strncmp(motor_received, "motor", 5) == 0)
        cmd_information.MOTOR = MOTOR_ALL;
    else
        return CMD_FIND_FAILURE;

    if (strncmp(pid_contorller_received, "speed", 5) == 0)
        cmd_information.PID_LOOP = PID_LOOP_SPEED;
    else if (strncmp(pid_contorller_received, "location", 8) == 0)
        cmd_information.PID_LOOP = PID_LOOP_LOCATION;
    else if (strncmp(pid_contorller_received, "steer_compensation", 18) == 0)
        cmd_information.PID_LOOP = PID_LOOP_STEER_COMPENSATION;
    else
        return CMD_FIND_FAILURE;

    if (strncmp(pid_value_type_received, "P_I_D", 5) == 0)
    {
        cmd_information.PID_VALUE_TYPE = PID_VALUE_P_I_D;
        data_num = 3;
    }
    else if (strncmp(pid_value_type_received, "err_limit", 9) == 0)
    {
        cmd_information.PID_VALUE_TYPE = PID_VALUE_ERR_LIMIT;
        data_num = 3;
    }
    else if (strncmp(pid_value_type_received, "out_limit", 9) == 0)
    {
        cmd_information.PID_VALUE_TYPE = PID_VALUE_OUT_LIMIT;
        data_num = 2;
    }
    else if (strncmp(pid_value_type_received, "target", 5) == 0)
    {
        cmd_information.PID_VALUE_TYPE = PID_VALUE_TARGET;
        data_num = 1;
    }
    else
        return CMD_FIND_FAILURE;

    char *data_start_pos = strchr(cmd, ':');
    if (data_start_pos == NULL)
        return CMD_FIND_FAILURE;

    if (data_num == 3)
    {
        EOF_flag = sscanf(data_start_pos,
                          ": %f, %f, %f %s",
                          &cmd_information.data[0],
                          &cmd_information.data[1],
                          &cmd_information.data[2],
                          check_char_received);
    }
    if (data_num == 2)
    {
        EOF_flag = sscanf(data_start_pos,
                          ": %f, %f %s",
                          &cmd_information.data[0],
                          &cmd_information.data[1],
                          check_char_received);
    }
    if (data_num == 1)
    {
        EOF_flag = sscanf(data_start_pos,
                          ": %f %s",
                          &cmd_information.data[0],
                          check_char_received);
    }

    if (EOF_flag == EOF)
        return CMD_FIND_FAILURE;

    if (strncmp(check_char_received, "OK?", 3) != 0)
        return CMD_FIND_FAILURE;

    return CMD_FIND_SUCCESS;
}

#if IS_DEBUG_UART_CMD_FEEDBACK_ON
/**
 * @brief 在读到正确的命令后串口返回读取到的信息
 *
 * @param time
 */
static void cmd_feedback(int32_t time)
{
    Transmit_printf(&uart_transmit_for_debug, "sent after %dms -->  ", time);
    Transmit_printf(&uart_transmit_for_debug, "received-> ");

    switch (cmd_information.MOTOR)
    {
    case MOTOR1:
        Transmit_printf(&uart_transmit_for_debug, "motor1 ");
        break;
    case MOTOR2:
        Transmit_printf(&uart_transmit_for_debug, "motor2 ");
        break;
    case MOTOR_ALL:
        Transmit_printf(&uart_transmit_for_debug, "motor ");
        break;
    }

    switch (cmd_information.PID_LOOP)
    {
    case PID_LOOP_SPEED:
        Transmit_printf(&uart_transmit_for_debug, "speed ");
        break;
    case PID_LOOP_LOCATION:
        Transmit_printf(&uart_transmit_for_debug, "location ");
        break;
    case PID_LOOP_STEER_COMPENSATION:
        Transmit_printf(&uart_transmit_for_debug, "steer_compensation ");
        break;
    }

    switch (cmd_information.PID_VALUE_TYPE)
    {
    case PID_VALUE_P_I_D:
        Transmit_printf(&uart_transmit_for_debug, "P_I_D ");
        Transmit_printf(&uart_transmit_for_debug, "set-> ");
        Transmit_printf(&uart_transmit_for_debug, "%.2f, %.2f, %.2f OK!\r\n", cmd_information.data[0], cmd_information.data[1], cmd_information.data[2]);
        break;
    case PID_VALUE_ERR_LIMIT:
        Transmit_printf(&uart_transmit_for_debug, "err_limit ");
        Transmit_printf(&uart_transmit_for_debug, "set-> ");
        Transmit_printf(&uart_transmit_for_debug, "%.2f, %.2f, %.2f OK!\r\n", cmd_information.data[0], cmd_information.data[1], cmd_information.data[2]);
        break;
    case PID_VALUE_OUT_LIMIT:
        Transmit_printf(&uart_transmit_for_debug, "out_limit ");
        Transmit_printf(&uart_transmit_for_debug, "set-> ");
        Transmit_printf(&uart_transmit_for_debug, "%.2f, %.2f, OK!\r\n", cmd_information.data[0], cmd_information.data[1]);
        break;
    case PID_VALUE_TARGET:
        Transmit_printf(&uart_transmit_for_debug, "target ");
        Transmit_printf(&uart_transmit_for_debug, "set-> ");
        Transmit_printf(&uart_transmit_for_debug, "%.2f OK!\r\n", cmd_information.data[0]);
        break;
    }
}
#endif // !IS_DEBUG_UART_CMD_FEEDBACK_ON

#if IS_DEBUG_UART_PID_FEEDBACK_ON
static void pid_feedback()
{
    Control_PIDs_Get(current_pids);
    Transmit_printf(&uart_transmit_for_debug, "sent after %dms -->  This is the current value of the pids ->\r\n", receive_time_ref);
    Transmit_printf(&uart_transmit_for_debug, "    -->  SPEED LOOP ->\r\n");
    Transmit_printf(&uart_transmit_for_debug, "        -->  MOTOR1 -> <kp> %.2f, <ki> %.2f, <kd> %.2f\r\n", current_pids[0], current_pids[1], current_pids[2]);
    Transmit_printf(&uart_transmit_for_debug, "                       <input_max_err> %.2f, <input_min_err> %.2f, <integral_separate_err> %.2f\r\n", current_pids[3], current_pids[4], current_pids[5]);
    Transmit_printf(&uart_transmit_for_debug, "                       <max_out> %.2f, <integral_limit> %.2f\r\n", current_pids[6], current_pids[7]);
    Transmit_printf(&uart_transmit_for_debug, "                       <set> %.2f\r\n", current_pids[8]);
    Transmit_printf(&uart_transmit_for_debug, "        -->  MOTOR2 -> <kp> %.2f, <ki> %.2f, <kd> %.2f\r\n", current_pids[9], current_pids[10], current_pids[11]);
    Transmit_printf(&uart_transmit_for_debug, "                       <input_max_err> %.2f, <input_min_err> %.2f, <integral_separate_err> %.2f\r\n", current_pids[12], current_pids[13], current_pids[14]);
    Transmit_printf(&uart_transmit_for_debug, "                       <max_out> %.2f, <integral_limit> %.2f\r\n", current_pids[15], current_pids[16]);
    Transmit_printf(&uart_transmit_for_debug, "                       <set> %.2f\r\n", current_pids[17]);
    Transmit_printf(&uart_transmit_for_debug, "    -->  LOCATION LOOP ->\r\n");
    Transmit_printf(&uart_transmit_for_debug, "                       <kp> %.2f, <ki> %.2f, <kd> %.2f\r\n", current_pids[18], current_pids[19], current_pids[20]);
    Transmit_printf(&uart_transmit_for_debug, "                       <input_max_err> %.2f, <input_min_err> %.2f, <integral_separate_err> %.2f\r\n", current_pids[21], current_pids[22], current_pids[23]);
    Transmit_printf(&uart_transmit_for_debug, "                       <max_out> %.2f, <integral_limit> %.2f\r\n", current_pids[24], current_pids[25]);
    Transmit_printf(&uart_transmit_for_debug, "                       <set> %.2f\r\n", current_pids[26]);
    Transmit_printf(&uart_transmit_for_debug, "    -->  STEER_COMPENSATION LOOP ->\r\n");
    Transmit_printf(&uart_transmit_for_debug, "                       <kp> %.2f, <ki> %.2f, <kd> %.2f\r\n", current_pids[27], current_pids[28], current_pids[29]);
    Transmit_printf(&uart_transmit_for_debug, "                       <input_max_err> %.2f, <input_min_err> %.2f, <integral_separate_err> %.2f\r\n", current_pids[30], current_pids[31], current_pids[32]);
    Transmit_printf(&uart_transmit_for_debug, "                       <max_out> %.2f, <integral_limit> %.2f\r\n", current_pids[33], current_pids[34]);
    Transmit_printf(&uart_transmit_for_debug, "                       <set> %.2f\r\n", current_pids[35]);
}
#endif // !IS_DEBUG_UART_PID_FEEDBACK_ON

static CMD_FIND_STATUS_t pid_set(void)
{
    pid_t *ppid;
    if (cmd_information.PID_LOOP == PID_LOOP_SPEED)
    {
        if (cmd_information.MOTOR == MOTOR1)
            ppid = &pids.speed.motor1;
        else if (cmd_information.MOTOR == MOTOR2)
            ppid = &pids.speed.motor2;
        else 
            return CMD_FIND_FAILURE;
    }
    else if (cmd_information.PID_LOOP == PID_LOOP_LOCATION)
    {
        if (cmd_information.MOTOR == MOTOR_ALL)
            ppid = &pids.location;
        else
            return CMD_FIND_FAILURE;
    }
    else if (cmd_information.PID_LOOP == PID_LOOP_STEER_COMPENSATION)
    {
        if (cmd_information.MOTOR == MOTOR_ALL)
            ppid = &pids.steer_compensation;
        else
            return CMD_FIND_FAILURE;
    }
    else
    {
        return CMD_FIND_FAILURE;
    }

    switch (cmd_information.PID_VALUE_TYPE)
    {
    case PID_VALUE_P_I_D:
        pid_set_P_I_D(ppid, cmd_information.data[0], cmd_information.data[1], cmd_information.data[2]);
        break;
    case PID_VALUE_ERR_LIMIT:
        pid_set_err_limit(ppid, cmd_information.data[0], cmd_information.data[1], cmd_information.data[2]);
        break;
    case PID_VALUE_OUT_LIMIT:
        pid_set_out_limit(ppid, cmd_information.data[0], cmd_information.data[1]);
        break;
    case PID_VALUE_TARGET:
        pid_set_target(ppid, cmd_information.data[0]);
        break;
    default:
        return CMD_FIND_FAILURE;
    }
    return CMD_FIND_SUCCESS;
}

/**
 * @brief 根据命令调节 PID
 *
 */
void Debug_SetPID_basedon_Receive(void)
{
    receive_time_ref = 0;
    memset(cmd, '\0', RxMainBuf_SIZE);
    if (Receive_FindFirstVaildString(&uart_receive_for_debug, cmd_start, cmd_end, cmd) == RECEIVE_SUCCESS)
    {
        if (cmd_information_find() == CMD_FIND_SUCCESS)
        {
            is_UART_working = 1;

#if IS_DEBUG_UART_CMD_FEEDBACK_ON
            cmd_feedback(receive_time_ref);
#endif // !IS_DEBUG_UART_CMD_FEEDBACK_ON

            if (pid_set() == CMD_FIND_SUCCESS)
            {
                Transmit_printf(&uart_transmit_for_debug, "sent after %dms -->  PID_SET_OK!\r\n", receive_time_ref);
#if IS_DEBUG_UART_PID_FEEDBACK_ON
                pid_feedback();
#endif // !IS_DEBUG_UART_PID_FEEDBACK_ON
            }
            else
            {
                Transmit_printf(&uart_transmit_for_debug, "sent after %dms -->  COMMAND_ERROR!\r\n", receive_time_ref);
            }
            is_UART_working = 0;
        }
        else
        {
            Transmit_printf(&uart_transmit_for_debug, "sent after %dms -->  COMMAND_ERROR!\r\n", receive_time_ref);
        }
    }
}

#endif // !IS_DEBUG_UART_ON

#if IS_DEBUG_LED_RUN_ON
void Debug_LED_run_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_run_GPIO_Port, LED_run_Pin);
}
#endif //! IS_DEBUG_LED_RUN_ON

#endif // !IS_DEBUG_ON
