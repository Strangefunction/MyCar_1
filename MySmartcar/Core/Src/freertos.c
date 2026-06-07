/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : freertos.c
 * @brief          : 麦轮精准控制系统 - 增加航向角(Yaw)闭环纠偏功能
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fifo.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "pid.h"
#include "mpu9250.h"
#include "WouoUI.h"
#include "WouoUI_user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId sensorTaskHandle;
osThreadId pidTaskHandle;
osThreadId oledTaskHandle;
osThreadId uart2TaskHandle;
osThreadId defaultTaskHandle;

// 外部硬件变量
extern int16_t encoderValue1, encoderValue2, encoderValue3, encoderValue4;                        //  编码器获得的速度
extern int targetSpeed_1, targetSpeed_2, targetSpeed_3, targetSpeed_4;                            //  目标速度（位置环的输出，作为速度环的输入）
extern long long int encoder_pulses_1, encoder_pulses_2, encoder_pulses_3, encoder_pulses_4;      //  编码器获取的脉冲数
extern PID_Position wheel1_pid, wheel2_pid, wheel3_pid, wheel4_pid;                               //  电机位置环pid
extern UART_HandleTypeDef huart2;                                                                 //  串口2与esp通讯
extern uint8_t rxBuffer1[];                                                                       //  接收缓存区

//  运动控制变量
float vx_set = 0.0f;
float vy_set = 0.0f;
float vw_set = 0.0f;
//  目标脉冲总数
float Total_angle_1 = 0;
float Total_angle_2 = 0;
float Total_angle_3 = 0;
float Total_angle_4 = 0;
//  车辆自稳定变量
uint8_t car_ready = 0;

// --- 航向锁定(Yaw-Hold) 核心变量 ---
Attitude_t my_car_angle = {0};
float target_yaw = 0.0f;        // 目标锁定角度
float yaw_correction = 0.0f;    // PID计算出的纠偏旋转速度
// PID参数：Kp=2.0, Ki=0, Kd=0.5 角度环pid
PID_Position yaw_pid = {1.5f, 0.0f, 0.4f, 0, 0};

/* USER CODE END PTD */

/* Private function prototypes -----------------------------------------------*/
void StartSensorTask(void const * argument);
void StartPidTask(void const * argument);
void StartOledTask(void const * argument);
void StartUart2Task(void const * argument);
void StartDefaultTask(void const * argument);

void mecanum_calc(float vx, float vy, float vw, float *w1, float *w2, float *w3, float *w4);
void Reset_All_Coordinates(void);

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS初始化并创建任务
  */
void MX_FREERTOS_Init(void)
{
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadDef(pidTask, StartPidTask, osPriorityHigh, 0, 256);
    pidTaskHandle = osThreadCreate(osThread(pidTask), NULL);

    osThreadDef(sensorTask, StartSensorTask, osPriorityAboveNormal, 0, 512);
    sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

    osThreadDef(uart2Task, StartUart2Task, osPriorityNormal, 0, 512);
    uart2TaskHandle = osThreadCreate(osThread(uart2Task), NULL);

    osThreadDef(oledTask, StartOledTask, osPriorityLow, 0, 512);
    oledTaskHandle = osThreadCreate(osThread(oledTask), NULL);
}

/**
  * @brief  序列执行任务：处理启动序列
  */
void StartDefaultTask(void const * argument)
{
    vTaskDelay(500); // 等待传感器稳定

    // 关键：上电执行陀螺仪校准（此时确保小车静止）
    MPU9250_CalibrateGyro();

    Reset_All_Coordinates();

    for(;;)
    {
        vTaskDelay(1000);
    }
}

/* USER CODE BEGIN Application */

/**
  * @brief  传感器采集任务 (10ms)：负责编码器读取与姿态解算
  */
void StartSensorTask(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_run_time = HAL_GetTick();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

        // 计算真实 dt
        uint32_t current_time = HAL_GetTick();
        float actual_dt = (current_time - last_run_time) / 1000.0f;
        last_run_time = current_time;
        if(actual_dt <= 0.0f || actual_dt > 0.1f) actual_dt = 0.01f;

        // 1. 读取硬件编码器并更新累加器
        encoderValue1 = Read_Encoder_1();
        encoderValue2 = Read_Encoder_2();
        encoderValue3 = Read_Encoder_3();
        encoderValue4 = Read_Encoder_4();
        encoder_real();

        encoder_pulses_1 += encoderValue1;
        encoder_pulses_2 += encoderValue2;
        encoder_pulses_3 += encoderValue3;
        encoder_pulses_4 += encoderValue4;

        // 2. 姿态解算_卡尔曼滤波之后的结果
        MPU9250_Get_Attitude(&my_car_angle, actual_dt);
    }
}

/**
  * @brief  PID控制核心任务 (10ms)：实现航向角闭环与麦轮逆解
  */
void StartPidTask(void const * argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float w1, w2, w3, w4;

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        //vx_set = 5;  //  测试
        if(!car_ready)
        {
            Total_angle_1 = (float)encoder_pulses_1;
            Total_angle_2 = (float)encoder_pulses_2;
            Total_angle_3 = (float)encoder_pulses_3;
            Total_angle_4 = (float)encoder_pulses_4;
            target_yaw = my_car_angle.Yaw;
            continue;
        }

        // --- 航向锁定逻辑开始 ---
        // 如果外部有明显的旋转指令(vw_set)，则认为用户正在转向，更新目标角
        if(fabs(vw_set) > 0.1f)
        {
            target_yaw = my_car_angle.Yaw;
            yaw_correction = 0;
        }
        else
        {
            // 否则，锁定航向：计算当前偏航误差
            float yaw_error = target_yaw - my_car_angle.Yaw;

            // 环形角度差值修正 (-180~180)
            if(yaw_error > 180.0f)  yaw_error -= 360.0f;
            if(yaw_error < -180.0f) yaw_error += 360.0f;

            // PID纠偏：当偏航角非0时，计算补救的旋转速度 vw
            // 目标值为0(因为误差已经计算出来了), 限制纠偏最大速度为15
            yaw_correction = Position_PID_control(&yaw_pid, -yaw_error, 0, 15);
        }
        // --- 航向锁定逻辑结束 ---

        // 麦轮逆解：叠加手动指令(vw_set)与自动纠偏指令(yaw_correction)
        mecanum_calc(vx_set, vy_set, vw_set + yaw_correction, &w1, &w2, &w3, &w4);

        // 更新目标脉冲点
        Total_angle_1 += w1;
        Total_angle_2 += w2;
        Total_angle_3 += w3;
        Total_angle_4 += w4;

        // 电机位置环 PID 控制
        targetSpeed_1 = Position_PID_control(&wheel1_pid, (float)encoder_pulses_1, Total_angle_1, 15);
        targetSpeed_2 = Position_PID_control(&wheel2_pid, (float)encoder_pulses_2, Total_angle_2, 15);
        targetSpeed_3 = Position_PID_control(&wheel3_pid, (float)encoder_pulses_3, Total_angle_3, 15);
        targetSpeed_4 = Position_PID_control(&wheel4_pid, (float)encoder_pulses_4, Total_angle_4, 15);

        // 输出到硬件驱动
        car_speed_control();
    }
}

/**
  * @brief  串口解析任务 (10ms)
  */
void StartUart2Task(void const * argument)
{
    HAL_UART_Receive_IT(&huart2, rxBuffer1, 1);
    for(;;)
    {
        My_cmd_analysis();  //  串口接收任务
        vTaskDelay(10);
    }
}

/**
  * @brief  OLED刷新任务 (100ms)
  */
void StartOledTask(void const * argument)
{
    for(;;)
    {
        uint16_t key_num = 0;
        switch(key_num)
        {
        case 1:  WOUOUI_MSG_QUE_SEND(msg_down);break;
        case 2:  WOUOUI_MSG_QUE_SEND(msg_left);break;
        case 3:  WOUOUI_MSG_QUE_SEND(msg_click);break;
        case 4:  WOUOUI_MSG_QUE_SEND(msg_right);break;
        case 5:  WOUOUI_MSG_QUE_SEND(msg_up);break;
        case 33: WOUOUI_MSG_QUE_SEND(msg_return);break;
        default:                                  break;
        }
        WouoUI_Proc(25);
        WouoUI_WavePageUpdateVal(&wave_page, 0, 0);
        WouoUI_WavePageUpdateVal(&wave_page, 1, 1);
        vTaskDelay(1);
    }
}

/* 闲置任务内存分配 -----------------------------------------------------------*/
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}


//////////////////////////////////////////////       一些自定义函数        //////////////////////////////////////

/**
  * @brief  深度复位：清除位置累加、PID状态及目标航向
*/
void Reset_All_Coordinates(void)
{
    car_ready = 0;

    // 编码器累积脉冲清零
    encoder_pulses_1 = 0; encoder_pulses_2 = 0;
    encoder_pulses_3 = 0; encoder_pulses_4 = 0;

    // 逻辑位置目标清零
    Total_angle_1 = 0; Total_angle_2 = 0;
    Total_angle_3 = 0; Total_angle_4 = 0;

    // PID状态重置
    wheel1_pid.integral = 0; wheel1_pid.last_error = 0;
    wheel2_pid.integral = 0; wheel2_pid.last_error = 0;
    wheel3_pid.integral = 0; wheel3_pid.last_error = 0;
    wheel4_pid.integral = 0; wheel4_pid.last_error = 0;

    yaw_pid.integral = 0; yaw_pid.last_error = 0;

    // 锁定当前角度为初始目标航向
    target_yaw = my_car_angle.Yaw;

    vTaskDelay(10);
    car_ready = 1;
}

/**
  * @brief  麦轮运动学逆解计算
  */
void mecanum_calc(float vx, float vy, float vw, float *w1, float *w2, float *w3, float *w4)
{
    *w1 = vx - vy - vw;
    *w2 = vx + vy + vw;
    *w3 = vx - vy + vw;
    *w4 = vx + vy - vw;
}

//  走直线函数
void Move_Distance_Precision(float distance) {
    // 你的距离控制逻辑
}

/**
 * @brief 原地旋转指定的角度
 * @param degree: 要旋转的角度（正数左转，负数右转）
 */
void Rotate_Degree_Precision(float degree) {
    // 1. 告诉系统我们要开始精准控制了
    // 确保 vw_set 为 0，这样 pid 任务才会进入“航向锁定”模式
    vw_set = 0;

    // 2. 直接修改目标航向角
    target_yaw += degree;

    // 3. 处理角度回环 (保证在 -180 到 180 之间)
    if (target_yaw > 180.0f)  target_yaw -= 360.0f;
    if (target_yaw < -180.0f) target_yaw += 360.0f;

    // 提示：调用这个函数后，StartPidTask 里的 yaw_pid 会自动接管。
    // 它会发现当前角度和新的 target_yaw 有误差，从而驱动轮子旋转。
}



/* USER CODE END Application */