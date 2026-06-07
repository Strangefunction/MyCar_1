#include "pid.h"
#include "encoder.h"
int16_t encoderValue1 = 0;
int16_t encoderValue2 = 0;
int16_t encoderValue3 = 0;
int16_t encoderValue4 = 0;
// ======================================================
// 1. 速度环 PID 实例 (增量式)
// ======================================================
PIDController pid_1 = {65, 0.5, 0, 0, 0, 0, 0, 0}; // 左前
PIDController pid_2 = {65, 0.5, 0, 0, 0, 0, 0, 0}; // 右前
PIDController pid_3 = {65, 0.5, 0, 0, 0, 0, 0, 0}; // 右后
PIDController pid_4 = {65, 0.5, 0, 0, 0, 0, 0, 0}; // 左后

// ======================================================
// 2. 位置环/角度环 PID 实例 (位置式)
// ======================================================
// 用于纠偏和精准定位
PID_Position wheel1_pid = {0.15f, 0.01f, 0.01f, 0, 0};
PID_Position wheel2_pid = {0.15f, 0.01f, 0.01f, 0, 0};
PID_Position wheel3_pid = {0.15f, 0.01f, 0.01f, 0, 0};
PID_Position wheel4_pid = {0.15f, 0.01f, 0.01f, 0, 0};

// ======================================================
// 3. 功能函数实现
// ======================================================

/**
 * @brief 编码器数据处理，防止溢出跳变
 */
void encoder_real() {
    if (encoderValue1 > 1000)  encoderValue1 -= 2000;
    if (encoderValue2 > 1000)  encoderValue2 -= 2000;
    if (encoderValue3 > 1000)  encoderValue3 -= 2000;
    if (encoderValue4 > 1000)  encoderValue4 -= 2000;
}

int16_t myabs(int a) {
    return a < 0 ? -a : a;
}

/**
 * @brief 速度环 - 增量式 PID 更新
 */
float pid_update(PIDController* pid, int16_t speed, int16_t target) {
    pid->err = target - speed;
    pid->add = pid->p * (pid->err - pid->last_err) +
               pid->i * pid->err +
               pid->d * (pid->err + pid->next_err - 2 * pid->last_err);

    pid->pwm += pid->add;
    pid->next_err = pid->last_err;
    pid->last_err = pid->err;
    return pid->pwm;
}

/**
 * @brief 位置环/角度环 - 位置式 PID 控制
 * @param reality 当前测量值 (角度或脉冲)
 * @param target  目标值
 * @param speed   输出限幅 (最大允许速度)
 */
int Position_PID_control(PID_Position *pid, int reality, int target, int speed) {
    // 误差计算
    float error = (float)target - (float)reality;

    // 积分累加
    pid->integral += error;

    // 积分限幅 (防过充，针对角度系统优化的阈值)
    if (pid->integral > 2000) pid->integral = 2000;
    if (pid->integral < -2000) pid->integral = -2000;

    // 微分项 (计算变化率)
    float derivative = error - pid->last_error;

    // PID 公式计算输出
    float output = pid->Kp * error +
                   pid->Ki * pid->integral +
                   pid->Kd * derivative;

    // 更新历史误差
    pid->last_error = error;

    // 输出限幅
    if (output > (float)speed)  output = (float)speed;
    if (output < (float)-speed) output = (float)-speed;

    return (int)output;
}