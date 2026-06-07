#ifndef PID_H
#define PID_H

#include <stdint.h>

// --- 增量式 PID (用于速度环：轮子转速控制) ---
typedef struct {
    float p;
    float i;
    float d;
    float err;
    float last_err;
    float next_err;
    float pwm;
    float add;
} PIDController;

// --- 位置式 PID (用于位置环和角度环：直线保持和距离控制) ---
typedef struct {
    float Kp;        // 比例增益
    float Ki;        // 积分增益
    float Kd;        // 微分增益
    float integral;  // 积分项
    float last_error; // 上一次误差
} PID_Position;

// 外部编码器声明
extern int16_t encoderValue1;
extern int16_t encoderValue2;
extern int16_t encoderValue3;
extern int16_t encoderValue4;

// 函数声明
void encoder_real(void);
int16_t myabs(int a);
float pid_update(PIDController* pid, int16_t speed, int16_t target);
int Position_PID_control(PID_Position *pid, int reality, int target, int speed);

#endif // PID_H