#ifndef __MPU9250_H
#define __MPU9250_H

#include "main.h"

// --- 卡尔曼滤波结构体 ---
typedef struct {
    float Q_angle;   // 过程噪声协方差：角度
    float Q_gyro;    // 过程噪声协方差：陀螺仪漂移
    float R_angle;   // 测量噪声协方差：加速度计
    float angle;     // 滤波输出的角度
    float bias;      // 陀螺仪零偏估计
    float P[2][2];   // 误差协方差矩阵
} Kalman_t;

// --- 姿态角结构体 ---
typedef struct {
    float Pitch;
    float Roll;
    float Yaw;
} Attitude_t;

// --- 函数声明 ---
void MPU9250_Init(void);
void MPU9250_CalibrateGyro(void);
void MPU9250_ReadAccel(int16_t *x, int16_t *y, int16_t *z);
void MPU9250_ReadGyro(int16_t *x, int16_t *y, int16_t *z);
void MPU9250_ReadMag(int16_t *x, int16_t *y, int16_t *z);

// 姿态解算（卡尔曼滤波版）
void MPU9250_Get_Attitude(Attitude_t *angle, float dt);

#endif