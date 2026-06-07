#include "mpu9250.h"
#include <math.h>

#define DELAY_TIME 30

// ================== 软件I2C配置 PB8(SCL) PB9(SDA) ==================
#define MPU_SCL_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define MPU_SCL_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define MPU_SDA_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define MPU_SDA_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define MPU_SDA_READ HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)

#define MPU9250_ADDR    0x68
#define MAG_ADDR        0x0C

static int16_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// 初始化卡尔曼参数 (经验值，可根据实际震动情况微调)
static Kalman_t K_Pitch = {0.001f, 0.003f, 0.03f, 0, 0, {{0,0},{0,0}}};
static Kalman_t K_Roll  = {0.001f, 0.003f, 0.03f, 0, 0, {{0,0},{0,0}}};

// ================== 内部 I2C 底层函数 ==================
static void MPU_I2C_GPIO_Init(void) {
    GPIO_InitTypeDef gpio_conf = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_conf.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio_conf.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_conf.Pull = GPIO_PULLUP;
    gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_conf);
    MPU_SCL_H; MPU_SDA_H;
}

static void MPU_I2C_Delay(void) { uint32_t i = DELAY_TIME; while(i--); }

static void MPU_I2C_Start(void) {
    MPU_SDA_H; MPU_SCL_H; MPU_I2C_Delay();
    MPU_SDA_L; MPU_I2C_Delay(); MPU_SCL_L;
}

static void MPU_I2C_Stop(void) {
    MPU_SCL_L; MPU_SDA_L; MPU_I2C_Delay();
    MPU_SCL_H; MPU_I2C_Delay(); MPU_SDA_H; MPU_I2C_Delay();
}

static void MPU_I2C_Send_Byte(uint8_t byte) {
    for(uint8_t i=0; i<8; i++) {
        MPU_SCL_L; MPU_I2C_Delay();
        if(byte & 0x80) MPU_SDA_H; else MPU_SDA_L;
        byte <<= 1; MPU_I2C_Delay(); MPU_SCL_H; MPU_I2C_Delay();
    }
    MPU_SCL_L;
}

static uint8_t MPU_I2C_Read_Byte(uint8_t ack) {
    uint8_t dat = 0; MPU_SDA_H;
    for(uint8_t i=0; i<8; i++) {
        dat <<= 1; MPU_SCL_L; MPU_I2C_Delay(); MPU_SCL_H; MPU_I2C_Delay();
        if(MPU_SDA_READ) dat |= 0x01;
    }
    MPU_SCL_L;
    if(ack) MPU_SDA_L; else MPU_SDA_H;
    MPU_I2C_Delay(); MPU_SCL_H; MPU_I2C_Delay(); MPU_SCL_L;
    return dat;
}

static uint8_t MPU_I2C_WaitAck(void) {
    uint8_t cnt = 200; MPU_SDA_H; MPU_I2C_Delay(); MPU_SCL_H; MPU_I2C_Delay();
    while(MPU_SDA_READ && cnt--); MPU_SCL_L;
    return cnt ? 0 : 1;
}

static void MPU_Write(uint8_t dev, uint8_t reg, uint8_t data) {
    MPU_I2C_Start();
    MPU_I2C_Send_Byte(dev << 1); MPU_I2C_WaitAck();
    MPU_I2C_Send_Byte(reg);      MPU_I2C_WaitAck();
    MPU_I2C_Send_Byte(data);     MPU_I2C_WaitAck();
    MPU_I2C_Stop();
}

static void MPU_Read(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    MPU_I2C_Start();
    MPU_I2C_Send_Byte(dev << 1); MPU_I2C_WaitAck();
    MPU_I2C_Send_Byte(reg);      MPU_I2C_WaitAck();
    MPU_I2C_Start();
    MPU_I2C_Send_Byte((dev << 1) | 0x01); MPU_I2C_WaitAck();
    for(uint8_t i=0; i<len; i++) buf[i] = MPU_I2C_Read_Byte(i == len-1 ? 0 : 1);
    MPU_I2C_Stop();
}

// ================== 卡尔曼滤波核心算法 ==================
static float Kalman_Update(Kalman_t *K, float newAngle, float newGyro, float dt) {
    // 1. 预测
    K->angle += dt * (newGyro - K->bias);
    K->P[0][0] += dt * (dt * K->P[1][1] - K->P[0][1] - K->P[1][0] + K->Q_angle);
    K->P[0][1] -= dt * K->P[1][1];
    K->P[1][0] -= dt * K->P[1][1];
    K->P[1][1] += K->Q_gyro * dt;

    // 2. 更新 (量测)
    float S = K->P[0][0] + K->R_angle;
    float Kg[2] = { K->P[0][0] / S, K->P[1][0] / S };

    float y = newAngle - K->angle;
    K->angle += Kg[0] * y;
    K->bias  += Kg[1] * y;

    float P00_temp = K->P[0][0], P01_temp = K->P[0][1];
    K->P[0][0] -= Kg[0] * P00_temp;
    K->P[0][1] -= Kg[0] * P01_temp;
    K->P[1][0] -= Kg[1] * P00_temp;
    K->P[1][1] -= Kg[1] * P01_temp;

    return K->angle;
}

// ================== MPU9250 功能实现 ==================

void MPU9250_Init(void) {
    MPU_I2C_GPIO_Init();
    HAL_Delay(100);
    MPU_Write(MPU9250_ADDR, 0x6B, 0x80); HAL_Delay(100);
    MPU_Write(MPU9250_ADDR, 0x6B, 0x00); HAL_Delay(100);
    MPU_Write(MPU9250_ADDR, 0x19, 0x07);
    MPU_Write(MPU9250_ADDR, 0x1A, 0x06);
    MPU_Write(MPU9250_ADDR, 0x1B, 0x18); // +/-2000dps
    MPU_Write(MPU9250_ADDR, 0x1C, 0x01); // +/-2g
    MPU_Write(MPU9250_ADDR, 0x37, 0x02); // Bypass Mode
    HAL_Delay(10);
    MPU_Write(MAG_ADDR, 0x0A, 0x16);     // 16bit, 100Hz cont.
}

void MPU9250_CalibrateGyro(void) {
    int32_t sx = 0, sy = 0, sz = 0;
    uint8_t buf[6];
    // 增加采样次数到 1000 次，提高均值精度
    for(int i = 0; i < 1000; i++) {
        MPU_Read(MPU9250_ADDR, 0x43, buf, 6);
        sx += (int16_t)((buf[0] << 8) | buf[1]);
        sy += (int16_t)((buf[2] << 8) | buf[3]);
        sz += (int16_t)((buf[4] << 8) | buf[5]);
        HAL_Delay(1);
    }
    gyro_x_offset = sx / 1000;
    gyro_y_offset = sy / 1000;
    gyro_z_offset = sz / 1000;
}

void MPU9250_ReadAccel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    MPU_Read(MPU9250_ADDR, 0x3B, buf, 6);
    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *y = (int16_t)((buf[2] << 8) | buf[3]);
    *z = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU9250_ReadGyro(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    MPU_Read(MPU9250_ADDR, 0x43, buf, 6);

    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]) - gyro_x_offset;
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]) - gyro_y_offset;
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]) - gyro_z_offset;

    // 设置死区：如果读取到的原始值变化极小，直接视为 0
    // 16.4 对应 1度/秒，这里设置约 0.2度/秒的过滤阈值
    if(abs(raw_x) < 4) *x = 0; else *x = raw_x;
    if(abs(raw_y) < 4) *y = 0; else *y = raw_y;
    if(abs(raw_z) < 4) *z = 0; else *z = raw_z;
}

void MPU9250_ReadMag(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t status, buf[7];
    MPU_Read(MAG_ADDR, 0x02, &status, 1);
    if(status & 0x01) {
        MPU_Read(MAG_ADDR, 0x03, buf, 7);
        *x = (int16_t)((buf[1] << 8) | buf[0]);
        *y = (int16_t)((buf[3] << 8) | buf[2]);
        *z = (int16_t)((buf[5] << 8) | buf[4]);
    }
}

void MPU9250_Get_Attitude(Attitude_t *angle, float dt) {
    int16_t ax, ay, az, gx, gy, gz;
    MPU9250_ReadAccel(&ax, &ay, &az);
    MPU9250_ReadGyro(&gx, &gy, &gz);

    // 单位换算
    float Accel_X = ax / 16384.0f;
    float Accel_Y = ay / 16384.0f;
    float Accel_Z = az / 16384.0f;
    float Gyro_X = gx / 16.4f;
    float Gyro_Y = gy / 16.4f;
    float Gyro_Z = gz / 16.4f;

    // 计算加速度计观测角
    float acc_pitch = atan2(-Accel_X, sqrt(Accel_Y * Accel_Y + Accel_Z * Accel_Z)) * 57.3f;
    float acc_roll  = atan2(Accel_Y, Accel_Z) * 57.3f;

    // 卡尔曼滤波融合
    angle->Pitch = Kalman_Update(&K_Pitch, acc_pitch, Gyro_Y, dt);
    angle->Roll  = Kalman_Update(&K_Roll,  acc_roll,  Gyro_X, dt);

    // Yaw 轴通过陀螺仪积分 (磁力计补偿逻辑较为复杂，通常需配合电子罗盘算法)
    angle->Yaw += Gyro_Z * dt;
    if (angle->Yaw > 180.0f)  angle->Yaw -= 360.0f;
    if (angle->Yaw < -180.0f) angle->Yaw += 360.0f;
}