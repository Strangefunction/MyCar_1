#ifndef CARCONTROL_H
#define CARCONTROL_H

// --- 模式定义 ---
typedef enum {
    CAR_MODE_REMOTE = 0,    // 模式0：速度矢量模式 (使用vx, vy, vw)
    CAR_MODE_PRECISION = 1  // 模式1：精准路径模式 (使用Move_Distance / Rotate_Degree)
} CarControlMode_t;

void car_motor_init(void);
void carcontrol(int l,int r,int x,int y);
int16_t pwm_control(int16_t t);
void car_speed_control(void);
void controlCarDirection(float value);
void car_go_low(int time);
void car_back_low(int time);
void car_turn(void);  //掉头
void car_turn_left(void);
void car_turn_right(void);
void xunji(void);
int Xbox_SmartCarSpeedControl(int trigLT, int trigRT);
void Xbox_XboxConnectMySmartCar(void);
void My_SmartCar_Fundamental_motion(uint8_t DIR, uint8_t Left_front_V, uint8_t Right_front_V, uint8_t Right_rear_V, uint8_t Left_rear_V);
void executeControlCommand(int joyLVert, int joyLHori);

#endif
