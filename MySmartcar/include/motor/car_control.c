#include "main.h"
#include "tim.h"
#include "car_control.h"
#include "pid.h"
#include "stdlib.h"

//car and encoder and sensor
int16_t pwm1,pwm2,pwm3,pwm4;
extern int16_t encoderValue1,encoderValue2,encoderValue3,encoderValue4;
int targetSpeed_1,targetSpeed_2,targetSpeed_3,targetSpeed_4;
extern double result;
extern PIDController pid_1,pid_2,pid_3,pid_4;
extern int zero_nums;
extern CircularBuffer buffer; //循环数组
//mpu6050
extern float pitch, roll, yaw;					//  获取到的欧拉角
extern float pitch_temp, roll_temp, yaw_temp;   //  用于临时存储欧拉角
//xbox
extern int btnY, btnX, btnB, btnA, btnLB, btnRB, btnSelect, btnStart, btnXbox, btnShare, btnLS, btnRS;
extern int btnDirUp, btnDirRight, btnDirDown, btnDirLeft;
extern int joyLHori, joyLVert, joyRHori, joyRVert;
extern int trigLT, trigRT;
//  指令控制模式
CarControlMode_t current_control_mode = CAR_MODE_REMOTE;

//  电机初始化，主要就是定时器初始化
void car_motor_init(void)
{
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
}


int16_t pwm_control(int16_t t)
{
    if(t > 999)
        t = 999;
    if(t < 0)
        t = 0;
		return t;
}

void car_speed_control()
{
    pwm1 = pid_update(&pid_1,encoderValue1, targetSpeed_1);
    pwm2 = pid_update(&pid_2,encoderValue2, targetSpeed_2);
    pwm3 = pid_update(&pid_3,encoderValue3, targetSpeed_3);
    pwm4 = pid_update(&pid_4,encoderValue4, targetSpeed_4);

    if (pwm1 < 0) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_control(abs(pwm1))+180);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_control(abs(pwm1))+180);
    }

    if (pwm2 < 0) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_control(abs(pwm2))+180);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_control(abs(pwm2))+180);
    }

    if (pwm4 < 0) {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_control(abs(pwm4))+120);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_control(abs(pwm4))+120);
    }

    if (pwm3 < 0) {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_control(abs(pwm3))+120);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_control(abs(pwm3))+120);
    }
}

//控制车运动
void carcontrol(int l,int r,int x,int y)
{
		targetSpeed_1 = l;
		targetSpeed_2 = r;
		targetSpeed_3 = x;
		targetSpeed_4 = y;
}

//////////////////////////////////////////////////////     Xbox    ///////////////////////////////////////////////////////////////////
unsigned char SpeedDifferent = 0; //用来计算两个轮子的差速

// 定义控制指令的结构体
typedef struct {
	int first;
	int second;
	int third;
	int fourth;
} ControlCommand;

// 初始化控制指令的查找表
ControlCommand global_controlCommands[9];

// 初始化标志
static int initialized = 0;

void initControlCommands() {
	// 检查是否已经初始化过
	if (initialized) {
		return; // 如果已经初始化过，则直接返回
	}

	// 初始化局部的 controlCommands 数组
	ControlCommand controlCommands[9] = {
		{0,0,0,0},  // 0
		{5,5,5,5},  // 1前
		{5,0,5,0},  // 2右上
		{5,-5,5,-5},  // 3右
		{0,-5,0,-5},  // 4右下
		{-5,-5,-5,-5},  // 5下
		{-5,0,-5,0},  // 6左下
		{-5,5,-5,5},  // 7左
		{0,5,0,5}  // 默认（无操作）
	};
	// 将 controlCommands 的内容复制到全局变量 global_controlCommands
	memcpy(global_controlCommands, controlCommands, sizeof(controlCommands));

	// 标记为已初始化
	initialized = 1;
}

// 计算摇杆位置对应的索引
int getControlIndex(int joyLVert, int joyLHori) {
	if (joyLVert < 30000 && joyLVert >= 0) {
		if (joyLHori >= 0 && joyLHori < 30000) return 8;
		if (joyLHori >= 30000 && joyLHori <= 35000) return 1;
		if (joyLHori > 35000 && joyLHori <= 65535) return 2;
	} else if (joyLVert >= 30000 && joyLVert <= 35000) {
		if (joyLHori >= 0 && joyLHori <= 30000) return 7;
		if (joyLHori >= 30000 && joyLHori <= 35000) return 0;
		if (joyLHori >= 35000 && joyLHori <= 65535) return 3;
	} else if (joyLVert > 35000 && joyLVert <= 65535) {
		if (joyLHori >= 0 && joyLHori < 30000) return 6;
		if (joyLHori >= 30000 && joyLHori <= 35000) return 5;
		if (joyLHori > 35000 && joyLHori <= 65535) return 4;
	}
	return 0;  // 默认（无操作）
}
// 使用查找表执行控制命令
void executeControlCommand(int joyLVert, int joyLHori) {
	//先初始化一次命令集,如果初始化了就不执行
	if(!initialized) {
		initControlCommands();
	}
	int index = getControlIndex(joyLVert, joyLHori);

	carcontrol(
	(global_controlCommands[index].first == 0 ) ? 0 : (global_controlCommands[index].first > 0) ? global_controlCommands[index].first + SpeedDifferent : global_controlCommands[index].first - SpeedDifferent,
	(global_controlCommands[index].second == 0 ) ? 0 : (global_controlCommands[index].second > 0) ? global_controlCommands[index].second + SpeedDifferent : global_controlCommands[index].second - SpeedDifferent,
	(global_controlCommands[index].third == 0 ) ? 0 : (global_controlCommands[index].third > 0) ? global_controlCommands[index].third + SpeedDifferent : global_controlCommands[index].third - SpeedDifferent,
	(global_controlCommands[index].fourth == 0 ) ? 0 : (global_controlCommands[index].fourth > 0) ? global_controlCommands[index].fourth + SpeedDifferent : global_controlCommands[index].fourth - SpeedDifferent
);
}
/*
 * function: 用来映射 左右扳机  左侧扳机加速  右侧扳机减速
 * Xbox_SmartCarSpeedControl(trigLT,trigRT)
 */
int Xbox_SmartCarSpeedControl(int trigLT, int trigRT) {
   int SpeedDifferent;
   if(trigLT != 0 || trigRT != 0){
        SpeedDifferent = (trigLT >= trigRT ? (trigLT - trigRT) : (~(int)(trigRT - trigLT) + 1));
   }else SpeedDifferent = 0;

   SpeedDifferent = (SpeedDifferent * 50) / 1023;

   return SpeedDifferent;
}

/*
 * function: 主要控制算法
***左侧扳机0到1023加速  右侧扳机0到1023减速
 */
void Xbox_XboxConnectMySmartCar() {

    //将两个轮子的差速算出来;  或者加速
    SpeedDifferent = Xbox_SmartCarSpeedControl(trigLT,trigRT);
	//用摇杆控制小车的平移
	executeControlCommand(joyLVert, joyLHori);
    //YAXB Xbox 微调
	if(getControlIndex(joyLVert, joyLHori) == 0) {
		while(1) {
			SpeedDifferent = Xbox_SmartCarSpeedControl(trigLT,trigRT);
			if(btnY == 1){
				carcontrol(5+SpeedDifferent,5+SpeedDifferent,5+SpeedDifferent,5+SpeedDifferent);
			}else if(btnA == 1){
				carcontrol(-(5+SpeedDifferent),-(5+SpeedDifferent),-(5+SpeedDifferent),-(5+SpeedDifferent));
			}else if(btnX == 1){
				carcontrol(-(5+SpeedDifferent),5+SpeedDifferent,5+SpeedDifferent,-(5+SpeedDifferent));
			}else if(btnB == 1){
				carcontrol(5+SpeedDifferent,-(5+SpeedDifferent),-(5+SpeedDifferent),5+SpeedDifferent);
			}else if(btnXbox == 1) {
				carcontrol(0,0,0,0);
			}else break;
		}
	}

}
///////////////////////////////////////            基本控制八个方位运动                   /////////////////////////////////////////////////////////////////////////

void My_SmartCar_Fundamental_motion(uint8_t DIR, uint8_t Left_front_V, uint8_t Right_front_V, uint8_t Right_rear_V, uint8_t Left_rear_V) {
	switch(DIR) {
		case 1 : carcontrol(Left_front_V,Right_front_V,Right_rear_V,Left_rear_V);	break;  //前
		case 2 : carcontrol(-Left_front_V,-Right_front_V,-Right_rear_V,-Left_rear_V);	break;  //后
		case 3 : carcontrol(-Left_front_V,Right_front_V,-Right_rear_V,Left_rear_V);	break;  //左
		case 4 : carcontrol(Left_front_V,-Right_front_V,Right_rear_V,-Left_rear_V);	break;  //右
		case 5 : carcontrol(0,Right_front_V,0,Left_rear_V);	break;  //左上
		case 6 : carcontrol(Left_front_V,0,Right_rear_V,0);	break;  //右上
		case 7 : carcontrol(-Left_front_V,0,-Right_rear_V,0);	break;  //左下
		case 8 : carcontrol(0,-Right_front_V,0,-Left_rear_V);	break;  //右下
		case 9 : carcontrol(-Left_front_V,Right_front_V,Right_rear_V,-Left_rear_V);	break;  //左 转向
		case 10 : carcontrol(Left_front_V,-Right_front_V,-Right_rear_V,Left_rear_V);	break;  //右 转向
		default: carcontrol(0,0,0,0);  //停止
	}
}
