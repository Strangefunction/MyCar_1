#include "encoder.h"

// 定义四个编码器脉冲计数变量
long long int encoder_pulses_1 = 0;
long long int encoder_pulses_2 = 0;
long long int encoder_pulses_3 = 0;
long long int encoder_pulses_4 = 0;

int Read_Encoder_1(){
	int Encoder_TIM = 0;
	Encoder_TIM = TIM3->CNT;
	//if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff;
	TIM3->CNT = 0;
	return Encoder_TIM;
}

int Read_Encoder_2(){
	int Encoder_TIM = 0;
	Encoder_TIM = TIM4->CNT;
	//if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff;
	TIM4->CNT = 0;
	return Encoder_TIM;
}

int Read_Encoder_3(){
	int Encoder_TIM = 0;
	Encoder_TIM = TIM5->CNT;
	//if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff;
	TIM5->CNT = 0;
	return Encoder_TIM;
}

int Read_Encoder_4(){
	int Encoder_TIM = 0;
	Encoder_TIM = TIM8->CNT;
	//if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff;
	TIM8->CNT = 0;
	return Encoder_TIM;
}

int encoder_value_to_angle(int encoder_value) {
    // 计算角度，避免整数除法导致精度丢失
    float angle = ((float)encoder_value / 1560) * 360;

    // 将浮点数角度四舍五入为整数，避免直接截断导致的精度问题
    return (int)(angle + 0.5);
}


void split_angle(int angle) {
    // 计算完整的360度的倍数
    int full_rotations = angle / 360;
    // 计算剩余的角度
    int remaining_angle = angle % 360;

    // 输出完整的360度的倍数
    for (int i = 0; i < full_rotations; i++) {
        //printf("360\n");
    }

    // 如果有剩余角度，输出剩余角度
    if (remaining_angle > 0) {
        //printf("%d\n", remaining_angle);
    }
}




