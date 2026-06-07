#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

int Read_Encoder_1(void);
int Read_Encoder_2(void);
int Read_Encoder_3(void);
int Read_Encoder_4(void);

int encoder_value_to_angle(int encoder_value);

#endif
