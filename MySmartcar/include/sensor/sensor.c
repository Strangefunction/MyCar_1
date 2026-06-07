
//  这个是八路灰度读取的程序



#include "sensor.h"
#include "main.h"
#include <stdbool.h>

#define GW_GRAY_ADDR GW_GRAY_ADDR_DEF
#define GW_GRAY_SERIAL_GPIO_GROUP GPIOD
#define GW_GRAY_SERIAL_GPIO_CLK GPIO_PIN_9  //PB8 ??CLK
#define GW_GRAY_SERIAL_GPIO_DAT GPIO_PIN_8 //PB9 ??DAT
#define GW_GRAY_SERIAL_DELAY_VALUE 270

uint8_t sensor_data;//传感器变量
uint8_t sensor[8];
int zero_nums = 0;
char binary[9];
double result;

void initBuffer(CircularBuffer* buffer) {
    buffer->head = 0;
    buffer->tail = 0;
    buffer->full = 0;
}

void insertData(CircularBuffer* buffer, float value) {
    buffer->data[buffer->tail] = value;
    buffer->tail = (buffer->tail + 1) % BUFFER_SIZE;

    if (buffer->full) {
        buffer->head = (buffer->head + 1) % BUFFER_SIZE;
    }

    if (buffer->tail == buffer->head) {
        buffer->full = 1;
    } else {
        buffer->full = 0;
    }
}

float getLastData(CircularBuffer* buffer) {
    if (buffer->tail == 0) {
        return buffer->data[BUFFER_SIZE - 1];
    } else {
        return buffer->data[buffer->tail - 1];
    }
}
//计算灰度返回倿
void decimalToBinary(int decimal, char* binary) {

	 int i;

	 for (i = 7; i >= 0; i--) {

	     binary[i] = (decimal % 2) + '0';

	     decimal /= 2;

	 }

	    // ??????????8?,????0

	    for (; i > 0; i--) {

	        binary[i - 1] = '0';

	    }

	    binary[8] = '\0'; // ??????

	}

double sumAndDivideZeros(char* binary) {
    double sum = 0.0,count = 0.0;

    for (int i = 0; i < 8; i++) {
        if (binary[i] == '0') {
            sum += (double)(i + 1); // 计算0的位置的咿
            count++;
        }
    }
    if (count == 8) {
        return 0.0;
    }
    if (count == 0) {
        return 255; // 如果没有0，返囿255
    }

    return sum / count; // 返回平均倿
}

int countZeros(uint8_t sensor[], int size) {
    int counts = 0;
    for (int i = 0; i < size; i++) {
        if (sensor[i] == 0) {
            counts++;
        }
    }
    return counts;
}

int areZerosConsecutive(const char binary[8]) {
    int firstZeroFound = 0;
    int lastZeroPosition = -1;

    for (int i = 0; i < 8; i++) {
        if (binary[i] == '0') {
            if (firstZeroFound) {
                if (i != lastZeroPosition + 1) {
                    return 0;
                }
            } else {
                firstZeroFound = 1;
            }
            lastZeroPosition = i;
        }
    }
    return 1;
}

//灰度函数
void delay(uint32_t delay_count)
{
    for (int i = 0; i < delay_count; ++i) {
        __NOP();
    }
}

uint8_t gw_gray_serial_read()
{
    uint8_t ret = 0;

    for (int i = 0; i < 8; ++i) {
        /* ????????? */
        HAL_GPIO_WritePin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, GPIO_PIN_RESET);
        delay(GW_GRAY_SERIAL_DELAY_VALUE); // ??????(??10k??) ???????

        ret |= HAL_GPIO_ReadPin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_DAT) << i;

        /* ?????????,????????*/
        HAL_GPIO_WritePin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, GPIO_PIN_SET);

        /* ??????????????????,???????5us?? */
        delay(320);
    }

    return ret;
}