/*
 * Copyright (c) 2022 ??????(??)
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

#define BUFFER_SIZE 50
typedef struct {
 float data[BUFFER_SIZE];
 int head;
 int tail;
 int full;
} CircularBuffer;

void insertData(CircularBuffer* buffer, float value);
float getLastData(CircularBuffer* buffer);
void decimalToBinary(int decimal, char* binary);
double sumAndDivideZeros(char* binary);
int countZeros(uint8_t sensor[], int size);
int areZerosConsecutive(const char binary[8]);
void delay(uint32_t delay_count);
uint8_t gw_gray_serial_read();
void initBuffer(CircularBuffer* buffer);

/* ???? */
#define GW_GRAY_ADDR_DEF 0x4C
#define GW_GRAY_PING 0xAA
#define GW_GRAY_PING_OK 0x66
#define GW_GRAY_PING_RSP GW_GRAY_PING_OK

/* ???????? */
#define GW_GRAY_DIGITAL_MODE 0xDD

/* ???????????? */
#define GW_GRAY_ANALOG_BASE_ 0xB0
#define GW_GRAY_ANALOG_MODE  (GW_GRAY_ANALOG_BASE_ + 0)

/* ?????????(v3.6??????) */
#define GW_GRAY_ANALOG_NORMALIZE 0xCF

/* ???????????? n?1???8 */
#define GW_GRAY_ANALOG(n) (GW_GRAY_ANALOG_BASE_ + (n))

/* ?????????? */
#define GW_GRAY_CALIBRATION_BLACK 0xD0
/* ?????????? */
#define GW_GRAY_CALIBRATION_WHITE 0xD1

// ???????????(CE: channel enable)
#define GW_GRAY_ANALOG_CHANNEL_ENABLE 0xCE
#define GW_GRAY_ANALOG_CH_EN_1 (0x1 << 0)
#define GW_GRAY_ANALOG_CH_EN_2 (0x1 << 1)
#define GW_GRAY_ANALOG_CH_EN_3 (0x1 << 2)
#define GW_GRAY_ANALOG_CH_EN_4 (0x1 << 3)
#define GW_GRAY_ANALOG_CH_EN_5 (0x1 << 4)
#define GW_GRAY_ANALOG_CH_EN_6 (0x1 << 5)
#define GW_GRAY_ANALOG_CH_EN_7 (0x1 << 6)
#define GW_GRAY_ANALOG_CH_EN_8 (0x1 << 7)
#define GW_GRAY_ANALOG_CH_EN_ALL (0xFF)


/* ?????? */
#define GW_GRAY_ERROR 0xDE

/* ?????? */
#define GW_GRAY_REBOOT 0xC0

/* ??????? */
#define GW_GRAY_FIRMWARE 0xC1


/**
 * @brief ?I2C???8????????? ???n????
 * @param sensor_value_8 ??IO???
 * @param n ?1??1??, n=1 ????????????, n=8?????
 * @return
 */
#define GET_NTH_BIT(sensor_value, nth_bit) (((sensor_value) >> ((nth_bit)-1)) & 0x01)


/**
 * @brief ???????????bit
 */
#define SEP_ALL_BIT8(sensor_value, val1, val2, val3, val4, val5, val6, val7, val8) \
do {                                                                              \
val1 = GET_NTH_BIT(sensor_value, 1);                                              \
val2 = GET_NTH_BIT(sensor_value, 2);                                              \
val3 = GET_NTH_BIT(sensor_value, 3);                                              \
val4 = GET_NTH_BIT(sensor_value, 4);                                              \
val5 = GET_NTH_BIT(sensor_value, 5);                                              \
val6 = GET_NTH_BIT(sensor_value, 6);                                              \
val7 = GET_NTH_BIT(sensor_value, 7);                                              \
val8 = GET_NTH_BIT(sensor_value, 8);                                              \
} while(0)

/* ????I2C?? */
#define GW_GRAY_CHANGE_ADDR 0xAD

/* ????????????? */
#define GW_GRAY_BROADCAST_RESET "\xB8\xD0\xCE\xAA\xBF\xC6\xBC\xBC"

#if defined (ESP_PLATFORM)
/* ESP32 */


#endif

#endif
