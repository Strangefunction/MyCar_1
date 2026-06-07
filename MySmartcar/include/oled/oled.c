#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "main.h"

uint8_t OLED_GRAM[144][8];

// ���Ժ���
void OLED_ColorTurn(uint8_t i)
{
    if (i == 0)
    {
        OLED_WR_Byte(0xA6, OLED_CMD); // ������ʾ
    }
    if (i == 1)
    {
        OLED_WR_Byte(0xA7, OLED_CMD); // ��ɫ��ʾ
    }
}

// ��Ļ��ת180��
void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0)
    {
        OLED_WR_Byte(0xC8, OLED_CMD); // ������ʾ
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1)
    {
        OLED_WR_Byte(0xC0, OLED_CMD); // ��ת��ʾ
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void IIC_delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 3; i++); // �򵥵�æ�ȴ���ʱ
}

// ��ʼ�ź�
void I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Clr();
    IIC_delay();
    OLED_SCL_Clr();
    IIC_delay();
}

// �����ź�
void I2C_Stop(void)
{
    OLED_SDA_Clr();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Set();
}

// �ȴ��ź���Ӧ
void I2C_WaitAck(void) // �������źŵĵ�ƽ
{
    OLED_SDA_Set();
    IIC_delay();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SCL_Clr();
    IIC_delay();
}

void Send_Byte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80) // ��dat��8λ�����λ����д��
        {
            OLED_SDA_Set();
        }
        else
        {
            OLED_SDA_Clr();
        }
        OLED_SCL_Set();
        dat <<= 1;
        OLED_SCL_Clr();
    }
}

// ����һ���ֽ�
// mode: ����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    Send_Byte(0x78); // OLED �� I2C ��ַ
    I2C_WaitAck();
    if (mode)
    {
        Send_Byte(0x40); // ����ģʽ
    }
    else
    {
        Send_Byte(0x00); // ����ģʽ
    }
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

// ���� OLED ��ʾ
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); // ��ɱ�ʹ��
    OLED_WR_Byte(0x14, OLED_CMD); // ������ɱ�
    OLED_WR_Byte(0xAF, OLED_CMD); // ������Ļ
}

// �ر� OLED ��ʾ
void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); // ��ɱ�ʹ��
    OLED_WR_Byte(0x10, OLED_CMD); // �رյ�ɱ�
    OLED_WR_Byte(0xAE, OLED_CMD); // �ر���Ļ
}

// �����Դ浽 OLED
void OLED_Refresh(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // ��������ʼ��ַ
        OLED_WR_Byte(0x00, OLED_CMD);     // ���õ�����ʼ��ַ
        OLED_WR_Byte(0x10, OLED_CMD);     // ���ø�����ʼ��ַ
        I2C_Start();
        Send_Byte(0x78);
        I2C_WaitAck();
        Send_Byte(0x40);
        I2C_WaitAck();
        for (n = 0; n < 128; n++)
        {
            Send_Byte(OLED_GRAM[n][i]);
            I2C_WaitAck();
        }
        I2C_Stop();
    }
}

// ��������
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            OLED_GRAM[n][i] = 0; // �����������
        }
    }
    OLED_Refresh(); // ������ʾ
}

// ����
// x:0~127
// y:0~63
// t:1 ��� 0,���
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t)
    {
        OLED_GRAM[x][i] |= n;
    }
    else
    {
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
        OLED_GRAM[x][i] |= n;
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    }
}

// ����
// x1,y1:�������
// x2,y2:��������
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; // ������������
    delta_y = y2 - y1;
    uRow = x1;         // �����������
    uCol = y1;
    if (delta_x > 0)
        incx = 1; // ���õ�������
    else if (delta_x == 0)
        incx = 0; // ��ֱ��
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // ˮƽ��
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; // ѡȡ��������������
    else
        distance = delta_y;
    for (t = 0; t < distance + 1; t++)
    {
        OLED_DrawPoint(uRow, uCol, mode); // ����
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

// x,y:Բ������
// r:Բ�İ뾶
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y + b, 1);

        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        OLED_DrawPoint(x - b, y + a, 1);

        a++;
        num = (a * a + b * b) - r * r; // ���㻭�ĵ���Բ�ĵľ���
        if (num > 0)
        {
            b--;
            a--;
        }
    }
}

// ��ָ��λ����ʾһ���ַ������������ַ�
// x:0~127
// y:0~63
// size1:ѡ������ 6x8/6x12/8x16/12x24
// mode:0,��ɫ��ʾ;1,������ʾ
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    if (size1 == 8)
        size2 = 6;
    else
        size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2); // �õ�����һ���ַ���Ӧ������ռ���ֽ���
    chr1 = chr - ' '; // ����ƫ�ƺ��ֵ
    for (i = 0; i < size2; i++)
    {
        if (size1 == 8)
        {
            temp = asc2_0806[chr1][i]; // ���� 0806 ����
        }
        else if (size1 == 12)
        {
            temp = asc2_1206[chr1][i]; // ���� 1206 ����
        }
        else if (size1 == 16)
        {
            temp = asc2_1608[chr1][i]; // ���� 1608 ����
        }
        else if (size1 == 24)
        {
            temp = asc2_2412[chr1][i]; // ���� 2412 ����
        }
        else
            return;
        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((size1 != 8) && ((x - x0) == size1 / 2))
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

// ��ʾ�ַ���
// x,y:�������
// size1:�����С
// *chr:�ַ�����ʼ��ַ
// mode:0,��ɫ��ʾ;1,������ʾ
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
    while ((*chr >= ' ') && (*chr <= '~')) // �ж��ǲ��ǷǷ��ַ�!
    {
        OLED_ShowChar(x, y, *chr, size1, mode);
        if (size1 == 8)
            x += 6;
        else
            x += size1 / 2;
        chr++;
    }
}

// m^n
uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
    {
        result *= m;
    }
    return result;
}

// ��ʾ����
// x,y :�������
// num :Ҫ��ʾ������
// len :���ֵ�λ��
// size:�����С
// mode:0,��ɫ��ʾ;1,������ʾ
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t t, temp, m = 0;
    if (size1 == 8)
        m = 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (temp == 0)
        {
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
        }
        else
        {
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
        }
    }
}

// ��ʾ����
// x,y:�������
// num:���ֶ�Ӧ�����
// mode:0,��ɫ��ʾ;1,������ʾ
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
    uint8_t m, temp;
    uint8_t x0 = x, y0 = y;
    uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1; // �õ�����һ���ַ���Ӧ������ռ���ֽ���
    for (i = 0; i < size3; i++)
    {
        if (size1 == 16)
        {
            temp = Hzk1[num][i]; // ���� 16*16 ����
        }
        else if (size1 == 24)
        {
            temp = Hzk2[num][i]; // ���� 24*24 ����
        }
        else if (size1 == 32)
        {
            temp = Hzk3[num][i]; // ���� 32*32 ����
        }
        else if (size1 == 64)
        {
            temp = Hzk4[num][i]; // ���� 64*64 ����
        }
        else
            return;
        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((x - x0) == size1)
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

// num ��ʾ���ֵĸ���
// space ÿһ����ʾ�ļ��
// mode:0,��ɫ��ʾ;1,������ʾ
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
    uint8_t i, n, t = 0, m = 0, r;
    while (1)
    {
        if (m == 0)
        {
            OLED_ShowChinese(128, 24, t, 16, mode); // д��һ�����ֱ����� OLED_GRAM[][] ������
            t++;
        }
        if (t == num)
        {
            for (r = 0; r < 16 * space; r++) // ��ʾ���
            {
                for (i = 1; i < 144; i++)
                {
                    for (n = 0; n < 8; n++)
                    {
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                    }
                }
                OLED_Refresh();
            }
            t = 0;
        }
        m++;
        if (m == 16)
        {
            m = 0;
        }
        for (i = 1; i < 144; i++) // ʵ������
        {
            for (n = 0; n < 8; n++)
            {
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
            }
        }
        OLED_Refresh();
    }
}

// x,y���������
// sizex,sizey,ͼƬ����
// BMP[]��Ҫд���ͼƬ����
// mode:0,��ɫ��ʾ;1,������ʾ
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
    uint16_t j = 0;
    uint8_t i, n, temp, m;
    uint8_t x0 = x, y0 = y;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (n = 0; n < sizey; n++)
    {
        for (i = 0; i < sizex; i++)
        {
            temp = BMP[j];
            j++;
            for (m = 0; m < 8; m++)
            {
                if (temp & 0x01)
                    OLED_DrawPoint(x, y, mode);
                else
                    OLED_DrawPoint(x, y, !mode);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == sizex)
            {
                x = x0;
                y0 = y0 + 8;
            }
            y = y0;
        }
    }
}

// OLED �ĳ�ʼ��
void OLED_Init(void)
{
    // ��λ OLED
    OLED_RES_Clr(); // �� RES ��������
    HAL_Delay(200); // ��ʱ 200ms
    OLED_RES_Set(); // �� RES ��������

    // OLED ��ʼ����������
    OLED_WR_Byte(0xAE, OLED_CMD); // �ر� OLED ��ʾ
    OLED_WR_Byte(0xFD, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD); // ����ʱ�ӷ�Ƶ
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD); // ���ö�·���ñ�
    OLED_WR_Byte(0x3F, OLED_CMD); // 1/64 ����
    OLED_WR_Byte(0xD3, OLED_CMD); // ������ʾƫ��
    OLED_WR_Byte(0x00, OLED_CMD); // ��ƫ��
    OLED_WR_Byte(0x40, OLED_CMD); // ������ʼ�е�ַ
    OLED_WR_Byte(0xA1, OLED_CMD); // ������ӳ��
    OLED_WR_Byte(0xC8, OLED_CMD); // ������ɨ�跽��
    OLED_WR_Byte(0xDA, OLED_CMD); // ���� COM ��������
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD); // ���öԱȶ�
    OLED_WR_Byte(0xFF, OLED_CMD); // ���öԱȶ�ֵ
    OLED_WR_Byte(0xD9, OLED_CMD); // ����Ԥ�������
    OLED_WR_Byte(0x22, OLED_CMD); // ����Ԥ�������ֵ
    OLED_WR_Byte(0xDB, OLED_CMD); // ���� VCOMH ��ƽ
    OLED_WR_Byte(0x30, OLED_CMD); // ���� VCOMH ��ƽֵ
    OLED_WR_Byte(0xA4, OLED_CMD); // �ر�������ʾ
    OLED_WR_Byte(0xA6, OLED_CMD); // �رշ�����ʾ

    // ����������ʾ
    OLED_Clear(); // ����
    OLED_DisPlay_On(); // ����ʾ
}

//  oled��ʾsmartcar����

void smartcar_oled_canshu(int mode, int16_t encoderValue1,int16_t encoderValue2,int16_t encoderValue3,int16_t encoderValue4)
{
    OLED_ShowString(0,0,"Mycar>_<",16,1);
    OLED_ShowNum(64,0,mode,2,16,1);
    //OLED_Refresh();
    OLED_ShowNum(0,20,encoderValue1,2,16,1);
    //OLED_Refresh();
    OLED_ShowNum(64,20,encoderValue2,2,16,1);
    //OLED_Refresh();
    OLED_ShowNum(0,40,encoderValue3,2,16,1);
    //OLED_Refresh();
    OLED_ShowNum(64,40,encoderValue4,2,16,1);
    OLED_Refresh();
}


void OLED_SendBuff(uint8_t buff[8][128])
{
    uint8_t i, n;

    for(i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);

        I2C_Start();
        Send_Byte(0x78);
        I2C_WaitAck();
        Send_Byte(0x40);
        I2C_WaitAck();

        for(n = 0; n < 128; n++)
        {
            Send_Byte(buff[i][n]);
            I2C_WaitAck();
        }

        I2C_Stop();
    }
}
