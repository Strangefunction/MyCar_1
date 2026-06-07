#include "fifo.h"
#include "stdio.h"
#include "stmflash.h"
#include "car_control.h"
#include "pid.h"
extern PIDController pid_1;
extern PIDController pid_2;
extern PIDController pid_3;
extern PIDController pid_4;
extern int16_t encoderValue1, encoderValue2, encoderValue3, encoderValue4;                                 //  获取编码器速度
extern int targetSpeed_1, targetSpeed_2, targetSpeed_3, targetSpeed_4;                                     //  目标速度
extern long long int encoder_pulses_1, encoder_pulses_2, encoder_pulses_3, encoder_pulses_4;               //  编码器位置
extern PID_Position wheel1_pid, wheel2_pid, wheel3_pid, wheel4_pid;                                        //  位置环
extern float Total_angle_1;
extern UART_HandleTypeDef huart2;
extern void Move_Distance_Precision(float distance_mm);  //  直走
extern void Rotate_Degree_Precision(float angle_deg);    //  转弯
/*--------------------------------------------------------------------
函数名称:void userFifoInit(FifoStruct *fifostruct)
函数功能:FIFO 队列初始化
注意事项:
提示说明:
输入参数:fifostruct - FIFO结构体指针
输出参数:buffer - 数据缓存区指针
--------------------------------------------------------------------*/
void userFifoInit(FifoStruct *fifostruct,u8 *buffer)
{
	fifostruct->readIndex = 0;
	fifostruct->writeIndex = 0;
	fifostruct->count = 0;
	fifostruct->buffer = buffer;
}
/*--------------------------------------------------------------------
函数名称:u8 userAddByte2Fifo(FifoStruct *fifostruct,u8 adata)
函数功能:添加一个数据到FIFO
注意事项:
提示说明:
输入参数:fifostruct - FIFO结构体指针, adata - 要添加的数据
输出参数:返回1表示成功
--------------------------------------------------------------------*/
u8 userAddByte2Fifo(FifoStruct *fifostruct,u8 adata)
{
	INTX_DISABLE();

	if( fifostruct->count == FifoCapacity )		// 缓存满了丢弃旧数据
	{
		fifostruct->readIndex ++;
		if( fifostruct->readIndex == FifoCapacity )
		{
			fifostruct->readIndex = 0;
		}
		// count 不增不减
	}

	fifostruct->buffer[fifostruct->writeIndex] = adata;
	fifostruct->writeIndex ++;
	fifostruct->count ++;

	if( fifostruct->writeIndex == FifoCapacity )
	{
		fifostruct->writeIndex = 0;		// 写满了则回到开头
	}

	if( fifostruct->count > FifoCapacity )
	{
		fifostruct->count = FifoCapacity;  // 确保count不超过上限
	}

	INTX_ENABLE();

	return 1;							// 成功
}
/*--------------------------------------------------------------------
函数名称:u8 userGetByteFromFifo(FifoStruct *fifostruct,u8 gdata)
函数功能:从FIFO读取一个数据
注意事项:
提示说明:
输入参数:fifostruct - FIFO结构体指针
输出参数:gdata - 读取的数据,返回1表示成功
--------------------------------------------------------------------*/
u8 userGetByteFromFifo(FifoStruct *fifostruct,u8 *gdata)
{
	if( fifostruct->count == 0 )
		return 0;
	else
	{
		*gdata = fifostruct->buffer[fifostruct->readIndex];
		fifostruct->buffer[fifostruct->readIndex] = 0;
		fifostruct->count --;
		fifostruct->readIndex ++;
	}

	if( fifostruct ->readIndex == FifoCapacity )
	{
		fifostruct->readIndex = 0;
	}
	return 1;
}
/*--------------------------------------------------------------------
函数名称:u16 userGetFifoCount(FifoStruct *fifostruct)
函数功能:获取缓存区已存数据个数
注意事项:
提示说明:
输入参数:fifostruct - FIFO结构体指针
输出参数:返回已存数据个数
--------------------------------------------------------------------*/
u16 userGetFifoCount(FifoStruct *fifostruct)
{
	// return ( ( fifostruct->writeIndex + FifoCapacity - fifostruct->readIndex) % FifoCapacity );
	return fifostruct->count;
}

/*--------------------------------------------------------------------
函数名称:void userFifoInit(FifoStruct *fifostruct)
函数功能:FIFO 队列初始化
注意事项:
提示说明:
输入参数:fifostruct - FIFO结构体指针
--------------------------------------------------------------------*/
void userFifoClear(FifoStruct *fifostruct)
{
	fifostruct->readIndex = 0;
	fifostruct->writeIndex = 0;
	fifostruct->count = 0;
}
/*--------------------------------------------------------------------
函数名称:u8 userAnalysisData(FifoStruct *fifostruct,u8 *pdata)
函数功能:解析提取数据包
注意事项:
提示说明:
输入参数:fifostruct - FIFO结构体指针, ptr - 数据包缓存区
输出参数:返回数据包长度
--------------------------------------------------------------------*/
u8 userAnalysisData(FifoStruct *fifostruct,u8 *ptr)
{
	static u8 _analysisStep = 0;
	static u8 _currentPtr = 0;
	u8 _tempData = 0;

	if( userGetByteFromFifo(fifostruct,&_tempData) )
	{
		switch(_analysisStep)
		{
			case 0:							// 截取接收0x09开始的
			{
				if( _tempData == 0x09 )
				{
					_currentPtr = 0;
					ptr[_currentPtr] = _tempData;
					_currentPtr ++;
					_analysisStep ++;
				}
			}break;
			case 1:							// 0x0D结尾
			{
				ptr[_currentPtr] = _tempData;
				if( _tempData == 0x0D )
				{
					// 提取到完整一帧数据
					_analysisStep ++;
				}
				else
				{
					_currentPtr ++;
				}
			}break;
			default:break;
		}
	}
	if( _analysisStep == 2 )
	{
		_analysisStep = 0;
		return _currentPtr;
	}
	else
	{
		return 0;
	}
}



///////////////////////////////////////////////////////////////     协议解析在这里写    /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////     协议解析在这里写    /////////////////////////////////////////////////

extern u8 packetBuffer[512];  //  有效数据包，就是截取包头包尾之后的
extern u8 packetLen;          //  有效数据包的长度
extern FifoStruct myFifo;
extern u8 fifoBuffer[FifoCapacity];

u8 dataLen = 0;               //  截取之后再截取也就是真正最后操作的
u8 *data = 0;

uint32_t file_len = 0;
uint16_t file_num = 0;
uint16_t file_crc = 0;

void My_cmd_response(uint8_t cmd,uint8_t yon);
void JumpToapp(void)   // 小写a，和你调用的名字一样
{
	__disable_irq();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;

	HAL_DeInit();

	RCC->AHBENR  = 0;
	RCC->APB1ENR = 0;
	RCC->APB2ENR = 0;

	for(uint8_t i = 0; i < 4; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	SCB->VTOR = FLASH_BASE | 0x08005000;

	__DSB();
	__ISB();

	typedef void (*pFunction)(void);
	pFunction Jump_App;
	uint32_t Jump_Addr;

	Jump_Addr = *(__IO uint32_t *)(0x08005000 + 4);
	Jump_App  = (pFunction)Jump_Addr;

	__set_MSP(*(__IO uint32_t *)0x08005000);

	Jump_App();
}

/*
 * function : 异或校验
 * */
uint8_t Xor_Check(uint8_t *buf, uint16_t len)
{
    uint8_t xor_result = 0;  // 初始值必须是 0

    for(uint16_t i = 0; i < len; i++)
    {
        xor_result ^= buf[i];  // 逐个字节异或
    }

    return xor_result;
}

/*
 * function : CRC16-Modbus
 * */
uint16_t CRC16_Modbus(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0x8005;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*
 * function : 解析命令1 前期准备  解析并且存入，总包数，总长度，和总crc校验  写入升级标志
 * */
void cmd_1(){
	//  接收升级文件总长度
	file_len = (data[18] << 24) | (data[19] << 16) | (data[20] << 8) | data[21];
	STMFLASH_Write(OTA_FILE_SIZE, (uint16_t *)&file_len, 2);
	//  接收总包数
	file_num = ((uint16_t)data[22] << 8) | data[23];
	STMFLASH_Write(OTA_FILE_NUM, (uint16_t *)&file_num, 1);
	//  总文件的crc
	file_crc = ((uint16_t)data[24] << 8) | data[25];
	STMFLASH_Write(OTA_FILE_CRC, (uint16_t *)&file_crc, 1);

	//  写入升级标志
	uint16_t otaflag = ((uint16_t)0x55 << 8) |  0xAA;
	STMFLASH_Write(OTA_FLAG_ADDR, (uint16_t *)&otaflag, 1);

}

/*
 * function : 解析命令2 开始要升级包  接收一包，就往app2放入一包
 * */
uint32_t ota_file_len;   // 整个数据包长度
uint16_t ota_file_num;   // 总包数
uint16_t ota_file_crc;   // 总CRC
uint16_t now_file_num;   // 当前包号

static uint8_t first_run = 1;  //  是否是第一次接收包
static uint32_t offset = 0;    //  记录当前长度，加在app2地址后面

void cmd_2(void)
{
    // ==================== 第一次进入初始化 ====================
    if(first_run)
    {
        STMFLASH_Read(OTA_FILE_SIZE, (uint16_t*)&ota_file_len, 2);
        STMFLASH_Read(OTA_FILE_NUM, (uint16_t*)&ota_file_num, 1);
        STMFLASH_Read(OTA_FILE_CRC, (uint16_t*)&ota_file_crc, 1);
        Erase_App2_Full();
        first_run = 0;
        offset = 0; // 强制复位
    }
    // ==================== 关中断，安全读取数据 ====================
    __disable_irq();
    now_file_num = ((uint16_t)data[11] << 8) | data[12];
    uint16_t data_len = ((uint16_t)data[4] << 8) | (data[5] - 9);
    __enable_irq();
    // ==================== 写入FLASH（关中断保证稳定） ====================
    __disable_irq();
    WriteToApp2Flash(APP2_ADDR + offset, &data[13], data_len);
    offset += data_len;
    __enable_irq();
    // ==================== 如果是最后一包 ====================
    if(now_file_num >= ota_file_num)
    {
        // CRC校验
        uint16_t calc_crc = CRC16_Modbus((uint8_t*)APP2_ADDR, ota_file_len);
        if(calc_crc != ota_file_crc)
        {
            My_cmd_response(UPGRADESUBCONTRACTING, 1);
            // 复位状态
            first_run = 1;
            offset = 0;
            return;
        }
        // 搬运数据到 APP1
        uint32_t offset_2 = 0;
        uint16_t buf[32];
        while(offset_2 < ota_file_len)
        {
            uint32_t remain = ota_file_len - offset_2;
            uint16_t read_cnt = (remain > 64) ? 32 : (remain + 1)/2;
            STMFLASH_Read(APP2_ADDR + offset_2, buf, read_cnt);
            STMFLASH_Write(APP1_ADDR + offset_2, buf, read_cnt);
            offset_2 += read_cnt * 2;
        }
        // 清除OTA标志
        uint16_t otaflag = ((uint16_t)0xFF << 8) |  0xFF;
        STMFLASH_Write(OTA_FLAG_ADDR, (uint16_t *)&otaflag, 1);

        // 全部完成，复位状态
        first_run = 1;
        offset = 0;
    }
    // ==================== 回复主机，要下一包 ====================
    My_cmd_response(UPGRADESUBCONTRACTING, 0);
}

/*
 * function : 解析命令3 进入app
 * */
void cmd_3(){
	//JumpToapp();  //  app1
}

extern float vx_set;
extern float vy_set;
extern float vw_set;

char debug_print_buf[128];

char debug_print_buf[128];

void pid_aaa_speed()
{
	// 协议校验：头(09 F5 A0) 尾(0D)
	if(data[0] == 0x09 && data[1] == 0xF5 && data[2] == 0xA0 && data[20] == 0x0D)
	{
		uint32_t p_raw, i_raw, d_raw, target_raw;
		float p_val, i_val, d_val, target_val;

		// 1. 大端模式解析原始 4 字节数据
		p_raw      = ((uint32_t)data[4] << 24)  | ((uint32_t)data[5] << 16)  | ((uint32_t)data[6] << 8)  | data[7];
		i_raw      = ((uint32_t)data[8] << 24)  | ((uint32_t)data[9] << 16)  | ((uint32_t)data[10] << 8) | data[11];
		d_raw      = ((uint32_t)data[12] << 24) | ((uint32_t)data[13] << 16) | ((uint32_t)data[14] << 8) | data[15];
		target_raw = ((uint32_t)data[16] << 24) | ((uint32_t)data[17] << 16) | ((uint32_t)data[18] << 8) | data[19];

		// 2. 全部转为浮点数 (使用 memcpy)
		memcpy(&p_val, &p_raw, 4);
		memcpy(&i_val, &i_raw, 4);
		memcpy(&d_val, &d_raw, 4);
		memcpy(&target_val, &target_raw, 4); // 关键：Target 也要按 float 转换

		// 3. 赋值
		wheel1_pid.Kp = p_val;
		wheel1_pid.Ki = i_val;
		wheel1_pid.Kd = d_val;
		Total_angle_1 = target_val; // 现在 Total_angle_1 会变成 500.0

		// 同步其他轮子
		// wheel2_pid.Kp = wheel3_pid.Kp = wheel4_pid.Kp = p_val;
		// wheel2_pid.Ki = wheel3_pid.Ki = wheel4_pid.Ki = i_val;
		// wheel2_pid.Kd = wheel3_pid.Kd = wheel4_pid.Kd = d_val;

		// 4. 打印确认
		int kp_int = (int)p_val;
		int kp_dec = (int)((p_val - kp_int) * 1000);
		if(kp_dec < 0) kp_dec = -kp_dec;
		//
		// // 打印 Target 时也转成整数看，方便识别
		// sprintf(debug_print_buf, "\r\n[OK] P:%d.%03d, Target:%d\r\n",
		// 		kp_int, kp_dec, (int)Total_angle_1);
		//
		// HAL_UART_Transmit(&huart1, (uint8_t*)debug_print_buf, strlen(debug_print_buf), 100);
	}
}

void car_mode_crt()
{
	uint32_t mode_raw, p1_raw, p2_raw, p3_raw;
	float mode_val, p1_val, p2_val, p3_val;
	char debug_buf[128];
	uint16_t len;

	// 1. 将 Mode 解析为 4 字节浮点数 (从 data[4] 开始)
	mode_raw = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | data[7];
	mode_val = *(float*)&mode_raw;

	// 2. 解析后续参数 (每个占 4 字节)
	p1_raw = ((uint32_t)data[8] << 24) | ((uint32_t)data[9] << 16) | ((uint32_t)data[10] << 8) | data[11];
	p2_raw = ((uint32_t)data[12] << 24) | ((uint32_t)data[13] << 16) | ((uint32_t)data[14] << 8) | data[15];
	p3_raw = ((uint32_t)data[16] << 24) | ((uint32_t)data[17] << 16) | ((uint32_t)data[18] << 8) | data[19];

	p1_val = *(float*)&p1_raw;
	p2_val = *(float*)&p2_raw;
	p3_val = *(float*)&p3_raw;

	// 3. 串口打印解析结果
	// 此时你会看到 Mode:1.00, P1:1.00, P2:90.00, P3:0.00
	len = sprintf(debug_buf, "\r\n[DEBUG] Mode:%.2f | P1:%.2f | P2:%.2f | P3:%.2f\r\n",
				  mode_val, p1_val, p2_val, p3_val);
	//HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, len, 100);

	// 4. 逻辑分发 (将浮点 Mode 转为整数判断)
	int mode_int = (int)mode_val;

	if(mode_int == 1) // 假设 Mode 1.0 是精准控制
	{
		// 如果 P1 是 1.0，说明要旋转 90 度 (P2)
		if (p1_val != 0.0f)
		{
			Rotate_Degree_Precision(p2_val);
			len = sprintf(debug_buf, "Result -> Execute Rotate: %.1f deg\r\n", p2_val);
		}
		else // 如果 P1 是 0，执行直线移动
		{
			Move_Distance_Precision(p2_val);
			len = sprintf(debug_buf, "Result -> Execute Move: %.1f mm\r\n", p2_val);
		}
	}
	else // 其他模式 (如速度控制)
	{
		vx_set = p1_val;
		vy_set = p2_val;
		vw_set = p3_val;
		len = sprintf(debug_buf, "Result -> Velocity Set: VX:%.1f VY:%.1f VW:%.1f\r\n", vx_set, vy_set, vw_set);
	}

	//HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, len, 100);
}

/*
 * function : 回复函数，根据命令以及操作成功还是失败返回；
 * name     : My_cmd_response
 * para     : cmd 命令宏定义 ； yon ： 成功0失败1
 *
 * -----------------------------------------------------
 * 包头    地址    信令    版本    数据长度    数据区    校验    包尾
 * 09     FN    FN     01     N                异或    0D
 *                                             (包头开始到数据区)
 * -----------------------------------------------------
 * */

void My_cmd_response(uint8_t cmd, uint8_t yon)
{
    uint8_t response_data[64];
    uint8_t send_len = 0;

    // 定义回复内容
    switch(cmd)
    {

        case REQUESTFORUPGRADE:  //  请求升级
            response_data[0] = 0x09;
            response_data[1] = 0x5F;
            response_data[2] = REQUESTFORUPGRADE;
            response_data[3] = 0x01;
            response_data[4] = 0x00;
            response_data[5] = 0x01;
            response_data[6] = yon;
            response_data[7] = Xor_Check(response_data, 7);
            response_data[8] = 0x0D;
            send_len = 9;
            break;

        case UPGRADESUBCONTRACTING:  //  升级分包
            response_data[0] = 0x09;
            response_data[1] = 0x5F;
            response_data[2] = UPGRADESUBCONTRACTING;
            response_data[3] = 0x01;
            response_data[4] = 0x00;
            response_data[5] = 0x03;
            response_data[6] = 0x00;
            response_data[7] = yon;
            response_data[8] = 0x00;
            response_data[9] = Xor_Check(response_data, 9);
            response_data[10] = 0x0D;
            send_len = 11;
            break;

        case INBOOTLOADER:  //  进入bootloader回复
            response_data[0] = 0x09;
            response_data[1] = 0x5F;
            response_data[2] = INBOOTLOADER;
            response_data[3] = 0x01;
            response_data[4] = 0x00;
            response_data[5] = 0x01;
            response_data[6] = yon;
            response_data[7] = Xor_Check(response_data, 7);
            response_data[8] = 0x0D;
            send_len = 9;
            break;
        case REQ_QUERY_WATCH_FACE:  //  查询表盘
            response_data[0] = 0x09;
            response_data[1] = 0x5F;
            response_data[2] = REQ_QUERY_WATCH_FACE;
            response_data[3] = 0x01;
            response_data[4] = 0x00;
            response_data[5] = 0x05;
            response_data[6] = 0x01;
            response_data[7] = 0x00;
            response_data[8] = 0x01;
            response_data[9] = 0x00;
            response_data[10] = 0x01;
            response_data[11] = Xor_Check(response_data, 11);
            response_data[12] = 0x0D;
            send_len = 13;
        case REQ_QUERY_VERSION:  //  请求版本
                    response_data[0] = 0x09;
                    response_data[1] = 0x5F;
                    response_data[2] = REQ_QUERY_VERSION;
                    response_data[3] = 0x01;
                    response_data[4] = 0x00;
                    response_data[5] = 0x11;
                    response_data[6]  = (VERSION_HARD >> 56) & 0xFF;
                    response_data[7]  = (VERSION_HARD >> 48) & 0xFF;
                    response_data[8]  = (VERSION_HARD >> 40) & 0xFF;
                    response_data[9]  = (VERSION_HARD >> 32) & 0xFF;
                    response_data[10] = (VERSION_HARD >> 24) & 0xFF;
                    response_data[11] = (VERSION_HARD >> 16) & 0xFF;
                    response_data[12] = (VERSION_HARD >> 8)  & 0xFF;
                    response_data[13] =  VERSION_HARD        & 0xFF;
                    response_data[14] = (VERSION_SOFT >> 56) & 0xFF;
                    response_data[15] = (VERSION_SOFT >> 48) & 0xFF;
                    response_data[16] = (VERSION_SOFT >> 40) & 0xFF;
                    response_data[17] = (VERSION_SOFT >> 32) & 0xFF;
                    response_data[18] = (VERSION_SOFT >> 24) & 0xFF;
                    response_data[19] = (VERSION_SOFT >> 16) & 0xFF;
                    response_data[20] = (VERSION_SOFT >> 8)  & 0xFF;
                    response_data[21] =  VERSION_SOFT        & 0xFF;
                    response_data[22] =  yon;
                    response_data[23] = Xor_Check(response_data, 23);
                    response_data[24] = 0x0D;
                    send_len = 25;
            break;

        default:
            return;
    }

    if(send_len > 0 && send_len < 64)
    {
        HAL_UART_Transmit(&huart2, response_data, send_len, 1000);
    }
}

/*
 * function : 主要的处理函数，解析命令->拆解->回复
 * */

void My_cmd_analysis(){
			  // 提取有效包数据
		      packetLen = userAnalysisData(&myFifo, packetBuffer);
		      if (packetLen > 0) {
		          // 解析数据
		          dataLen = packetLen - 2;
		          data = &packetBuffer[0];
		          u8 cmd = data[2];
		          // 处理命令
		          switch (cmd) {
		          	  //  请求升级
		              case REQUESTFORUPGRADE:
		            	  cmd_1();
		            	  My_cmd_response(cmd,0);
		                  break;
		              //  分包请求
		              case UPGRADESUBCONTRACTING:
		            	  cmd_2();          //  命令函数里面有回复
		                  break;
			          //  进入bootloader
			          case INBOOTLOADER:
			        	  //  擦除升级标志，跳转到app
			        	  uint16_t otaflag = ((uint16_t)0x55 << 8) |  0xAA;
			        	  STMFLASH_Write(OTA_FLAG_ADDR, (uint16_t *)&otaflag, 1);
                          //My_cmd_response(INBOOTLOADER,0);  //  
			              cmd_3();
			              break;
			          case REQ_QUERY_WATCH_FACE:  //  查询表盘
			        	  My_cmd_response(cmd,0);
			        	  break;
			          case REQ_QUERY_VERSION:     //  查询版本
			        	  My_cmd_response(cmd,0);
			        	  break;
		              case PID_SPEED_SET:     //  设置pid参数
		          	      pid_aaa_speed();
		          	      My_cmd_response(cmd,0);
		          	     break;
					  case CAR_MODE_CRT:     //  设置CAR_MODE_CRT //  直行转向
		          	      car_mode_crt();
		          	      My_cmd_response(cmd,0);
		          	     break;

		              default:
		                  break;
		          }  
		      }
}















