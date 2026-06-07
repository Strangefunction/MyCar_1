#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>
#include "stm32f1xx.h"

typedef uint8_t u8;
typedef uint16_t u16;

#define INTX_DISABLE() __disable_irq()
#define INTX_ENABLE() __enable_irq()

#define FifoCapacity	512*2

typedef struct
{
	uint16_t writeIndex;			// 当前写的索引
	uint16_t readIndex;				// 当前写的索引
	uint16_t count;					// 当前缓存已存数据�???�???
//	uint16_t capacity;				// 缓存容量
	uint8_t  *buffer;				// 缓存
}FifoStruct;


/////////////////////    一些命�??
#define REQUESTFORUPGRADE        0xF1      //  请求升级
#define UPGRADESUBCONTRACTING    0xFF      //  升级分包
#define INBOOTLOADER             0xE0      //  进入bootloader
//#define REQ_QUERY_SOUND_EFFECT   0xF0      // 请求查询音效
#define REQ_QUERY_WATCH_FACE     0xEF      // 请求查询表盘
#define REQ_QUERY_VERSION        0xF0      // 请求查询版本
//#define REQ_WRITE_SOUND_EFFECT   0xF1      // 请求写入音效
#define PID_SPEED_SET        0xA0      // 设置pid参数
#define CAR_MODE_CRT         0xA1      //  设置车的命令以及传递参数

#define VERSION_HARD             0xAABBCCDDC1002233ULL  //  八字节硬件版本，（前四字节厂家代码）
#define VERSION_SOFT             0x0000000000000001ULL  //  八字节软件版本， 前四字节是0

extern uint32_t file_len;                   //  升级文件总长�?
extern uint16_t file_num;                   //  升级总包�?
extern uint16_t file_crc;                   //  总文件的crc校验

////////////////////     一些命�??

void userFifoInit(FifoStruct *fifostruct,u8 *buffer);
u8 userAddByte2Fifo(FifoStruct *fifostruct,u8 adata);
u8 userGetByteFromFifo(FifoStruct *fifostruct,u8 *gdata);
u16 userGetFifoCount(FifoStruct *fifostruct);
u8 userAnalysisData(FifoStruct *fifostruct,u8 *ptr);
void userFifoClear(FifoStruct *fifostruct);

void My_cmd_analysis(void);
void My_cmd_response(uint8_t cmd, uint8_t yon);
#endif

