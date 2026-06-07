#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "main.h"

// STM32F103C8T6 64KB Flash
#define STM32_FLASH_SIZE     64
#define STM32_FLASH_BASE     0x08000000

/*==============================================================================
 * STM32F103C8T6 Flash 内存布局
 * 
 * 总容量: 64KB (0x0000 - 0xFFFF)
 * 
 * BOOT Loader:  0x08000000 - 0x08004FFF (20KB)
 * APP1 区域:    0x08005000 - 0x08009FFF (20KB)
 * APP2 区域:    0x0800A000 - 0x0800EFFF (20KB)
 * OTA 标志区:   0x0800F000 - 0x0800FFFF (4KB)
 *============================================================================*/

// Bootloader 起始地址 (0KB - 20KB)
#define BOOT_LOADER_ADDR     0x08000000    // Bootloader 程序存储区域

// APP 应用程序区域 (20KB - 40KB)
#define APP1_ADDR            0x08005000    // 应用程序 A 区 (主程序区)

// APP 应用程序区域 (40KB - 60KB)
#define APP2_ADDR            0x0800A000    // 应用程序 B 区 (备份程序区，和A区完全一样大)

// OTA 升级标志存储区域 (60KB - 64KB)
#define OTA_FLAG_ADDR        0x0800F000    // OTA 升级标志位地址
#define OTA_FILE_NUM         0x0800F004    // 升级文件总包数
#define OTA_FILE_SIZE        0x0800F008    // 升级文件总长度
#define OTA_FILE_CRC         0x0800F00C    // 升级文件 CRC 校验值
#define APP_VALID_ADDR       0x0800F010    // APP 程序有效标志位

//==============================================================================
// Flash 操作函数声明
//==============================================================================

/**
 * @brief 从指定地址读取半字(16位数据)
 * @param faddr: 读取地址
 * @return 读取到的半字数据
 */
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);

/**
 * @brief 向指定地址写入半字数据(自动擦除)
 * @param WriteAddr: 写入地址
 * @param pBuffer: 写入数据缓冲区指针
 * @param NumToWrite: 写入半字个数
 * @note 会自动检测并擦除需要写入的页面
 */
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);

/**
 * @brief 从指定地址读取半字数据
 * @param ReadAddr: 读取地址
 * @param pBuffer: 读取数据存放缓冲区指针
 * @param NumToRead: 读取半字个数
 */
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);

/**
 * @brief 擦除 APP2 整个区域(8KB = 8页)
 * @note 在写入新数据前需要先擦除
 */
void Erase_App2_Full(void);

/**
 * @brief 向 APP2 Flash 写入 OTA 升级数据
 * @param offset: 写入偏移地址(相对于 APP2_ADDR)
 * @param ota_data: OTA 数据缓冲区指针
 * @param len: 数据长度(字节)
 * @note 自动处理奇数长度对齐问题
 */
void WriteToApp2Flash(uint32_t offset, uint8_t *ota_data, uint32_t len);

#endif
