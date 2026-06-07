#include "stmflash.h"

/*--------------------------------------------------------------------
函数名称:uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
函数功能:从指定Flash地址读取半字(16位)数据
注意事项:直接读取，不需要解锁Flash
提示说明:
输入参数:faddr - Flash地址(必须是偶数地址)
输出参数:返回读取到的半字数据
--------------------------------------------------------------------*/
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
    return *(__IO uint16_t*)faddr;
}

/*--------------------------------------------------------------------
函数名称:uint32_t GetPage(uint32_t Address)
函数功能:根据地址计算所在的Flash页编号
注意事项:STM32F103每页大小为1KB
提示说明:
输入参数:Address - Flash地址
输出参数:返回页编号(从0开始)
--------------------------------------------------------------------*/
uint32_t GetPage(uint32_t Address)
{
    return (Address - 0x08000000) / 1024;
}

/*--------------------------------------------------------------------
函数名称:void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
函数功能:向Flash写入数据(不检查擦除)
注意事项:调用此函数前必须先擦除目标页面，否则写入会失败
提示说明:
输入参数:WriteAddr - 写入地址, pBuffer - 数据缓冲区, NumToWrite - 半字个数
--------------------------------------------------------------------*/
void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    for(uint16_t i = 0; i < NumToWrite; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
        WriteAddr += 2;
    }
}

/*--------------------------------------------------------------------
函数名称:void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
函数功能:向Flash指定地址写入数据(自动擦除)
注意事项:
  1. STM32 Flash只能将1写成0，不能将0写成1，所以写入前必须擦除
  2. 地址必须是偶数
  3. 每次写入前会检查目标页是否需要擦除
提示说明:
输入参数:WriteAddr - 写入地址(必须是偶数), pBuffer - 数据缓冲区, NumToWrite - 半字个数
输出参数:无
--------------------------------------------------------------------*/
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t PageError;
    uint32_t addrx = WriteAddr;
    uint32_t endaddr = WriteAddr + NumToWrite * 2;

    if(WriteAddr < STM32_FLASH_BASE || WriteAddr % 2 != 0)
        return;

    HAL_FLASH_Unlock();

    for(addrx = WriteAddr; addrx < endaddr; addrx += 1024)
    {
        if(STMFLASH_ReadHalfWord(addrx) != 0xFFFF)
        {
            FlashEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
            FlashEraseInit.PageAddress = addrx;
            FlashEraseInit.NbPages = 1;

            if(HAL_FLASHEx_Erase(&FlashEraseInit, &PageError) != HAL_OK)
            {
                HAL_FLASH_Lock();
                return;
            }
        }
    }

    STMFLASH_Write_NoCheck(WriteAddr, pBuffer, NumToWrite);
    HAL_FLASH_Lock();
}

/*--------------------------------------------------------------------
函数名称:void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
函数功能:从Flash指定地址读取数据
注意事项:读取Flash不需要擦除或解锁操作
提示说明:
输入参数:ReadAddr - 读取地址, pBuffer - 数据存放缓冲区, NumToRead - 半字个数
输出参数:无
--------------------------------------------------------------------*/
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
    for(uint16_t i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);
        ReadAddr += 2;
    }
}

/*--------------------------------------------------------------------
函数名称:void Erase_App2_Full(void)
函数功能:擦除 APP2 整个区域
注意事项:
  1. APP2 区域大小为 20KB (8页，每页1KB)
  2. 擦除后所有数据变为 0xFFFF
  3. 在写入新数据前必须先调用此函数擦除
提示说明:
输入参数:无
输出参数:无
--------------------------------------------------------------------*/
// 擦除 APP2 (8KB = 8页)
void Erase_App2_Full(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef er = {0};
    uint32_t error = 0;

    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.PageAddress = APP2_ADDR;
    er.NbPages = 8;

    HAL_FLASHEx_Erase(&er, &error);
    HAL_FLASH_Lock();
}

/*--------------------------------------------------------------------
函数名称:void WriteToApp2Flash(uint32_t offset, uint8_t *ota_data, uint32_t len)
函数功能:向 APP2 区域写入 OTA 升级数据
注意事项:
  1. 自动处理奇数长度对齐问题(如果长度为奇数，会自动加1对齐到偶数)
  2. 写入前需要先调用 Erase_App2_Full() 擦除
  3. 实际写入地址 = APP2_ADDR + offset
提示说明:
输入参数:offset - 写入偏移地址(相对于 APP2_ADDR), ota_data - OTA数据缓冲区, len - 数据长度(字节)
输出参数:无
--------------------------------------------------------------------*/
// OTA 写入 APP2
void WriteToApp2Flash(uint32_t offset, uint8_t *ota_data, uint32_t len)
{
    uint32_t flash_addr = APP2_ADDR + offset;
    uint32_t write_len = len;

    if(write_len % 2 != 0)
        write_len++;

    STMFLASH_Write(flash_addr, (uint16_t *)ota_data, write_len / 2);
}


