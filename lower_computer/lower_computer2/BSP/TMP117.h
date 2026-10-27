/*
 * @Date: 2026-10-27 10:42:45
 * @LastEditors: thomas-smith123 thomas-smith@live.cn
 * @LastEditTime: 2026-10-27 10:46:11
 * @FilePath: \stm32lora\BSP\TMP117.h
 */

 
#define Temp_Result     0x00    //温度结果寄存器
#define Configuration   0x01    //配置寄存器
#define THigh_Limit     0x02    //温度上限寄存器
#define TLow_Limit   0x03       //温度下限寄存器
#define EEPROM_UL    0x04       //解锁寄存器
#define EEPROM1      0x05       //寄存器
#define EEPROM2      0x06       //寄存器
#define Temp_Offset  0x07       //温度偏移寄存器
#define EEPROM3      0x08       //EEPROM3 寄存器
#define Device_ID    0x0F       //器件 ID 寄存器