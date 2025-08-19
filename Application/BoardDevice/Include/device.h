#ifndef DEVICE_H
#define DEVICE_H

#include "modbus.h"

#define MODBUS_REG_OUT_NUMBER (6)

#pragma pack(push, 1)
typedef struct Modbus
{
    uint8_t addr;   // slave address
    Modbus_PDU pdu; //
} Modbus;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct RegMap
{
    uint16_t id;     // 地址寄存器  rw  0
    uint16_t cmd;    // 控制寄存器  w   1
    uint16_t config; // 配置寄存器  rw  2
} RegMap;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct DeviceMap
{
    uint32_t out; // 线圈输出寄存器  rw
    RegMap regs;  // 保持寄存器
} DeviceMap;
#pragma pack(pop)

#define REG_ID (0)
#define REG_CMD (1)
#define REG_CONFIG (2)

#define REG_CMD_RESTART (1) //  重启
#define REG_CMD_RESTORE (2) //  恢复出厂设置

#define REG_CONFIG_BAUDRATE (3 << 0) // 波特率配置  0:115200 1:57600 2：19200: 3:9600
#define REG_CONFIG_PROTOCOL (1 << 2) // 串口协议   0: RTU  1: ASCII
#define REG_CONFIG_LOG (1 << 3)      // 串口日志使能

#define REG_CONFIG_OUTKEEP (1 << 4) // 输出掉电保存使能

#define FLASH_ADDR (0x8003c00)

#define FLASH_REG_OUT (*(uint32_t *)(FLASH_ADDR))
#define FLASH_REG_ID ((*(uint16_t *)(FLASH_ADDR + 4 + REG_ID * 2)))
#define FLASH_REG_CMD ((*(uint16_t *)(FLASH_ADDR + 4 + REG_CMD * 2)))
#define FLASH_REG_CONFIG ((*(uint16_t *)(FLASH_ADDR + 4 + REG_CONFIG * 2)))

#define DEVICE_TASK_NAME "device_task"
#define DEVICE_TASK_PRIORITY 1
#define DEVICE_TASK_STACK_SIZE 128

void device_init(void);

#endif