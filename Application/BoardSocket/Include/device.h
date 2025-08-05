#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include <string.h> 

#define FLASH_ADDR (0x800fc00)
#define MODBUS_REG_OUT_NUMBER (8)
#define MODBUS_REG_IN_NUMBER (8)
#define MODBUS_REG_VAL_NUMBER (32)

#define MODBUS_REGS_NUMBER

typedef enum
{
    REG_DEVICEID, // 设备ID
    REG_REBOOT,   // 重启
    REG_UART_0, // 保留
    REG_UART_1, // 保留
    REG_UART_2, // 保留
    REG_UART_3, // 保留
    REG_UART_4, // 保留
    REG_UART_5, // 保留

    REG_NET_IP_0,     
    REG_NET_IP_1,     
    REG_NET_IP_2,     
    REG_NET_IP_3,    

    REG_NET_GATEWAY_0,  
    REG_NET_GATEWAY_1,
    REG_NET_GATEWAY_2,
    REG_NET_GATEWAY_3,

    REG_NET_MASK_0,
    REG_NET_MASK_1,
    REG_NET_MASK_2,
    REG_NET_MASK_3,

    REG_NET_MAC_0,
    REG_NET_MAC_1,
    REG_NET_MAC_2,
    REG_NET_MAC_3,
    REG_NET_MAC_4,
    REG_NET_MAC_5,

    REG_NET_PORT_0,
    REG_NET_PORT_1,
    REG_NET_PORT_2,
    REG_NET_PORT_3,
    REG_NET_PORT_4,
    REG_NET_PORT_5,
    REG_NET_PORT_6,
    REG_NET_PORT_7,


    REG_END,      // 保持寄存器结束
} REG_TYPE;

#pragma pack(push, 1)
typedef struct DeviceMap
{                                            // 1kb
    uint32_t reg_out;                        // 0x800fc00   FLASH_ADDR
    uint32_t reg_in;                         // 0x800fc04   FLASH_ADDR + 4
    uint16_t reg_val[MODBUS_REG_VAL_NUMBER]; // 0x800fc08   FLASH_ADDR + 4 + 4
    uint16_t regs[REG_END];                  // 0x800fc28   FLASH_ADDR + 4 + 4 + 64
} DeviceMap;
#pragma pack(pop)

#define DEVICE_TASK_NAME "device_task"
#define DEVICE_TASK_PRIORITY 3
#define DEVICE_TASK_STACK_SIZE 256

void device_init(void);


#endif