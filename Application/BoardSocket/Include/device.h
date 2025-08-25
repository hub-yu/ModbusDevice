#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include <string.h>
#include "modbus.h"

#define MODBUS_REG_OUT_NUMBER (8)
#define MODBUS_REG_IN_NUMBER (11)
#define MODBUS_REG_VAL_NUMBER (32)

#define MAX_DOMAIN  (32)

#pragma pack(push, 1)
typedef struct Modbus
{
    uint8_t from;   // from interface
    uint8_t addr;   // slave address
    uint16_t index; //序号
    Modbus_PDU pdu; //
} Modbus;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct NetMap
{
    uint16_t type;              // 类型  rw 0:关闭 1:udp 2:tcpserver 3:tcpclient 4:tcpclient_domain
    uint16_t timeout_ms;       // 超时重连  rw
    uint16_t local_port;       // 本地端口  rw
    uint16_t remote_port;      // 目标端口  rw
    uint8_t remote_ip[4];      // 目标地址  rw
    uint8_t remote_domain[MAX_DOMAIN]; // 目标域名  rw
} NetMap;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct RegMap
{
    uint16_t id;     // 地址寄存器  rw  0
    uint16_t cmd;    // 控制寄存器  w   1
    uint16_t config; // 配置寄存器  rw  2

    uint8_t net_mac[6];     // 网络mac寄存器   rw   3
    uint8_t net_ip[4];      // 网络ip寄存器    rw   6
    uint8_t net_mask[4];    // 网络子网寄存器  rw   8
    uint8_t net_gateway[4]; // 网络网关寄存器  rw   10
    uint8_t net_dns[4];     // 网络DNS寄存器   rw   12

    NetMap netMap[6]; // 网络通道寄存器
} RegMap;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct DeviceMap
{
    uint32_t out;                        // 线圈输出寄存器  rw
    uint32_t in;                         // 开关输入寄存器  r
    uint16_t val[MODBUS_REG_VAL_NUMBER]; // 模拟量寄存器    r
    RegMap regs;                             // 保持寄存器

} DeviceMap;
#pragma pack(pop)

#define REG_ID (0)
#define REG_CMD (1)
#define REG_CONFIG (2)
#define REG_MAC (3)
#define REG_IP (6)
#define REG_MASK (8)
#define REG_GATEWAY (10)
#define REG_DNS (12)
#define REG_SOCKET_TYPE(n) (14 + n * 22)
#define REG_SOCKET_TIMEOUT(n) (14 + n * 22 + 1)
#define REG_SOCKET_LOCAL_PORT(n) (14 + n * 22 + 2)
#define REG_SOCKET_REMOTE_PORT(n) (14 + n * 22 + 3)
#define REG_SOCKET_REMOTE_IP(n)  (14 + n * 22 + 4)
#define REG_SOCKET_REMOTE_DOMAIN(n)  (14 + n * 22 + 6)


#define REG_CMD_RESTART (1) //  重启
#define REG_CMD_RESTORE (2) //  恢复出厂设置

#define REG_CONFIG_BAUDRATE (3 << 0) // 波特率配置  0:115200 1:57600 2：19200: 3:9600
#define REG_CONFIG_PROTOCOL (1 << 2) // 串口协议   0: RTU  1: ASCII
#define REG_CONFIG_LOG (1 << 3)     // 串口日志使能

#define REG_CONFIG_OUTKEEP (1 << 4)  // 输出掉电保存使能
#define REG_CONFIG_DHCP (1 << 5)     // DHCP使能

#define REG_SOCKET_TYPE_STYLE (0xf)         // 通讯类型
#define REG_SOCKET_TYPE_STYLE_OFF (0)       // 关闭
#define REG_SOCKET_TYPE_STYLE_UDP (1)       // UDP
#define REG_SOCKET_TYPE_STYLE_TCPSERVER (2) // TCP SERVER
#define REG_SOCKET_TYPE_STYLE_TCPCLIENT (3) // TCP CLIENT
#define REG_SOCKET_TYPE_STYLE_MQTT (4)      // MQTT CLIENT

#define REG_SOCKET_TYPE_PROTOCOL (7 << 4)   // 协议类型
#define REG_SOCKET_TYPE_PROTOCOL_RTU (0 << 4)    // RTU
#define REG_SOCKET_TYPE_PROTOCOL_ASCII (1 << 4)  // ASCII
#define REG_SOCKET_TYPE_PROTOCOL_MBAP (2 << 4)  // MBAP

#define REG_SOCKET_TYPE_DOMAIN (1 << 7)     // DNS



#define FLASH_ADDR (0x800fc00)

#define FLASH_REG_OUT (*(uint32_t *)(FLASH_ADDR))
#define FLASH_REG_IN (*(uint32_t *)(FLASH_ADDR + 4))
#define FLASH_REG_VAL(c) ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (c * 2))))

#define FLASH_REG_ID ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_ID * 2)))
#define FLASH_REG_CMD ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_CMD * 2)))
#define FLASH_REG_CONFIG ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_CONFIG * 2)))
#define FLASH_REG_MAC ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_MAC * 2))
#define FLASH_REG_IP ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_IP * 2))
#define FLASH_REG_MASK ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_MASK * 2))
#define FLASH_REG_GATEWAY ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_GATEWAY * 2))
#define FLASH_REG_DNS ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_DNS * 2))

#define FLASH_REG_CHANNEL_TYPE(c) ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_SOCKET_TYPE(n) * 2)))
#define FLASH_REG_CHANNEL_TIMEOUT(c) ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_SOCKET_TIMEOUT(n) * 2)))
#define FLASH_REG_CHANNEL_LOCAL_PORT(c) ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_SOCKET_LOCAL_PORT(n) * 2)))
#define FLASH_REG_CHANNEL_REMOTE_PORT(c) ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_SOCKET_REMOTE_PORT(n) * 2)))
#define FLASH_REG_CHANNEL_REMOTE_IP(c) ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_SOCKET_REMOTE_IP(n) * 2))
#define FLASH_REG_CHANNEL_REMOTE_DOMAIN(c) ((uint8_t *)(FLASH_ADDR + 4 + 4 + (MODBUS_REG_VAL_NUMBER * 2) + REG_SOCKET_REMOTE_DOMAIN(n) * 2))

#define DEVICE_TASK_NAME "device_task"
#define DEVICE_TASK_PRIORITY 3
#define DEVICE_TASK_STACK_SIZE (512-128)

void device_init(void);

#endif