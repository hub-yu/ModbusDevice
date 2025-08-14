#include "device.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "log.h"
#include "modbus.h"
#include "uart.h"
#include "net.h"

#include <string.h>

static QueueHandle_t xQueue;

static DeviceMap deviceMap;

static const uint8_t cover[] = {
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,

};

inline uint16_t UShortCover(uint16_t data)
{
    return ((data << 8) | (data >> 8));
}

static void regRestore()
{

    static const DeviceMap defaultMap = {
        .out = 0,  // 输出寄存器
        .in = 0,   // 输入寄存器
        .val = {}, // 模拟量寄存器

        .regs = {
            // 保持寄存器
            .id = 0,     // 地址寄存器
            .cmd = 0,    // 控制寄存器
            .config = 0, // 配置寄存器

            .net_mac = {0, 0, 0, 0, 0, 0},   // 网络mac寄存器
            .net_ip = {192, 168, 1, 49},     // 网络ip寄存器
            .net_mask = {255, 255, 255, 0},  // 网络子网寄存器
            .net_gateway = {192, 168, 1, 1}, // 网络网关寄存器
            .net_dns = {114, 114, 114, 114}, // 网络DNS寄存器

            .netMap = {
                // 网络通道寄存器
                // 通道0
                {
                    .type = REG_SOCKET_TYPE_UDP,       // 类型
                    .local_port = 50000,               // 本地端口
                    .remote_port = 60000,              // 目标端口
                    .remote_ip = {192, 168, 1, 4},     // 目标地址
                    .remote_domain = "www.xiaopj.com", // 目标域名
                    .timeout_ms = 10000                // 超时重连
                },
                // 通道1
                {
                    .type = REG_SOCKET_TYPE_UDP,       // 类型
                    .local_port = 50001,               // 本地端口
                    .remote_port = 60001,              // 目标端口
                    .remote_ip = {192, 168, 1, 4},     // 目标地址
                    .remote_domain = "www.xiaopj.com", // 目标域名
                    .timeout_ms = 10000                // 超时重连
                },
                // 通道2
                {
                    .type = REG_SOCKET_TYPE_TCPSERVER,      // 类型
                    .local_port = 50002,              // 本地端口
                    .remote_port = 60002,             // 目标端口
                    .remote_ip = {192, 168, 1, 4},    // 目标地址
                    .remote_domain = "www.xiaopj.com", // 目标域名
                    .timeout_ms = 10000               // 超时重连
                },
                // 通道3
                {
                    .type = REG_SOCKET_TYPE_TCPSERVER, // 类型
                    .local_port = 50003,                      // 本地端口
                    .remote_port = 60003,                     // 目标端口
                    .remote_ip = {192, 168, 1, 4},            // 目标地址
                    .remote_domain = "yulc.fun",              // 目标域名
                    .timeout_ms = 10000                       // 超时重连
                },
                // 通道4
                {
                    .type = REG_SOCKET_TYPE_TCPCLIENT,       // 类型
                    .local_port = 50004,               // 本地端口
                    .remote_port = 60004,              // 目标端口
                    .remote_ip = {192, 168, 1, 4},     // 目标地址
                    .remote_domain = "www.xiaopj.com", // 目标域名
                    .timeout_ms = 10000              // 超时重连
                    },
                // 通道5
                {

                    .type = REG_SOCKET_TYPE_TCPCLIENT,       // 类型
                    .local_port = 50005,               // 本地端口
                    .remote_port = 60005,              // 目标端口
                    .remote_ip = {192, 168, 1, 4},     // 目标地址
                    .remote_domain = "www.xiaopj.com", // 目标域名
                    .timeout_ms = 10000                // 超时重连
                },
                // 通道6
                {
                    .type = REG_SOCKET_TYPE_TCPCLIENT_DOMAIN,       // 类型
                    .local_port = 50006,               // 本地端口
                    .remote_port = 60006,              // 目标端口
                    .remote_ip = {192, 168, 1, 4},     // 目标地址
                    .remote_domain = "www.xiaopj.com", // 目标域名
                    .timeout_ms = 10000                // 超时重连
                }

            }}};

    deviceMap = defaultMap;
}

// 生成基于UID的MAC地址
static void generate_mac(uint8_t *mac)
{
    uint8_t uid[12];
    memcpy(uid, (uint8_t *)0x1FFFF7E8, 12);
    LOG_INFO("UID: %02x%02x%02x%02x%02x%02x\r\n", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
    LOG_INFO("UID: %02x%02x%02x%02x%02x%02x\r\n", uid[6], uid[7], uid[8], uid[9], uid[10], uid[11]);

    mac[0] = uid[0] ^ uid[6]; // 第一个字节（设置本地管理位）
    mac[1] = uid[1] ^ uid[7]; // 第二个字节（本地管理位设为1）
    mac[2] = uid[2] ^ uid[8];
    mac[3] = uid[3] ^ uid[9];
    mac[4] = uid[4] ^ uid[10];
    mac[5] = uid[5] ^ uid[11];

    // 设置本地管理地址标志位（第二字节最低位为1）
    mac[0] &= 0xFE; // 确保全局/本地位为0（本地管理）
    mac[0] |= 0x02; // 推荐设置为0x02（避免与常见OUI冲突）
    LOG_INFO("Generated MAC: %02x.%02x.%02x.%02x.%02x.%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void flash_sync()
{

    DeviceMap map = *(DeviceMap *)(FLASH_ADDR);
    if ((memcmp(&map.regs, &deviceMap.regs, sizeof(RegMap))) // 比较保持寄存器
        || (map.out != deviceMap.out))                       // 比较输出寄存器
    {

        LOG_INFO("flash_sync");

        FLASH_Unlock();
        FLASH_ErasePage(FLASH_ADDR);
        FLASH_ProgramWord(FLASH_ADDR, deviceMap.out); // 保存输出寄存器

        // FLASH_ProgramWord(FLASH_ADDR + 4, deviceMap.in); // 保存开关寄存器
        // for (int32_t i = 0; i < MODBUS_REG_VAL_NUMBER; i++)  // 保存输入寄存器
        //     FLASH_ProgramHalfWord(FLASH_ADDR + 4 + 4 + i * 2, deviceMap.val[i]);

        uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
        int32_t len = sizeof(RegMap) / sizeof(uint16_t);
        for (int32_t i = 0; i < len; i++) // 保持寄存器

            FLASH_ProgramHalfWord(FLASH_ADDR + 4 + 4 + 2 * MODBUS_REG_VAL_NUMBER + i * 2, addr[i]);

        FLASH_Lock();
    }
}

size_t uart_rcv_override(const void *array, size_t len)
{
    Modbus modbus = {};
    int32_t ret = modebus_deserilize(&modbus, array, len);
    modbus.from = 254;

    if (ret > 0)
        xQueueSend(xQueue, (void *)&modbus, 0);

    return ret < 0 ? 1 : ret;
}

size_t net_rcv_override(uint8_t sn, const void *array, size_t len)
{
    Modbus modbus = {};
    int32_t ret = modebus_deserilize(&modbus, array, len);
    modbus.from = sn;
    if (ret > 0)
        xQueueSend(xQueue, (void *)&modbus, 0);

    return ret < 0 ? 1 : ret;
}

static void snd(const Modbus *modbus)
{
    uint8_t data[256] = {};
    int32_t len = modebus_serilize(modbus, data);

    switch (modbus->from)
    {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
        net_snd(modbus->from, data, len); // 发送到网络
        break;
    case 254:
        uart_snd(data, len);
        break;

    default:
        break;
    }
}

static void modbus_get_out(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.length = modbus->num / 8 + (modbus->num % 8 ? 1 : 0);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= 32)
            continue;

        if (deviceMap.out & (1 << reg))
            t.data[i / 8] |= 1 << (i % 8);
    }

    snd(&t);
}

static void modbus_set_out(const Modbus *modbus)
{

    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;

    snd(&t);

    if (modbus->reg >= 32)
        return;

    switch (modbus->num)
    {
    case 0: // OFF
        deviceMap.out &= ~(1 << modbus->reg);
        break;

    case 0xff00: // ON
        deviceMap.out |= (1 << modbus->reg);
        break;
    default:
        break;
    }
}

static void modbus_set_out_mult(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;
    t.length = modbus->length;
    memcpy(t.data, modbus->data, modbus->length);

    snd(&t);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= 32)
            continue;

        if (modbus->data[i / 8] & (1 << (i % 8)))
            deviceMap.out |= (1 << reg);
        else
            deviceMap.out &= ~(1 << reg);
    }
}

static void modbus_get_in(const Modbus *modbus)
{

    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.length = modbus->num / 8 + (modbus->num % 8 ? 1 : 0);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= 32)
            continue;

        if (deviceMap.in & (1 << reg))
            t.data[i / 8] |= 1 << (i % 8);
    }

    snd(&t);
}

static void modbus_get_val(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.length = modbus->num * 2;

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= MODBUS_REG_VAL_NUMBER)
            continue;

        t.data[2 * i] = MODBUS_FROM_UINT16_HIGH(deviceMap.val[reg]);
        t.data[2 * i + 1] = MODBUS_FROM_UINT16_LOW(deviceMap.val[reg]);
    }
    snd(&t);
}

static void modbus_get_reg(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.length = modbus->num * 2;

    uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
    int32_t len = sizeof(RegMap) / sizeof(uint16_t);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= len)
            continue;

        uint16_t val = addr[reg];
        val = cover[reg] ? UShortCover(val) : val;
        t.data[2 * i] = MODBUS_FROM_UINT16_HIGH(val);
        t.data[2 * i + 1] = MODBUS_FROM_UINT16_LOW(val);
    }
    snd(&t);
}

static void modbus_set_reg(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;

    snd(&t);

    uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
    int32_t len = sizeof(RegMap) / sizeof(uint16_t);

    if (modbus->reg >= len)
        return;

    uint16_t val = modbus->num;
    val = cover[modbus->reg] ? UShortCover(modbus->num) : val;

    addr[modbus->reg] = val;
}

static void modbus_set_mult(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs.id;
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;
    t.length = modbus->length;
    memcpy(t.data, modbus->data, modbus->length);

    snd(&t);

    uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
    int32_t len = sizeof(RegMap) / sizeof(uint16_t);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= len)
            continue;

        uint16_t val = MODBUS_TO_UINT16(t.data[2 * i], t.data[2 * i + 1]);
        val = cover[reg] ? UShortCover(val) : val;
        addr[reg] = val;
    }
}

static void process_modbus(const Modbus *modbus)
{

    switch (modbus->cmd)
    {
    case MODBUS_CMD_GET_OUT:
        modbus_get_out(modbus);
        break;
    case MODBUS_CMD_SET_OUT:
        modbus_set_out(modbus);
        break;
    case MODBUS_CMD_SET_OUT_MULT:
        modbus_set_out_mult(modbus);
        break;

    case MODBUS_CMD_GET_IN:
        modbus_get_in(modbus);
        break;
    case MODBUS_CMD_GET_VAL:
        modbus_get_val(modbus);
        break;

    case MODBUS_CMD_GET_REG:
        modbus_get_reg(modbus);
        break;
    case MODBUS_CMD_SET_REG:
        modbus_set_reg(modbus);
        break;
    case MODBUS_CMD_SET_REG_MULT:
        modbus_set_mult(modbus);
        break;

    default:

        break;
    }

    // {
    //     LOG_DEBUG("addr: %d, cmd: %d, reg: %d, num: %d, length: %d, data: %s\r\n", modbus.addr, modbus.cmd, modbus.reg, modbus.num, modbus.length, ByteArrayToStr(modbus.data, modbus.length));
    // }
}

static uint16_t out_pin[] = {GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_11, GPIO_Pin_12};
static GPIO_TypeDef *out_port[] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA};
static uint16_t in_pin[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_15};
static GPIO_TypeDef *in_port[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOA};

static void process_cmd(uint16_t cmd)
{
    deviceMap.regs.cmd = 0;

    switch (cmd)
    {
    case REG_CMD_RESTORE:
        regRestore(&deviceMap);
        flash_sync();
        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(out_port[i], out_pin[i], Bit_RESET);
        vTaskDelay(pdMS_TO_TICKS(200)); // 等待应答发送完成
        NVIC_SystemReset();
        break;
    case REG_CMD_RESTART:
        flash_sync();
        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(out_port[i], out_pin[i], Bit_RESET);
        vTaskDelay(pdMS_TO_TICKS(200)); // 等待应答发送完成
        NVIC_SystemReset();
        break;
    default:
        break;
    }
}

static void device_task(void *param)
{

    // 关闭所有输出
    for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
        GPIO_WriteBit(out_port[i], out_pin[i], Bit_RESET);

    while (1)
    {
        // 处理设备控制
        process_cmd(deviceMap.regs.cmd);

        // 更新所有输出
        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(out_port[i], out_pin[i], (deviceMap.out & (1 << i)) ? Bit_SET : Bit_RESET);

        // 等待MODBUS帧
        Modbus modbus;
        if (xQueueReceive(xQueue, &modbus, pdMS_TO_TICKS(2000)) != pdPASS)
        {
            flash_sync();
            continue;
        }

        // 过滤MODBUS帧
        if (modbus.addr && (modbus.addr != deviceMap.regs.id))
            continue;

        // 更新所有输入
        for (int32_t i = 0; i < MODBUS_REG_IN_NUMBER; i++)
        {
            if (GPIO_ReadInputDataBit(in_port[i], in_pin[i]) == Bit_RESET)
                deviceMap.in |= (1 << i);
            else
                deviceMap.in &= ~(1 << i);
        }

        // 处理MODBUS帧
        process_modbus(&modbus);
    }
}

void device_init(void)
{

    xQueue = xQueueCreate(10, sizeof(Modbus));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 初始化输出
    for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {
            .GPIO_Pin = out_pin[i],        //
            .GPIO_Mode = GPIO_Mode_Out_PP, // 复用推挽输出
            .GPIO_Speed = GPIO_Speed_50MHz // 50MHz
        };
        GPIO_Init(out_port[i], &GPIO_InitStruct);

        GPIO_WriteBit(out_port[i], out_pin[i], Bit_RESET);
    }

    // 初始化输入
    for (int32_t i = 0; i < MODBUS_REG_IN_NUMBER; i++)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {
            .GPIO_Pin = in_pin[i],              //
            .GPIO_Mode = GPIO_Mode_IN_FLOATING, // 复用推挽输出
            .GPIO_Speed = GPIO_Speed_50MHz      // 50MHz
        };
        GPIO_Init(in_port[i], &GPIO_InitStruct);
    }

    deviceMap = *(DeviceMap *)(FLASH_ADDR);
    if ((deviceMap.regs.id > 255) || (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_RESET))
        regRestore(&deviceMap);

    if ((deviceMap.regs.config & REG_CONFIG_OUTKEEP) == 0)
        deviceMap.out = 0;

    deviceMap.in = 0;
    memset(deviceMap.val, 0, MODBUS_REG_VAL_NUMBER);

    if ((deviceMap.regs.net_mac[0] == 0)     // mac 0
        || (deviceMap.regs.net_mac[1] == 0)  // mac 1
        || (deviceMap.regs.net_mac[2] == 0)  // mac 2
        || (deviceMap.regs.net_mac[3] == 0)  // mac 3
        || (deviceMap.regs.net_mac[4] == 0)  // mac 4
        || (deviceMap.regs.net_mac[5] == 0)) // mac 5
        generate_mac(deviceMap.regs.net_mac);

    flash_sync();
    xTaskCreate(device_task, DEVICE_TASK_NAME, DEVICE_TASK_STACK_SIZE, NULL, DEVICE_TASK_PRIORITY, NULL);
}