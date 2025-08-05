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

static volatile DeviceMap deviceMap = {
    .reg_out = 0,  // 输出寄存器
    .reg_in = 0,   // 输入寄存器
    .reg_val = {}, // 模拟量寄存器
    .regs = {
        0,                                       // 设备ID
        0,                                       // 重启
        0, 0, 0, 0, 0, 0,                        // 串口保留
        192, 168, 1, 49,                         // IP地址
        192, 168, 1, 1,                          // 网关
        255, 255, 255, 0,                        // 掩码
        0, 0, 0, 0, 0, 0,                        // 全0时由 stm32序号动态生成
        50000,                                   // udp 端口
        50001,                                   // tcp server 端口
        50002, 50003, 50004, 50005, 50006, 50007 // 其他端口
    } // 保持寄存器
};

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

size_t uart_rcv_override(const void *array, size_t len)
{
    Modbus modbus = {};
    int32_t ret = modebus_deserilize(&modbus, array, len);
    modbus.from = 255;

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
    uint8_t data[64] = {};
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
    case 7:
        net_snd(modbus->from, data, len); // 发送到网络
        break;

    default:
        uart_snd(data, len);
        break;
    }
}

static void modbus_get_out(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.length = modbus->num / 8 + (modbus->num % 8 ? 1 : 0);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= 32)
            continue;

        if (deviceMap.reg_out & (1 << reg))
            t.data[i / 8] |= 1 << (i % 8);
    }

    snd(&t);
}

static void modbus_set_out(const Modbus *modbus)
{

    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;

    snd(&t);

    if (modbus->reg >= 32)
        return;

    switch (modbus->num)
    {
    case 0: // OFF
        deviceMap.reg_out &= ~(1 << modbus->reg);
        break;

    case 0xff00: // ON
        deviceMap.reg_out |= (1 << modbus->reg);
        break;
    default:
        break;
    }
}

static void modbus_set_out_mult(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
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
            deviceMap.reg_out |= (1 << reg);
        else
            deviceMap.reg_out &= ~(1 << reg);
    }
}

static void modbus_get_in(const Modbus *modbus)
{

    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.length = modbus->num / 8 + (modbus->num % 8 ? 1 : 0);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= 32)
            continue;

        if (deviceMap.reg_in & (1 << reg))
            t.data[i / 8] |= 1 << (i % 8);
    }

    snd(&t);
}

static void modbus_get_val(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.length = modbus->num * 2;

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= MODBUS_REG_VAL_NUMBER)
            continue;

        t.data[2 * i] = MODBUS_FROM_UINT16_HIGH(deviceMap.reg_val[reg]);
        t.data[2 * i + 1] = MODBUS_FROM_UINT16_LOW(deviceMap.reg_val[reg]);
    }
    snd(&t);
}

static void modbus_get_reg(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.length = modbus->num * 2;

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= REG_END)
            continue;

        t.data[2 * i] = MODBUS_FROM_UINT16_HIGH(deviceMap.regs[reg]);
        t.data[2 * i + 1] = MODBUS_FROM_UINT16_LOW(deviceMap.regs[reg]);
    }
    snd(&t);
}

static void modbus_set_reg(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;

    snd(&t);

    if (modbus->reg >= REG_END)
        return;

    deviceMap.regs[modbus->reg] = modbus->num;
}

static void modbus_set_reg_mult(const Modbus *modbus)
{
    Modbus t = {};
    t.from = modbus->from;
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;
    t.length = modbus->length;
    memcpy(t.data, modbus->data, modbus->length);

    snd(&t);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= REG_END)
            continue;

        deviceMap.regs[reg] = MODBUS_TO_UINT16(t.data[2 * i], t.data[2 * i + 1]);
    }
}

static void flash_sync()
{

    DeviceMap map = *(DeviceMap *)(FLASH_ADDR);
    uint16_t temp_regs[REG_END];
    memcpy(temp_regs, (void *)deviceMap.regs, REG_END * sizeof(uint16_t));

    // 没有变化，不需要写入
    if (map.reg_out == deviceMap.reg_out &&
        memcmp(map.regs, temp_regs, REG_END * sizeof(uint16_t)) == 0)
        return;
    LOG_INFO("flash_sync");

    FLASH_Unlock();
    FLASH_ErasePage(FLASH_ADDR);

    FLASH_ProgramWord(FLASH_ADDR, deviceMap.reg_out);

    for (int32_t i = 0; i < REG_END; i++)
        FLASH_ProgramHalfWord(FLASH_ADDR + 4 + 4 + 2 * MODBUS_REG_VAL_NUMBER + i * 2, deviceMap.regs[i]);

    FLASH_Lock();
}

static void device_task(void *param)
{
    static uint16_t out_pin[] = {GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_11, GPIO_Pin_12};
    static GPIO_TypeDef *out_port[] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA};
    static uint16_t in_pin[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_15};
    static GPIO_TypeDef *in_port[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOA};

    for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
        GPIO_WriteBit(out_port[i], out_pin[i], Bit_RESET);

    vTaskDelay(pdMS_TO_TICKS(500)); // 避免电源未稳定时的抖动

    while (1)
    {
        if (deviceMap.regs[REG_REBOOT])
        {
            vTaskDelay(pdMS_TO_TICKS(200)); //等待应答发送完成
            NVIC_SystemReset();
            for (;;)
            {
            }
        }

        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(out_port[i], out_pin[i], (deviceMap.reg_out & (1 << i)) ? Bit_SET : Bit_RESET);

        Modbus modbus;
        if (xQueueReceive(xQueue, &modbus, pdMS_TO_TICKS(2000)) != pdPASS)
        {
            flash_sync();
            continue;
        }

        if (modbus.addr && (modbus.addr != deviceMap.regs[REG_DEVICEID]))
            continue;

        for (int32_t i = 0; i < MODBUS_REG_IN_NUMBER; i++)
        {
            if (GPIO_ReadInputDataBit(in_port[i], in_pin[i]) == Bit_RESET)
                deviceMap.reg_in |= (1 << i);
            else
                deviceMap.reg_in &= ~(1 << i);
        }

        switch (modbus.cmd)
        {
        case MODBUS_CMD_GET_OUT:
            modbus_get_out(&modbus);
            break;
        case MODBUS_CMD_SET_OUT:
            modbus_set_out(&modbus);
            break;
        case MODBUS_CMD_SET_OUT_MULT:
            modbus_set_out_mult(&modbus);
            break;

        case MODBUS_CMD_GET_IN:
            modbus_get_in(&modbus);
            break;
        case MODBUS_CMD_GET_VAL:
            modbus_get_val(&modbus);
            break;

        case MODBUS_CMD_GET_REG:
            modbus_get_reg(&modbus);
            break;
        case MODBUS_CMD_SET_REG:
            modbus_set_reg(&modbus);
            break;
        case MODBUS_CMD_SET_REG_MULT:
            modbus_set_reg_mult(&modbus);
            break;

        default:

            break;
        }
        // {
        //     LOG_DEBUG("addr: %d, cmd: %d, reg: %d, num: %d, length: %d, data: %s\r\n", modbus.addr, modbus.cmd, modbus.reg, modbus.num, modbus.length, ByteArrayToStr(modbus.data, modbus.length));
        // }
    }
}

void device_init(void)
{

    xQueue = xQueueCreate(10, sizeof(Modbus));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, //
        .GPIO_Mode = GPIO_Mode_Out_PP,                                                           // 复用推挽输出
        .GPIO_Speed = GPIO_Speed_50MHz                                                           // 50MHz
    };
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15; //
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 从掉电存储中读取设备状态
    if ((*(uint16_t *)(FLASH_ADDR + 4 + 4 + 2 * MODBUS_REG_VAL_NUMBER) <= 255) //校验 设备ID
        && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_SET))   // 恢复默认配置标识
        deviceMap = *(DeviceMap *)(FLASH_ADDR);
    else
    {
        uint8_t mac[6] = {};
        if (deviceMap.regs[REG_NET_MAC_0] == 0 &&
            deviceMap.regs[REG_NET_MAC_1] == 0 &&
            deviceMap.regs[REG_NET_MAC_2] == 0 &&
            deviceMap.regs[REG_NET_MAC_3] == 0 &&
            deviceMap.regs[REG_NET_MAC_4] == 0 &&
            deviceMap.regs[REG_NET_MAC_5] == 0)
            generate_mac(mac); // 生成基于UID的MAC地址

        for (int32_t i = 0; i < 6; i++)
            deviceMap.regs[REG_NET_MAC_0 + i] = mac[i];
        flash_sync();
    }

    xTaskCreate(device_task, DEVICE_TASK_NAME, DEVICE_TASK_STACK_SIZE, NULL, DEVICE_TASK_PRIORITY, NULL);
}