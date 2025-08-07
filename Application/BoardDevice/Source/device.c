#include "device.h"
#include "stm32f0xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "modbus.h"
#include "uart.h"

#include <string.h>

static QueueHandle_t xQueue;

#define FLASH_ADDR (0x8003c00)
#define MODBUS_ADDR (17)
#define MODBUS_REG_OUT_NUMBER (6)
#define MODBUS_REG_IN_NUMBER (0)
#define MODBUS_REG_VAL_NUMBER (2)

#define MODBUS_REGS_NUMBER

typedef enum
{
    REG_DEVICEID, // 设备ID
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

static volatile DeviceMap deviceMap = {
    .reg_out = 0,  // 输出寄存器
    .reg_in = 0,   // 输入寄存器
    .reg_val = {}, // 模拟量寄存器
    .regs = {}     // 保持寄存器
};

size_t uart_rcv_override(const void *array, size_t len)
{
    Modbus modbus = {};
    int32_t ret = modebus_deserilize(&modbus, array, len);

    if (ret > 0)
        xQueueSend(xQueue, (void *)&modbus, 0);

    return ret < 0 ? 1 : ret;
}

static void modbus_get_out(const Modbus *modbus)
{
    Modbus t = {};
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

    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);
}

static void modbus_set_out(const Modbus *modbus)
{

    Modbus t = {};
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;

    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);

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
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;
    t.length = modbus->length;
    memcpy(t.data, modbus->data, modbus->length);

    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);

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

    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);
}

static void modbus_get_val(const Modbus *modbus)
{
    Modbus t = {};
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
    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);
}

static void modbus_get_reg(const Modbus *modbus)
{
    Modbus t = {};
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
    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);
}

static void modbus_set_reg(const Modbus *modbus)
{
    Modbus t = {};
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;

    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);

    if (modbus->reg >= REG_END)
        return;

    deviceMap.regs[modbus->reg] = modbus->num;
}

static void modbus_set_reg_mult(const Modbus *modbus)
{
    Modbus t = {};
    t.addr = deviceMap.regs[REG_DEVICEID];
    t.cmd = modbus->cmd;
    t.reg = modbus->reg;
    t.num = modbus->num;
    t.length = modbus->length;
    memcpy(t.data, modbus->data, modbus->length);

    uint8_t data[64] = {};
    int32_t len = modebus_serilize(&t, data);

    uart_snd(data, len);

    for (uint8_t i = 0; i < modbus->num; i++)
    {
        uint16_t reg = modbus->reg + i;
        if (reg >= REG_END)
            continue;

        deviceMap.regs[reg] = MODBUS_TO_UINT16(t.data[2 * i], t.data[2 * i + 1]);
    }
}

void flash_sync()
{

    DeviceMap map = *(DeviceMap *)(FLASH_ADDR);
    uint16_t temp_regs[REG_END];
    memcpy(temp_regs, (void *)deviceMap.regs, REG_END * sizeof(uint16_t));

    // 没有变化，不需要写入
    if (
        // map.reg_out == deviceMap.reg_out &&
        memcmp(map.regs, temp_regs, REG_END * sizeof(uint16_t)) == 0)
        return;

    FLASH_Unlock();
    FLASH_ErasePage(FLASH_ADDR);

    // FLASH_ProgramWord(FLASH_ADDR, deviceMap.reg_out);

    for (int32_t i = 0; i < REG_END; i++)
        FLASH_ProgramHalfWord(FLASH_ADDR + 4 + 4 + 2 * MODBUS_REG_VAL_NUMBER + i * 2, deviceMap.regs[i]);

    FLASH_Lock();
}

void device_task(void *param)
{
    static uint16_t out_pin[] = {GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_9, GPIO_Pin_10};

    for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
        GPIO_WriteBit(GPIOA, out_pin[i], Bit_SET);

    vTaskDelay(pdMS_TO_TICKS(500)); // 避免电源未稳定时的抖动

    // 从掉电存储中读取设备状态
    if (*(uint16_t *)(FLASH_ADDR + 4 + 4 + 2 * MODBUS_REG_VAL_NUMBER) <= 255)
        deviceMap = *(DeviceMap *)(FLASH_ADDR);

    while (1)
    {

        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(GPIOA, out_pin[i], (deviceMap.reg_out & (1 << i)) ? Bit_RESET : Bit_SET);

        Modbus modbus;
        if (xQueueReceive(xQueue, &modbus, pdMS_TO_TICKS(2000)) != pdPASS)
        {
            flash_sync();
            continue;
        }

        if (modbus.addr && (modbus.addr != deviceMap.regs[REG_DEVICEID]))
            continue;

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

    xQueue = xQueueCreate(2, sizeof(Modbus));

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    xTaskCreate(device_task, DEVICE_TASK_NAME, DEVICE_TASK_STACK_SIZE, NULL, DEVICE_TASK_PRIORITY, NULL);
}