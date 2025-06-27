#include "modbus.h"
#include "config.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "uart.h"

#include "stm32f0xx.h"

typedef enum
{
    MODBUS_CMD_GET_OUT = 1,       // 00001-09999
    MODBUS_CMD_GET_IN = 2,        // 10001-19999
    MODBUS_CMD_GET_REG = 3,       // 40001-49999
    MODBUS_CMD_GET_VAL = 4,       // 30001-39999
    MODBUS_CMD_SET_OUT = 5,       // 00001-09999
    MODBUS_CMD_SET_REG = 6,       // 40001-49999
    MODBUS_CMD_SET_OUT_MULT = 15, // 00001-09999
    MODBUS_CMD_SET_REG_MULT = 16, // 40001-49999
} MODBUS_CMD;

typedef struct Modbus_t
{
    uint8_t addr;
    uint8_t cmd;
    uint16_t reg;
    uint16_t num;
    uint8_t length;
    uint8_t data[20];
} Modbus_t;

#define MODBUS_TO_UINT16(val_h, val_l) ((val_h << 8) | val_l)
#define MODBUS_FROM_UINT16_HIGH(val) (val >> 8)
#define MODBUS_FROM_UINT16_LOW(val) (val & 0xff)

static QueueHandle_t xQueue;
static volatile uint32_t reg_out = 0UL; // 输出寄存器
static volatile uint16_t reg = 0UL;     // 保持寄存器
static uint16_t slaveId = MODBUS_ADDR;

static uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

size_t uart_rcv_override(const void *array, size_t len)
{
    const uint8_t *data = (const uint8_t *)array;

    if (len < 8)
        return 0;

    if (data[0] != slaveId)
        return 1;

    switch (data[1])
    {

    case MODBUS_CMD_GET_OUT:
    case MODBUS_CMD_GET_IN:
    case MODBUS_CMD_GET_REG:
    case MODBUS_CMD_GET_VAL:
    case MODBUS_CMD_SET_OUT:
    case MODBUS_CMD_SET_REG:
    {
        uint16_t crc = MODBUS_TO_UINT16(data[7], data[6]);
        if (crc && (crc != crc16(array, 8 - 2)))
            return 1;

        Modbus_t modbus_t = {
            .addr = data[0],
            .cmd = data[1],
            .reg = MODBUS_TO_UINT16(data[2], data[3]),
            .num = MODBUS_TO_UINT16(data[4], data[5]),
            .length = 0};

        xQueueSend(xQueue, (void *)&modbus_t, 0);
        return 8;
    }
    break;
    case MODBUS_CMD_SET_OUT_MULT:
    case MODBUS_CMD_SET_REG_MULT:
    {
        uint8_t len_total = data[6] + 9;
        if (len < len_total)
            return 0;

        uint16_t crc = MODBUS_TO_UINT16(data[len_total - 1], data[len_total - 2]);
        if (crc && (crc != crc16(array, len_total - 2)))
            return 1;

        Modbus_t modbus_t = {
            .addr = data[0],
            .cmd = data[1],
            .reg = MODBUS_TO_UINT16(data[2], data[3]),
            .num = MODBUS_TO_UINT16(data[4], data[5]),
            .length = 0};

        memcpy(modbus_t.data, data + 7, data[6]);
        xQueueSend(xQueue, (void *)&modbus_t, 0);
        return 8;
    }
    break;

    default:
        break;
    }
    return 1;
}

static size_t modbus_get_out(const Modbus_t *modbus_t, uint8_t *data)
{
    uint8_t reg_number = modbus_t->num / 8 + (modbus_t->num % 8 ? 1 : 0);

    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = reg_number;
    memset(data + 3, 0, reg_number);

    for (uint8_t i = 0; i < modbus_t->num; i++)
    {
        uint16_t reg = modbus_t->reg + i;
        if (reg >= MODBUS_GPIO_NUM)
            continue;

        if (reg_out & (1 << reg))
            data[3 + i / 8] |= 1 << (i % 8);
    }

    return (3 + reg_number);
}

static size_t modbus_get_in(const Modbus_t *modbus_t, uint8_t *data)
{
    uint8_t reg_number = modbus_t->num / 8 + (modbus_t->num % 8 ? 1 : 0);
    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = reg_number;
    memset(data + 3, 0, reg_number);
    // todo
    {
        uint32_t in = 0x87654321; // 虚拟输入

        for (uint8_t i = 0; i < modbus_t->num; i++)
        {
            if (in & (1 << i))
                data[3 + i / 8] |= 1 << (i % 8);
        }
    }
    return (3 + reg_number);
}

static size_t modbus_get_reg(const Modbus_t *modbus_t, uint8_t *data)
{
    uint8_t reg_number = modbus_t->num * 2;
    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = reg_number;
    // todo
    {
        uint16_t val = *(uint16_t *)(0x8003c00); // 虚拟寄存器

        for (uint8_t i = 0; i < modbus_t->num; i++)
        {
            data[2 * i + 3] = MODBUS_FROM_UINT16_HIGH(val);
            data[2 * i + 4] = MODBUS_FROM_UINT16_LOW(val);
        }
    }
    return (3 + reg_number);
}

static size_t modbus_get_val(const Modbus_t *modbus_t, uint8_t *data)
{
    uint8_t reg_number = modbus_t->num * 2;
    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = reg_number;
    // todo
    {
        uint16_t val = 0xfedc; // 虚拟采样
        for (uint8_t i = 0; i < modbus_t->num; i++)
        {
            data[2 * i + 3] = MODBUS_FROM_UINT16_HIGH(val);
            data[2 * i + 4] = MODBUS_FROM_UINT16_LOW(val);
        }
    }

    return (3 + reg_number);
}

static size_t modbus_set_out(const Modbus_t *modbus_t, uint8_t *data)
{

    if (modbus_t->reg >= 0 && modbus_t->reg <= 5)
    {
        // OFF
        if (modbus_t->num == 0)
            reg_out &= ~(1 << modbus_t->reg);
        // ON
        if (modbus_t->num == 0xff00)
            reg_out |= (1 << modbus_t->reg);
    }

    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = MODBUS_FROM_UINT16_HIGH(modbus_t->reg);
    data[3] = MODBUS_FROM_UINT16_LOW(modbus_t->reg);
    data[4] = MODBUS_FROM_UINT16_HIGH(modbus_t->num);
    data[5] = MODBUS_FROM_UINT16_LOW(modbus_t->num);

    return 6;
}

static size_t modbus_set_reg(const Modbus_t *modbus_t, uint8_t *data)
{
    // todo
    if (modbus_t->reg == 0)
        reg = modbus_t->num;
    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = MODBUS_FROM_UINT16_HIGH(modbus_t->reg);
    data[3] = MODBUS_FROM_UINT16_LOW(modbus_t->reg);
    data[4] = MODBUS_FROM_UINT16_HIGH(modbus_t->num);
    data[5] = MODBUS_FROM_UINT16_LOW(modbus_t->num);
    return 6;
}

static size_t modbus_set_out_mult(const Modbus_t *modbus_t, uint8_t *data)
{
    for (uint8_t i = 0; i < modbus_t->num; i++)
    {
        uint16_t reg = modbus_t->reg + i;
        if (reg >= MODBUS_GPIO_NUM)
            continue;

        if (modbus_t->data[i / 8] & (1 << (i % 8)))
            reg_out |= (1 << reg);
        else
            reg_out &= ~(1 << reg);
    }

    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = MODBUS_FROM_UINT16_HIGH(modbus_t->reg);
    data[3] = MODBUS_FROM_UINT16_LOW(modbus_t->reg);
    data[4] = MODBUS_FROM_UINT16_HIGH(modbus_t->num);
    data[5] = MODBUS_FROM_UINT16_LOW(modbus_t->num);
    return 6;
}

static size_t modbus_set_reg_mult(const Modbus_t *modbus_t, uint8_t *data)
{

    for (uint8_t i = 0; i < modbus_t->num; i++)
    {
        uint16_t reg = modbus_t->reg + i;
        if (reg >= MODBUS_GPIO_NUM)
            continue;

        if (reg_out & (1 << reg))
            data[3 + i / 8] |= 1 << (i % 8);
    }

    data[0] = modbus_t->addr;
    data[1] = modbus_t->cmd;
    data[2] = MODBUS_FROM_UINT16_HIGH(modbus_t->reg);
    data[3] = MODBUS_FROM_UINT16_LOW(modbus_t->reg);
    data[4] = MODBUS_FROM_UINT16_HIGH(modbus_t->num);
    data[5] = MODBUS_FROM_UINT16_LOW(modbus_t->num);
    return 6;
}

void flash_sync()
{
    uint32_t addr = 0x8003c00;
    FLASH_Unlock();
    FLASH_ErasePage(addr);
    FLASH_ProgramHalfWord(addr, reg);
    FLASH_ProgramWord(addr + 2, reg_out);
    FLASH_Lock();
}

static void modbus_task(void *param)
{
    static const uint16_t out_pin[MODBUS_GPIO_NUM] = {
        MODBUS_GPIO_OUT0_PIN,
        MODBUS_GPIO_OUT1_PIN,
        MODBUS_GPIO_OUT2_PIN,
        MODBUS_GPIO_OUT3_PIN,
        MODBUS_GPIO_OUT4_PIN,
        MODBUS_GPIO_OUT5_PIN};

    slaveId = *(uint16_t *)(0x8003c00);
    reg = *(uint16_t *)(0x8003c00);
    reg_out = *(uint32_t *)(0x8003c02);

    for (;;)
    {
        // volatile UBaseType_t uxHighWaterMark; // 70
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

        for (int32_t i = 0; i < MODBUS_GPIO_NUM; i++)
        {
            if (reg_out & (1 << i))
                GPIO_ResetBits(MODBUS_GPIO_PORT, out_pin[i]);
            else
                GPIO_SetBits(MODBUS_GPIO_PORT, out_pin[i]);
        }

        Modbus_t modbus_t;
        if (xQueueReceive(xQueue, &modbus_t, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            uint8_t data[64];
            size_t length = 0;

            switch (modbus_t.cmd)
            {
            case MODBUS_CMD_GET_OUT:
                length = modbus_get_out(&modbus_t, data);
                break;
            case MODBUS_CMD_GET_IN:
                length = modbus_get_in(&modbus_t, data);
                break;
            case MODBUS_CMD_GET_REG:
                length = modbus_get_reg(&modbus_t, data);
                break;
            case MODBUS_CMD_GET_VAL:
                length = modbus_get_val(&modbus_t, data);
                break;
            case MODBUS_CMD_SET_OUT:
                length = modbus_set_out(&modbus_t, data);
                break;
            case MODBUS_CMD_SET_REG:
                length = modbus_set_reg(&modbus_t, data);
                break;
            case MODBUS_CMD_SET_OUT_MULT:
                length = modbus_set_out_mult(&modbus_t, data);
                break;
            case MODBUS_CMD_SET_REG_MULT:
                length = modbus_set_reg_mult(&modbus_t, data);
                break;

            default:
                length = 0;
                break;
            }

            if (length)
            {
                uint16_t crc = crc16(data, length);
                data[length + 1] = MODBUS_FROM_UINT16_HIGH(crc);
                data[length] = MODBUS_FROM_UINT16_LOW(crc);
                uart_snd(data, 2 + length);
            }
        }
        else {
            // 延时保存到FLASH, 避免频繁刷新
            if ((reg != *(uint16_t *)(0x8003c00)) || (reg_out != *(uint32_t *)(0x8003c02)))
                flash_sync();
        }
    }
}

void modbus_init()
{
    xQueue = xQueueCreate(2, sizeof(Modbus_t));

    RCC_AHBPeriphClockCmd(MODBUS_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = (MODBUS_GPIO_OUT0_PIN | MODBUS_GPIO_OUT1_PIN | MODBUS_GPIO_OUT2_PIN | MODBUS_GPIO_OUT3_PIN | MODBUS_GPIO_OUT4_PIN | MODBUS_GPIO_OUT5_PIN);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MODBUS_GPIO_PORT, &GPIO_InitStruct);

    xTaskCreate(modbus_task, MODBUS_TASK_NAME, MODBUS_TASK_STACK_SIZE, NULL, MODBUS_TASK_PRIORITY, NULL);
}