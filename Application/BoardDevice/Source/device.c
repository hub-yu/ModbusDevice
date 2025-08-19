#include "device.h"
#include "stm32f0xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "modbus.h"
#include "uart.h"
// #include "log.h"

#include <string.h>

static QueueHandle_t xQueue;

static DeviceMap deviceMap;

static void regRestore()
{

    static const DeviceMap defaultMap = {
        .out = 0, // 输出寄存器
        .regs = {
            // 保持寄存器
            .id = 1,     // 地址寄存器
            .cmd = 0,    // 控制寄存器
            .config = 0, // 配置寄存器
        }};
    deviceMap = defaultMap;
}

size_t uart_rcv_override(const void *array, size_t len)
{
    // ASCII
    if (deviceMap.regs.config & REG_CONFIG_PROTOCOL)
    {
        Modbus_ASCII ascii = {};
        int32_t ret = modebus_deserilize_ascii(&ascii, array, len);
        if (ret > 0)
        {
            Modbus modbus = {
                .addr = ascii.addr,
                .pdu = ascii.pdu};
            xQueueSend(xQueue, (void *)&modbus, 0);
        }
        return ret < 0 ? 1 : ret;
    }

    // RTU
    else
    {
        Modbus_RTU rtu = {};
        int32_t ret = modebus_deserilize_rtu(&rtu, array, len);
        if (ret > 0)
        {
            Modbus modbus = {
                .addr = rtu.addr,
                .pdu = rtu.pdu};
            xQueueSend(xQueue, (void *)&modbus, 0);
        }
        return ret < 0 ? 1 : ret;
    }
}

static void snd(const Modbus *modbus)
{
    uint8_t data[64] = {};
    int32_t len = 0;

    if (deviceMap.regs.config & REG_CONFIG_PROTOCOL)
    {
        Modbus_ASCII ascii = {
            .addr = modbus->addr,
            .pdu = modbus->pdu};
        len = modebus_serilize_ascii(&ascii, data);
    }
    else
    {
        Modbus_RTU rtu = {
            .addr = modbus->addr,
            .pdu = modbus->pdu};
        len = modebus_serilize_rtu(&rtu, data);
    }
    uart_snd(data, len);
}

static void modbus_get_out(const Modbus *modbus)
{
    Modbus t = {};
    t.addr = deviceMap.regs.id;

    t.pdu.cmd = modbus->pdu.cmd;
    t.pdu.length = modbus->pdu.num / 8 + (modbus->pdu.num % 8 ? 1 : 0);

    for (uint8_t i = 0; i < modbus->pdu.num; i++)
    {
        uint16_t reg = modbus->pdu.reg + i;
        if (reg >= 32)
            continue;

        if (deviceMap.out & (1 << reg))
            t.pdu.data[i / 8] |= 1 << (i % 8);
    }

    snd(&t);
}

static void modbus_set_out(const Modbus *modbus)
{

    {
        Modbus t = {};
        t.addr = deviceMap.regs.id;
        t.pdu.cmd = modbus->pdu.cmd;
        t.pdu.reg = modbus->pdu.reg;
        t.pdu.num = modbus->pdu.num;

        snd(&t);
    }

    if (modbus->pdu.reg >= 32)
        return;

    switch (modbus->pdu.num)
    {
    case 0: // OFF
        deviceMap.out &= ~(1 << modbus->pdu.reg);
        break;

    case 0xff00: // ON
        deviceMap.out |= (1 << modbus->pdu.reg);
        break;
    default:
        break;
    }
}

static void modbus_set_out_mult(const Modbus *modbus)
{
    {
        Modbus t = {};
        t.addr = deviceMap.regs.id;
        t.pdu.cmd = modbus->pdu.cmd;
        t.pdu.reg = modbus->pdu.reg;
        t.pdu.num = modbus->pdu.num;
        // t.pdu.length = modbus->pdu.length;
        // memcpy(t.data, modbus->data, modbus->length);
        snd(&t);
    }

    for (uint8_t i = 0; i < modbus->pdu.num; i++)
    {
        uint16_t reg = modbus->pdu.reg + i;
        if (reg >= 32)
            continue;

        if (modbus->pdu.data[i / 8] & (1 << (i % 8)))
            deviceMap.out |= (1 << reg);
        else
            deviceMap.out &= ~(1 << reg);
    }
}

static void modbus_get_in(const Modbus *modbus)
{
    Modbus t = {};
    t.addr = deviceMap.regs.id;

    t.pdu.cmd = modbus->pdu.cmd;
    t.pdu.length = modbus->pdu.num / 8 + (modbus->pdu.num % 8 ? 1 : 0);

    // for (uint8_t i = 0; i < modbus->pdu.num; i++)
    // {

    //     uint16_t reg = modbus->pdu.reg + i;
    //     if (reg >= 32)
    //         continue;

    //     if (deviceMap.in & (1 << reg))
    //         t.pdu.data[i / 8] |= 1 << (i % 8);
    // }

    snd(&t);
}

static void modbus_get_val(const Modbus *modbus)
{
    Modbus t = {};
    t.addr = deviceMap.regs.id;

    t.pdu.cmd = modbus->pdu.cmd;
    t.pdu.length = modbus->pdu.num * 2;

    // for (uint8_t i = 0; i < modbus->pdu.num; i++)
    // {
    //     uint16_t reg = modbus->pdu.reg + i;
    //     if (reg >= MODBUS_REG_VAL_NUMBER)
    //         continue;

    //     t.pdu.data[2 * i] = MODBUS_FROM_UINT16_HIGH(deviceMap.val[reg]);
    //     t.pdu.data[2 * i + 1] = MODBUS_FROM_UINT16_LOW(deviceMap.val[reg]);
    // }
    snd(&t);
}

static void modbus_get_reg(const Modbus *modbus)
{
    Modbus t = {};
    t.addr = deviceMap.regs.id;

    t.pdu.cmd = modbus->pdu.cmd;
    t.pdu.length = modbus->pdu.num * 2;

    uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
    int32_t len = sizeof(RegMap) / sizeof(uint16_t);

    for (uint8_t i = 0; i < modbus->pdu.num; i++)
    {
        uint16_t reg = modbus->pdu.reg + i;
        if (reg >= len)
            continue;

        uint16_t val = addr[reg];
        // val = cover(reg) ? UShortCover(val) : val;
        t.pdu.data[2 * i] = MODBUS_FROM_UINT16_HIGH(val);
        t.pdu.data[2 * i + 1] = MODBUS_FROM_UINT16_LOW(val);
    }
    snd(&t);
}

static void modbus_set_reg(const Modbus *modbus)
{
    {
        Modbus t = {};
        t.addr = deviceMap.regs.id;

        t.pdu.cmd = modbus->pdu.cmd;
        t.pdu.reg = modbus->pdu.reg;
        t.pdu.num = modbus->pdu.num;

        snd(&t);
    }

    uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
    int32_t len = sizeof(RegMap) / sizeof(uint16_t);

    if (modbus->pdu.reg >= len)
        return;

    uint16_t val = modbus->pdu.num;
    // val = cover(modbus->pdu.reg) ? UShortCover(modbus->pdu.num) : val;

    addr[modbus->pdu.reg] = val;
}

static void modbus_set_mult(const Modbus *modbus)
{
    {
        Modbus t = {};
        t.addr = deviceMap.regs.id;

        t.pdu.cmd = modbus->pdu.cmd;
        t.pdu.reg = modbus->pdu.reg;
        t.pdu.num = modbus->pdu.num;
        // t.pdu.length = modbus->pdu.length;
        // memcpy(t.pdu.data, modbus->pdu.data, modbus->pdu.length);
        snd(&t);
    }

    uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
    int32_t len = sizeof(RegMap) / sizeof(uint16_t);

    for (uint8_t i = 0; i < modbus->pdu.num; i++)
    {
        uint16_t reg = modbus->pdu.reg + i;
        if (reg >= len)
            continue;

        uint16_t val = MODBUS_TO_UINT16(modbus->pdu.data[2 * i], modbus->pdu.data[2 * i + 1]);
        // val = cover(reg) ? UShortCover(val) : val;
        addr[reg] = val;
    }
}


void flash_sync()
{
    DeviceMap map = *(DeviceMap *)(FLASH_ADDR);
    if ((memcmp(&map.regs, &deviceMap.regs, sizeof(RegMap))) // 比较保持寄存器
        || (map.out != deviceMap.out))                       // 比较输出寄存器
    {

        // LOG_INFO("flash_sync");

        FLASH_Unlock();
        FLASH_ErasePage(FLASH_ADDR);
        FLASH_ProgramWord(FLASH_ADDR, deviceMap.out); // 保存输出寄存器

        uint16_t *addr = (uint16_t *)(&(deviceMap.regs));
        int32_t len = sizeof(RegMap) / sizeof(uint16_t);
        for (int32_t i = 0; i < len; i++) // 保持寄存器
            FLASH_ProgramHalfWord(FLASH_ADDR + 4 + i * 2, addr[i]);

        FLASH_Lock();
    }
}

static void process_modbus(const Modbus *modbus)
{

    switch (modbus->pdu.cmd)
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

static uint16_t out_pin[] = {GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_9, GPIO_Pin_10};

static void process_cmd(uint16_t cmd)
{
    deviceMap.regs.cmd = 0;

    switch (cmd)
    {
    case REG_CMD_RESTORE:
        regRestore(&deviceMap);
        flash_sync();
        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(GPIOA, out_pin[i], Bit_SET);
        vTaskDelay(pdMS_TO_TICKS(200)); // 等待应答发送完成
        NVIC_SystemReset();
        break;
    case REG_CMD_RESTART:
        flash_sync();
        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(GPIOA, out_pin[i], Bit_SET);
        vTaskDelay(pdMS_TO_TICKS(200)); // 等待应答发送完成
        NVIC_SystemReset();
        break;
    default:
        break;
    }
}

void device_task(void *param)
{

    // 关闭所有输出
    for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
        GPIO_WriteBit(GPIOA, out_pin[i], Bit_SET);

    while (1)
    {
        // 处理设备控制
        process_cmd(deviceMap.regs.cmd);

        for (int32_t i = 0; i < MODBUS_REG_OUT_NUMBER; i++)
            GPIO_WriteBit(GPIOA, out_pin[i], (deviceMap.out & (1 << i)) ? Bit_RESET : Bit_SET);

        Modbus modbus;
        if (xQueueReceive(xQueue, &modbus, pdMS_TO_TICKS(2000)) != pdPASS)
        {
            flash_sync();
            continue;
        }

        // 过滤MODBUS帧
        if (modbus.addr && (modbus.addr != deviceMap.regs.id))
            continue;

        // 处理MODBUS帧
        process_modbus(&modbus);
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

    deviceMap = *(DeviceMap *)(FLASH_ADDR);
    if (deviceMap.regs.id > 255)
        regRestore(&deviceMap);

    if ((deviceMap.regs.config & REG_CONFIG_OUTKEEP) == 0)
        deviceMap.out = 0;

    flash_sync();
    xTaskCreate(device_task, DEVICE_TASK_NAME, DEVICE_TASK_STACK_SIZE, NULL, DEVICE_TASK_PRIORITY, NULL);
}