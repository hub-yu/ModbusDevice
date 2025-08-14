#include "modbus.h"

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

int32_t modebus_deserilize(Modbus *modbus, const uint8_t *data, int32_t len)
{
    if (len < 8)
        return 0;

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
        if (crc && (crc != crc16(data, 8 - 2)))
            return -1;

        modbus->addr = data[0];
        modbus->cmd = data[1];
        modbus->reg = MODBUS_TO_UINT16(data[2], data[3]);
        modbus->num = MODBUS_TO_UINT16(data[4], data[5]);
        modbus->length = 0;
        memset(modbus->data, 0, MAX_DATA_LEN);

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
        if (crc && (crc != crc16(data, len_total - 2)))
            return -1;

        modbus->addr = data[0];
        modbus->cmd = data[1];
        modbus->reg = MODBUS_TO_UINT16(data[2], data[3]);
        modbus->num = MODBUS_TO_UINT16(data[4], data[5]);
        modbus->length = data[6];

        memcpy(modbus->data, data + 7, data[6]);

        return len_total;
    }
    break;

    default:
        break;
    }
    return -1;
}

int32_t modebus_serilize(const Modbus *modbus, uint8_t *data)
{
    int32_t length = 0;

    if (modbus->length > MAX_DATA_LEN)
        return 0;

    switch (modbus->cmd)
    {

    case MODBUS_CMD_GET_OUT:
    case MODBUS_CMD_GET_IN:
    case MODBUS_CMD_GET_VAL:
    case MODBUS_CMD_GET_REG:
        data[0] = modbus->addr;
        data[1] = modbus->cmd;
        data[2] = modbus->length;
        memcpy(data + 3, modbus->data, modbus->length);
        length = (modbus->length + 3);
        break;

    case MODBUS_CMD_SET_OUT:
    case MODBUS_CMD_SET_REG:
        data[0] = modbus->addr;
        data[1] = modbus->cmd;
        data[2] = MODBUS_FROM_UINT16_HIGH(modbus->reg);
        data[3] = MODBUS_FROM_UINT16_LOW(modbus->reg);
        data[4] = MODBUS_FROM_UINT16_HIGH(modbus->num);
        data[5] = MODBUS_FROM_UINT16_LOW(modbus->num);
        length = 6;
        break;

    case MODBUS_CMD_SET_OUT_MULT:
    case MODBUS_CMD_SET_REG_MULT:
        data[0] = modbus->addr;
        data[1] = modbus->cmd;
        data[2] = MODBUS_FROM_UINT16_HIGH(modbus->reg);
        data[3] = MODBUS_FROM_UINT16_LOW(modbus->reg);
        data[4] = MODBUS_FROM_UINT16_HIGH(modbus->num);
        data[5] = MODBUS_FROM_UINT16_LOW(modbus->num);
        length = 6;
        if(modbus->from == 255) {
        data[6] = modbus->length;
        memcpy(data + 7, modbus->data, modbus->length);
        length = modbus->length + 7;
        }
        break;

    default:
        break;
    }

    if (length == 0)
        return 0;

    uint16_t crc = crc16(data, length);
    data[length + 1] = MODBUS_FROM_UINT16_HIGH(crc);
    data[length] = MODBUS_FROM_UINT16_LOW(crc);
    return (length + 2);
}
