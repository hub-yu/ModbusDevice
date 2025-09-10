#include "modbus.h"
#include <ctype.h>

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

static uint8_t calc_lrc(const uint8_t *data, size_t length)
{
    uint8_t lrc = 0;
    for (size_t i = 0; i < length; i++)
        lrc += data[i];
    return (~lrc + 1) & 0xff;
}

int32_t modebus_deserilize_pdu(Modbus_PDU *pdu, const uint8_t *data, int32_t len)
{
    if (len < 5)
        return 0;

    switch (data[0])
    {
    case MODBUS_CMD_GET_OUT:
    case MODBUS_CMD_GET_IN:
    case MODBUS_CMD_GET_REG:
    case MODBUS_CMD_GET_VAL:
    case MODBUS_CMD_SET_OUT:
    case MODBUS_CMD_SET_REG:
        pdu->cmd = data[0];
        pdu->reg = MODBUS_TO_UINT16(data[1], data[2]);
        pdu->num = MODBUS_TO_UINT16(data[3], data[4]);
        pdu->length = 0;
        memset(pdu->data, 0, MAX_DATA_LEN);
        return 5;

    case MODBUS_CMD_SET_OUT_MULT:
    case MODBUS_CMD_SET_REG_MULT:
    {
        uint8_t len_total = data[5] + 6;
        if (len < len_total)
            return 0;

        pdu->cmd = data[0];
        pdu->reg = MODBUS_TO_UINT16(data[1], data[2]);
        pdu->num = MODBUS_TO_UINT16(data[3], data[4]);
        pdu->length = data[5];

        memcpy(pdu->data, data + 6, pdu->length);

        return len_total;
    }
    break;
    default:
        break;
    }
    return -1;
}

int32_t modebus_serilize_pdu(const Modbus_PDU *pdu, uint8_t *data)
{
    int32_t length = 0;

    if (pdu->length > MAX_DATA_LEN)
        return 0;

    switch (pdu->cmd)
    {

    case MODBUS_CMD_GET_OUT:
    case MODBUS_CMD_GET_IN:
    case MODBUS_CMD_GET_VAL:
    case MODBUS_CMD_GET_REG:
        data[0] = pdu->cmd;
        data[1] = pdu->length;
        memcpy(data + 2, pdu->data, pdu->length);
        length = (2 + pdu->length);
        break;

    case MODBUS_CMD_SET_OUT:
    case MODBUS_CMD_SET_REG:
        data[0] = pdu->cmd;
        data[1] = MODBUS_FROM_UINT16_HIGH(pdu->reg);
        data[2] = MODBUS_FROM_UINT16_LOW(pdu->reg);
        data[3] = MODBUS_FROM_UINT16_HIGH(pdu->num);
        data[4] = MODBUS_FROM_UINT16_LOW(pdu->num);
        length = 5;
        break;

    case MODBUS_CMD_SET_OUT_MULT:
    case MODBUS_CMD_SET_REG_MULT:
        data[0] = pdu->cmd;
        data[1] = MODBUS_FROM_UINT16_HIGH(pdu->reg);
        data[2] = MODBUS_FROM_UINT16_LOW(pdu->reg);
        data[3] = MODBUS_FROM_UINT16_HIGH(pdu->num);
        data[4] = MODBUS_FROM_UINT16_LOW(pdu->num);
        length = 5;

        if (pdu->length)
        {
            data[5] = pdu->length;
            memcpy(data + 6, pdu->data, pdu->length);
            length = (6 + pdu->length);
        }
        break;

    default:
        break;
    }
    return length;
}

int32_t modebus_deserilize_rtu(Modbus_RTU *rtu, const uint8_t *data, int32_t len)
{
    if (len < 8)
        return 0;

    int32_t ret = modebus_deserilize_pdu(&(rtu->pdu), data + 1, len - 1);
    if (ret <= 0)
        return ret;

    if (len < (ret + 3))
        return 0;

    rtu->addr = data[0];
    rtu->crc = MODBUS_TO_UINT16(data[ret + 2], data[ret + 1]);
    // if (rtu->crc && (rtu->crc != crc16(data, ret + 1)))
    //     return -1;
    if (rtu->crc != crc16(data, ret + 1))
        return -1;
    return (ret + 3);
}

int32_t modebus_serilize_rtu(const Modbus_RTU *rtu, uint8_t *data)
{

    if (rtu->pdu.length > MAX_DATA_LEN)
        return 0;

    int32_t ret = modebus_serilize_pdu(&rtu->pdu, data + 1);
    if (ret <= 0)
        return ret;

    data[0] = rtu->addr;
    uint16_t crc = rtu->crc ? rtu->crc : crc16(data, ret + 1);
    data[ret + 2] = MODBUS_FROM_UINT16_HIGH(crc);
    data[ret + 1] = MODBUS_FROM_UINT16_LOW(crc);
    return (ret + 3);
}

int asciiToHex(char c)
{
    c = toupper(c); // 统一转为大写处理
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else if (c >= 'A' && c <= 'F')
    {
        return 10 + (c - 'A');
    }
    return -1; // 非法字符返回-1
}

char hexToAscii(int val, int lowercase)
{
    if (val >= 0 && val <= 9)
    {
        return '0' + val;
    }
    else if (val >= 10 && val <= 15)
    {
        return (lowercase ? 'a' : 'A') + (val - 10);
    }
    return '\0'; // 非法值返回空字符
}

static int32_t ascii_to_byte(const uint8_t *ascii, int32_t ascii_len, uint8_t *byte)
{
    int32_t i = 0;
    for (; i < (ascii_len / 2); i++)
    {
        byte[i] = (((uint8_t)asciiToHex((char)ascii[i * 2])) << 4) | (((uint8_t)asciiToHex((char)ascii[i * 2 + 1])) << 0);
    }
    return i;
}

static int32_t byte_to_ascii(const uint8_t *byte, int32_t byte_len, uint8_t *ascii)
{
    int32_t i = 0;
    for (; i < byte_len; i++)
    {
        ascii[2 * i] = hexToAscii(byte[i] >> 4, 0);
        ascii[2 * i + 1] = hexToAscii(byte[i] & 0xf, 0);
    }
    return 2 * byte_len;
}

int32_t modebus_deserilize_ascii(Modbus_ASCII *ascii, const uint8_t *data, int32_t len)
{

    if (len < 17)
        return 0;

    ascii->start = data[0];
    if (ascii->start != 0x3a)
        return -1;

    uint8_t byte[MAX_DATA_LEN * 2] = {};
    int32_t byte_len = ascii_to_byte(data + 1, len - 1, byte);

    int32_t ret = modebus_deserilize_pdu(&(ascii->pdu), byte + 1, len - 2);
    if (ret <= 0)
        return ret;

    if (len < (2 * ret + 7))
        return 0;

    ascii->endr = data[2 * ret + 5];
    ascii->endn = data[2 * ret + 6];
    ascii->lrc = byte[byte_len - 2];

    if (ascii->endr != '\r')
        return -1;
    if (ascii->endn != '\n')
        return -1;
    if (ascii->lrc != calc_lrc(byte, ret + 1))
        return -1;

    return len;
}

int32_t modebus_serilize_ascii(const Modbus_ASCII *ascii, uint8_t *data)
{
    if (ascii->pdu.length > MAX_DATA_LEN)
        return 0;

    uint8_t byte[MAX_DATA_LEN * 2] = {};
    int32_t ret = modebus_serilize_pdu(&ascii->pdu, byte + 1);
    if (ret <= 0)
        return ret;

    byte[0] = ascii->addr;
    byte[ret + 1] = ascii->lrc ? ascii->lrc : calc_lrc(byte, ret + 1);

    int32_t data_len = byte_to_ascii(byte, ret + 2, data + 1);

    data[0] = ascii->start ? ascii->start : ':';
    data[1 + data_len] = ascii->endr ? ascii->endr : '\r';
    data[2 + data_len] = ascii->endn ? ascii->endn : '\n';

    return (data_len + 3);
}

int32_t modebus_deserilize_mbap(Modbus_MBAP *mbap, const uint8_t *data, int32_t len)
{
    if (len < 6)
        return 0;

    mbap->index = MODBUS_TO_UINT16(data[0], data[1]);
    mbap->protocol = MODBUS_TO_UINT16(data[2], data[3]);
    mbap->len = MODBUS_TO_UINT16(data[4], data[5]);

    if (mbap->protocol != 0)
        return -1;

    if (len < (mbap->len + 6))
        return 0;

    int32_t ret = modebus_deserilize_pdu(&mbap->pdu, data + 7, len - 7);
    if (ret <= 0)
        return ret;

    return (ret + 7);
}

int32_t modebus_serilize_mbap(const Modbus_MBAP *mbap, uint8_t *data)
{

    if (mbap->pdu.length > MAX_DATA_LEN)
        return 0;

    int32_t ret = modebus_serilize_pdu(&mbap->pdu, data + 7);
    if (ret <= 0)
        return ret;

    data[0] = MODBUS_FROM_UINT16_HIGH(mbap->index);
    data[1] = MODBUS_FROM_UINT16_LOW(mbap->index);
    data[2] = MODBUS_FROM_UINT16_HIGH(mbap->protocol);
    data[3] = MODBUS_FROM_UINT16_LOW(mbap->protocol);
    data[4] = MODBUS_FROM_UINT16_HIGH(mbap->len ? mbap->len : (ret + 1));
    data[5] = MODBUS_FROM_UINT16_LOW(mbap->len ? mbap->len : (ret + 1));
    data[6] = mbap->addr;

    return (ret + 7);
}