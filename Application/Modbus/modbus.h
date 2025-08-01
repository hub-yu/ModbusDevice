#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include <string.h> 

#define MODBUS_TO_UINT16(val_h, val_l) ((val_h << 8) | val_l)
#define MODBUS_FROM_UINT16_HIGH(val) (val >> 8)
#define MODBUS_FROM_UINT16_LOW(val) (val & 0xff)

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

#pragma pack(push, 1)
typedef struct Modbus
{
    uint8_t addr;   // slave address
    uint8_t cmd;    // command
    uint16_t reg;   // register address
    uint16_t num;   // number of registers or values
    uint8_t length; // data length
    uint8_t data[20];   // data buffer
    // uint16_t crc;   // crc
} Modbus;
#pragma pack(pop)


int32_t modebus_deserilize(Modbus* modbus_t, const uint8_t *data, int32_t len);
int32_t modebus_serilize(const Modbus* modbus_t, uint8_t *data);


#endif