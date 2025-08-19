#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include <string.h> 

#define MODBUS_TO_UINT16(val_h, val_l) ((val_h << 8) | val_l)
#define MODBUS_FROM_UINT16_HIGH(val) (val >> 8)
#define MODBUS_FROM_UINT16_LOW(val) (val & 0xff)

#define MAX_DATA_LEN (64)

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
typedef struct Modbus_PDU
{
    uint8_t cmd;    // command
    uint16_t reg;   // register address
    uint16_t num;   // number of registers or values
    uint8_t length; // data length
    uint8_t data[MAX_DATA_LEN];   // data buffer
} Modbus_PDU;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Modbus_RTU
{
    uint8_t addr;   // slave address
    Modbus_PDU pdu;
    uint16_t crc;   // crc
} Modbus_RTU;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Modbus_ASCII
{
    uint8_t start;  // :
    uint8_t addr;   // slave address
    Modbus_PDU pdu;
    uint8_t lrc;   // lrc
    uint8_t endr;   //  \r
    uint8_t endn;   //  \n
} Modbus_ASCII;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct Modbus_MBAP
{
    uint16_t index;  //
    uint16_t protocol;   // def：0
    uint16_t len;
    uint8_t addr;
    Modbus_PDU pdu;    
} Modbus_MBAP;
#pragma pack(pop)

int32_t modebus_deserilize_pdu(Modbus_PDU *pdu, const uint8_t *data, int32_t len);
int32_t modebus_serilize_pdu(const Modbus_PDU* pdu, uint8_t *data);

int32_t modebus_deserilize_rtu(Modbus_RTU* rtu, const uint8_t *data, int32_t len);
int32_t modebus_serilize_rtu(const Modbus_RTU* rtu, uint8_t *data);

int32_t modebus_deserilize_ascii(Modbus_ASCII* ascii, const uint8_t *data, int32_t len);
int32_t modebus_serilize_ascii(const Modbus_ASCII* ascii, uint8_t *data);

int32_t modebus_deserilize_mbap(Modbus_MBAP* mbap, const uint8_t *data, int32_t len);
int32_t modebus_serilize_mbap(const Modbus_MBAP* mbap, uint8_t *data);

#endif