#ifndef HEXPARSER_H_GUARD
#define HEXPARSER_H_GUARD

#include <stdint.h>
#include <stdbool.h>

#define HEXPARSER_MAX_BYTE_COUNT 0x10

typedef enum
{
    INVALID_RECORD = -1,
    DATA_RECORD = 0,
    END_OF_FILE_RECORD,
    EXTENDED_SEGMENT_ADDRESS_RECORD,
    START_SEGMENT_ADDRESS_RECORD,
    EXTENDED_LINEAR_ADDRESS_RECORD,
    START_LINEAR_ADDRESS_RECORD,
} hexparser_record_type;

typedef struct
{
    uint8_t byte_count;
    uint16_t address;
    hexparser_record_type type;
    union
    {
        uint8_t bytes[HEXPARSER_MAX_BYTE_COUNT];
        uint32_t words[HEXPARSER_MAX_BYTE_COUNT / 4];
    } data;
    uint8_t checksum;
} hexparser_record;

uint32_t hexparser_parse_string(const char * record_string, uint8_t length, hexparser_record * record);
bool hexparser_is_record_valid(hexparser_record * record);






#endif
