#include "hexparser.h"

//#include "nrf_error.h"

#include <stdio.h>
#include <stdint.h>

uint8_t nibble_from_hex_char(char c)
{
    switch (c)
    {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            return c - '0';

        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            return c - 'a' + 10;

        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
            return c - 'A' + 10;

    }
    return 0;
}

static uint8_t uint8_from_hex_char(const char c[2])
{
    return nibble_from_hex_char(c[0]) << 4 | 
           nibble_from_hex_char(c[1]);
}

static uint16_t uint16_from_hex_char(const char c[4])
{
    return nibble_from_hex_char(c[0]) << 12 | 
           nibble_from_hex_char(c[1]) << 8 | 
           nibble_from_hex_char(c[2]) << 4 | 
           nibble_from_hex_char(c[3]); 
}

static uint32_t data_uint32_from_hex_char(const char c[4])
{
    return nibble_from_hex_char(c[7]) << 24 | 
           nibble_from_hex_char(c[6]) << 28 | 
           nibble_from_hex_char(c[5]) << 16 | 
           nibble_from_hex_char(c[4]) << 20 |
           nibble_from_hex_char(c[3]) << 8 | 
           nibble_from_hex_char(c[2]) << 12 | 
           nibble_from_hex_char(c[1]) << 0 | 
           nibble_from_hex_char(c[0]) << 4; 
}

uint32_t hexparser_parse_string(const char * record_string, uint8_t length, hexparser_record * record)
{
    record->type = uint8_from_hex_char(&record_string[7]);
    record->address = uint16_from_hex_char(&record_string[3]);
    record->byte_count = uint8_from_hex_char(&record_string[1]);

    record->checksum = uint8_from_hex_char(&record_string[9+(record->byte_count*2)]);

    switch (record->type)
    {
        case DATA_RECORD:
            for (uint8_t i = 0; i < (record->byte_count / 4); i++) 
            {
                record->data.words[i] = data_uint32_from_hex_char(&record_string[9+i*8]);
            }
            break;

        case EXTENDED_LINEAR_ADDRESS_RECORD:
        case EXTENDED_SEGMENT_ADDRESS_RECORD:
            record->data.words[0] = uint16_from_hex_char(&record_string[9]);
            break;

        case START_SEGMENT_ADDRESS_RECORD:
            for (uint8_t i = 0; i < (record->byte_count / 4); i++) 
            {
                record->data.words[i] = data_uint32_from_hex_char(&record_string[9+i*8]);
            }
            break;

        default:
            break;
    }

    return 0;//NRF_SUCCESS;
}

bool hexparser_is_record_valid(hexparser_record * record)
{
    uint8_t sum = 0;
    sum += record->byte_count;
    sum += record->address & 0x00FF;
    sum += (record->address & 0xFF00) >> 8;
    sum += record->type;

    for (uint8_t i = 0; i < record->byte_count; i++)
    {
        sum += record->data.bytes[i];
    }

    return (record->checksum == ((0x100 - sum) & 0xFF));
}
