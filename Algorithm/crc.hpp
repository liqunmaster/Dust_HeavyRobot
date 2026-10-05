#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bsp_crc.h"

#ifdef __cplusplus
extern "C"
{
    #endif
        unsigned char get_crc8_check_sum(unsigned char *message, unsigned int length, unsigned char crc8);

        unsigned int verify_crc8_check_sum(unsigned char *message, unsigned int length);

        void append_crc8_check_sum(unsigned char *message, unsigned int length);

        uint16_t get_crc16_check_sum(uint8_t *message, uint32_t length, uint16_t crc_value);

        uint32_t verify_crc16_check_sum(uint8_t *message, uint32_t length);

        void append_crc16_check_sum(uint8_t *message, uint32_t length);

        uint16_t crc16_ccitt_false(const uint8_t *data, size_t length);

        uint16_t crc16_dji(const uint8_t *data, size_t length);
    #ifdef __cplusplus
}
#endif
