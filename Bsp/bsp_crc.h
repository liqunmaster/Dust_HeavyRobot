#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include <hpm_soc.h>
#include <hpm_clock_drv.h>
#include <hpm_crc_drv.h>


#ifdef __cplusplus
extern "C"
{
    #endif
        uint32_t bsp_crc_calculate(const uint8_t *data, size_t length, uint32_t initial, uint32_t polynomial, uint32_t width, bool reflected);
    #ifdef __cplusplus
}
#endif
