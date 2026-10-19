#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/sys/byteorder.h>

#include "bsp_canfd.hpp"

class DjiMotor
{
    public:
    struct Feedback
    {
        uint16_t encoder;
        int16_t omega;
        uint16_t current;
        uint8_t auxiliary_0;
        uint8_t auxiliary_1;
    };

    static bool ValidDevice(CanfdDevice device);

    static bool ValidId(int motor_id);

    static uint16_t ReadU16(const uint8_t *data);

    static int16_t SignedValue(uint16_t raw);

    static int ValidateFeedback(const CanfdFrame &frame, uint8_t motor_id);

    static Feedback DecodeFeedback(const uint8_t data[8]);

    static int Set(CanfdDevice device, uint8_t motor_id, int16_t raw);

    static int BuildControlFrame(CanfdDevice device, uint8_t motor_id, CanfdFrame &frame);

    private:
    static void EncodeControl(const int16_t current[4], uint8_t data[8]);
};
