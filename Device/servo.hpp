#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>

enum class servo_output : uint8_t
{
    p2 = 2U,
    p3 = 3U,
};

class servo final
{
    public:
    explicit servo(servo_output output) : output_(output) {}

    int init();

    int set_pulse_us(uint16_t pulse_us);

    int set_angle(uint16_t degrees);

    private:
    servo_output output_;

    bool initialized_ = false;
};
