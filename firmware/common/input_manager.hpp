#pragma once

#include "input_parser.hpp"

class InputManager
{
    public:
    void PublishSample(const InputSample &sample);
    void PublishMcu(uint16_t chassis_x, uint16_t chassis_y, uint16_t rotation,
                    uint8_t flags, uint8_t mouse_lr, uint16_t keyboard, uint32_t timestamp_ms);

    private:
    float Axis(uint16_t raw);
};

void input_manager_publish_sample(const InputSample &sample);
void input_manager_publish_mcu(uint16_t chassis_x, uint16_t chassis_y, uint16_t rotation,
                               uint8_t flags, uint8_t mouse_lr, uint16_t keyboard, uint32_t timestamp_ms);
