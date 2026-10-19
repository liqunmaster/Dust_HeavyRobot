#pragma once

#include "mcu.hpp"

class BoardInput
{
    public:
    void PublishChassis(const McuChassisCommand &value, uint32_t timestamp_ms);
    void PublishCommand(const McuCommandCommand &value, uint32_t timestamp_ms);

    private:
    McuChassisCommand chassis_{1024, 1024, 1024, 0};
    McuCommandCommand command_{0, 0, 0.0f};
    uint32_t chassis_timestamp_ms_ = 0;
    uint32_t command_timestamp_ms_ = 0;
};
