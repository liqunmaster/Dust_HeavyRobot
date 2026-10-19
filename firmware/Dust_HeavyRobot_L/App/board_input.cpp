#include "board_input.hpp"

#include <zephyr/kernel.h>
#include "input_manager.hpp"

void BoardInput::PublishChassis(const McuChassisCommand &value, uint32_t timestamp_ms)
{
    chassis_ = value;
    chassis_timestamp_ms_ = timestamp_ms;
    const bool command_fresh = k_uptime_get_32() - command_timestamp_ms_ < 30U;
    input_manager_publish_mcu(chassis_.x, chassis_.y, chassis_.rotation, chassis_.flags,
                               command_fresh ? command_.mouse_lr : 0,
                               command_fresh ? command_.keyboard : 0, timestamp_ms);
}

void BoardInput::PublishCommand(const McuCommandCommand &value, uint32_t timestamp_ms)
{
    command_ = value;
    command_timestamp_ms_ = timestamp_ms;
    const bool chassis_fresh = k_uptime_get_32() - chassis_timestamp_ms_ < 30U;
    input_manager_publish_mcu(chassis_fresh ? chassis_.x : 1024,
                               chassis_fresh ? chassis_.y : 1024,
                               chassis_fresh ? chassis_.rotation : 1024,
                               chassis_fresh ? chassis_.flags : 0,
                               command_.mouse_lr, command_.keyboard,
                               chassis_timestamp_ms_);
}

