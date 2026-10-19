#include "motor_health.hpp"

namespace
{
    MotorHealth motor_health;
}

MotorHealth::MotorState MotorHealth::ClassifyState(const CanfdMotorHealthData &motor, uint32_t now_ms, uint32_t last_feedback)
{
    if (motor.kind == MotorChannelKind::cubemars) {
        if (!motor.enable_acknowledged || now_ms - last_feedback >= 100) {
            return MotorState::offline;
        }
        return MotorState::enabled;
    }
    if (now_ms - last_feedback >= 30) {
        return MotorState::offline;
    }
    if (motor.kind == MotorChannelKind::dm) {
        if (motor.status == 0x01) {
            return MotorState::enabled;
        }
        return motor.status >= 0x08 ? MotorState::error : MotorState::disabled;
    }
    return MotorState::enabled;
}

void MotorHealth::CheckMotors(uint32_t now_ms, uint32_t elapsed_ms)
{

    const size_t motor_count = canfd_port_motor_count();
    for (size_t index = 0; index < motor_count; ++index) {
        present_[index] = false;
        motors_[index] = {};
        if (canfd_port_motor_health(index, motors_[index]) != 0) {
            continue;
        }
        present_[index] = true;
        const uint32_t count = motors_[index].valid_feedback_count;
        if (!registered_[index]) {
            registered_[index] = true;
            last_count_[index] = count;
            if (motors_[index].kind == MotorChannelKind::cubemars) {
                atomic_set(&offline_[index], 1);
            }
            last_feedback_ms_[index] = motors_[index].kind == MotorChannelKind::cubemars ? (motors_[index].request_count != 0 && count == 0 ? motors_[index].pending_since_ms : now_ms) : now_ms - 30;
        } else if (count != last_count_[index]) {
            last_count_[index] = count;
            last_feedback_ms_[index] = now_ms;
        }

        const MotorState next = ClassifyState(motors_[index], now_ms, last_feedback_ms_[index]);
        if (next != states_[index]) {
            states_[index] = next;
            recovery_ticks_[index] = 0;
        }
        const bool is_offline = next == MotorState::offline;
        if (atomic_set(&offline_[index], is_offline ? 1 : 0) != (is_offline ? 1 : 0) &&
            motors_[index].kind == MotorChannelKind::cubemars) {
            canfd_port_set_motor_offline(index, is_offline);
        }
    }

    for (size_t index = 0; index < motor_count; ++index) {
        if (!present_[index] || !motors_[index].automatic_recovery || states_[index] == MotorState::enabled || recovery_ticks_[index] != 0) {
            continue;
        }
        const CanfdMotorHealthData &motor = motors_[index];
        if (motor.kind == MotorChannelKind::c610 || motor.kind == MotorChannelKind::c620) {
            continue;
        }
        if (motor.kind == MotorChannelKind::cubemars &&
            now_ms - last_feedback_ms_[index] < 100) {
            continue;
        }
        const bool clear_error = states_[index] == MotorState::error;
        CanfdFrame frame{};
        if (canfd_port_build_recovery_frame(index, clear_error, frame) != 0) {
            continue;
        }
        (void)canfd_port_send_once(motor.bus, frame);
        recovery_ticks_[index] = motor.kind == MotorChannelKind::cubemars ? 50 : 1;
    }

    for (size_t index = 0; index < motor_count; ++index) {
        if (present_[index] && states_[index] != MotorState::enabled &&
            recovery_ticks_[index] != 0) {
            recovery_ticks_[index] = recovery_ticks_[index] > elapsed_ms
                                         ? static_cast<uint16_t>(recovery_ticks_[index] - elapsed_ms)
                                         : 0;
        }
    }
}

void MotorHealth::Init()
{
    if (initialized_) {
        return;
    }
    const int result = canfd_port_init();
    if (result != 0) {
        return;
    }
    initialized_ = true;
}

void MotorHealth::Poll()
{
    if (!initialized_) {
        return;
    }
    const uint32_t now_ms = k_uptime_get_32();
    if (checked_ && now_ms - last_check_ms_ < 10) {
        return;
    }
    const uint32_t elapsed_ms = checked_ ? now_ms - last_check_ms_ : 10;
    checked_ = true;
    last_check_ms_ = now_ms;
    CheckMotors(now_ms, elapsed_ms);
}

bool MotorHealth::IsOffline(size_t index)
{
    return index < CANFD_PORT_MAX_MOTORS && atomic_get(&offline_[index]) != 0;
}

void motor_health_init()
{
    motor_health.Init();
}

void motor_health_poll()
{
    motor_health.Poll();
}

bool motor_health_is_offline(size_t index)
{
    return motor_health.IsOffline(index);
}
