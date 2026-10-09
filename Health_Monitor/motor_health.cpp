#include "motor_health.hpp"

namespace
{
    enum class motor_state : uint8_t
    {
        unknown  = 0,
        enabled  = 1,
        disabled = 2,
        error    = 3,
        offline  = 4,
    };

    constexpr uint8_t dm_status_enabled = 0x01U;
    constexpr uint8_t dm_status_error_min = 0x08U;

    uint32_t last_count[FDCAN_PORT_MAX_MOTORS]{};
    uint32_t last_feedback_ms[FDCAN_PORT_MAX_MOTORS]{};
    uint16_t recovery_ticks[FDCAN_PORT_MAX_MOTORS]{};
    bool registered[FDCAN_PORT_MAX_MOTORS]{};
    atomic_t offline[FDCAN_PORT_MAX_MOTORS]{};
    motor_state states[FDCAN_PORT_MAX_MOTORS]{};
    bool initialized = false;
    bool checked = false;
    uint32_t last_check_ms = 0U;

    motor_state classify_state(const FdcanMotorHealthData &motor, uint32_t now_ms, uint32_t last_feedback)
    {
        if (motor.kind == FdcanMotorKind::cubemars) {
            if (!motor.enable_acknowledged || now_ms - last_feedback >= CUBEMARS_FEEDBACK_TIMEOUT_MS) {
                return motor_state::offline;
            }
            return motor_state::enabled;
        }
        if (now_ms - last_feedback >= MOTOR_OFFLINE_TIMEOUT_MS) {
            return motor_state::offline;
        }
        if (motor.kind == FdcanMotorKind::dm) {
            if (motor.status == dm_status_enabled) {
                return motor_state::enabled;
            }
            return motor.status >= dm_status_error_min ? motor_state::error : motor_state::disabled;
        }
        return motor_state::enabled;
    }

    void check_motors(uint32_t now_ms, uint32_t elapsed_ms)
    {
        // CAN interrupts use the interrupted thread's stack on this target.
        // Keep the 32-motor snapshot off the 2 KB main thread stack.
        static FdcanMotorHealthData motors[FDCAN_PORT_MAX_MOTORS];
        static bool present[FDCAN_PORT_MAX_MOTORS];
        const size_t motor_count = fdcan_port_motor_count();
        for (size_t index = 0U; index < motor_count; ++index) {
            present[index] = false;
            motors[index] = {};
            if (fdcan_port_motor_health(index, motors[index]) != 0) {
                continue;
            }
            present[index] = true;
            const uint32_t count = motors[index].valid_feedback_count;
            if (!registered[index]) {
                registered[index] = true;
                last_count[index] = count;
                if (motors[index].kind == FdcanMotorKind::cubemars) {
                    atomic_set(&offline[index], 1);
                }
                last_feedback_ms[index] = motors[index].kind == FdcanMotorKind::cubemars ? (motors[index].request_count != 0U && count == 0U ? motors[index].pending_since_ms : now_ms) : now_ms - MOTOR_OFFLINE_TIMEOUT_MS;
            } else if (count != last_count[index]) {
                last_count[index] = count;
                last_feedback_ms[index] = now_ms;
            }

            const motor_state next = classify_state(motors[index], now_ms, last_feedback_ms[index]);
            if (next != states[index]) {
                states[index] = next;
                recovery_ticks[index] = 0U;
            }
            const bool is_offline = next == motor_state::offline;
            if (atomic_set(&offline[index], is_offline ? 1 : 0) != (is_offline ? 1 : 0) &&
                motors[index].kind == FdcanMotorKind::cubemars) {
                fdcan_port_set_motor_offline(index, is_offline);
            }
        }

        for (size_t index = 0U; index < motor_count; ++index) {
            if (!present[index] || !motors[index].automatic_recovery || states[index] == motor_state::enabled || recovery_ticks[index] != 0U) {
                continue;
            }
            const FdcanMotorHealthData &motor = motors[index];
            if (motor.kind == FdcanMotorKind::c610 || motor.kind == FdcanMotorKind::c620) {
                continue;
            }
            if (motor.kind == FdcanMotorKind::cubemars &&
                now_ms - last_feedback_ms[index] < CUBEMARS_FEEDBACK_TIMEOUT_MS) {
                continue;
            }
            const bool clear_error = states[index] == motor_state::error;
            fdcan_frame frame{};
            if (fdcan_port_build_recovery_frame(index, clear_error, frame) != 0) {
                continue;
            }
            (void)fdcan_port_send_once(motor.bus, frame);
            recovery_ticks[index] = motor.kind == FdcanMotorKind::cubemars ? CUBEMARS_RECOVERY_PERIOD_MS : MOTOR_RECOVERY_PERIOD_MS;
        }

        for (size_t index = 0U; index < motor_count; ++index) {
            if (present[index] && states[index] != motor_state::enabled &&
                recovery_ticks[index] != 0U) {
                recovery_ticks[index] = recovery_ticks[index] > elapsed_ms
                    ? static_cast<uint16_t>(recovery_ticks[index] - elapsed_ms) : 0U;
            }
        }
    }
}

void motor_health_init()
{
    if (initialized) {
        return;
    }
    if (MOTOR_CHECK_PERIOD_MS == 0U || MOTOR_OFFLINE_TIMEOUT_MS == 0U ||  CUBEMARS_FEEDBACK_TIMEOUT_MS == 0U || MOTOR_RECOVERY_PERIOD_MS == 0U || CUBEMARS_RECOVERY_PERIOD_MS == 0U) {
        return;
    }
    const int result = fdcan_port_init();
    if (result != 0) {
        return;
    }
    initialized = true;
}

void motor_health_poll()
{
    if (!initialized) {
        return;
    }
    const uint32_t now_ms = k_uptime_get_32();
    if (checked && now_ms - last_check_ms < MOTOR_CHECK_PERIOD_MS) {
        return;
    }
    const uint32_t elapsed_ms = checked ? now_ms - last_check_ms : MOTOR_CHECK_PERIOD_MS;
    checked = true;
    last_check_ms = now_ms;
    check_motors(now_ms, elapsed_ms);
}

bool motor_health_is_offline(size_t index)
{
    return index < FDCAN_PORT_MAX_MOTORS && atomic_get(&offline[index]) != 0;
}
