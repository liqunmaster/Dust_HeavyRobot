#include "app_liftup.hpp"

namespace
{
    static_assert(REMOTE_LIFT_ANGLE_RANGE_RAD <= liftup_config::max_relative_angle_rad, "Remote lift range exceeds the lift controller limit");
    constexpr bool motor_present[4]{liftup_config::motor_present_can0,
                                    liftup_config::motor_present_can1,
                                    liftup_config::motor_present_can2,
                                    liftup_config::motor_present_can3};
    constexpr uint32_t period_ms = 1U;
    constexpr uint32_t command_timeout_ms = 100U;
    constexpr uint32_t feedback_timeout_ms = 100U;
    constexpr float direction[4]{liftup_config::motor_direction_can0,
                                 liftup_config::motor_direction_can1,
                                 liftup_config::motor_direction_can2,
                                 liftup_config::motor_direction_can3};

    cubemars motors[4];
    PositionSpeedLoop loops[4];
    float origin_rad[4]{};
    bool has_origin[4]{};
    bool initialized = false;
    LiftMotorStatus motor_status[4]{};
    struct k_spinlock status_lock;
    K_THREAD_STACK_DEFINE(lift_task, 2048);
    struct k_thread lift_thread;
    K_SEM_DEFINE(lift_sem, 0, 1);
    struct k_timer lift_timer;

    void lift_timer_callback(struct k_timer *)
    {
        k_sem_give(&lift_sem);
    }

    void control_step(float dt_s)
    {
        const uint32_t now_ms = k_uptime_get_32();
        for (uint8_t index = 0U; index < 4U; ++index) {
            if (!motor_present[index]) {
                continue;
            }
            const fdcan_device bus = static_cast<fdcan_device>(index);
            FdcanFeedbackTopicData feedback{};
            const bool has_feedback = fdcan_topic_latest_feedback(bus, motors[index].feedback_id(), FdcanMotorKind::cubemars, feedback) == 0;
            const bool feedback_fresh = has_feedback && now_ms - feedback.timestamp_ms < feedback_timeout_ms;
            LiftAngleTopicData request{};
            const bool command_fresh = remote_channel_latest_lift(index, request) == 0 && now_ms - request.timestamp_ms < command_timeout_ms;
            const float target_rad = command_fresh ? request.relative_angle_rad : 0.0F;
            float torque_nm = 0.0F;
            float measured_rad = 0.0F;
            if (feedback_fresh) {
                if (!has_origin[index]) {
                    origin_rad[index] = feedback.angle_rad;
                    has_origin[index] = true;
                }
                measured_rad = direction[index] * (feedback.angle_rad - origin_rad[index]);
                const float measured_speed = direction[index] * feedback.speed_rad_s;
                torque_nm = direction[index] * loops[index].update(target_rad, measured_rad, measured_speed, dt_s);
            } else {
                loops[index].reset();
            }

            int result = 0;
            if (feedback_fresh) {
                result = motors[index].set_mit(0.0F, 0.0F, 0.0F, 0.0F, torque_nm);
                if (result == 0) {
                    fdcan_frame frame{};
                    result = motors[index].build_control_frame(frame);
                    if (result == 0) {
                        result = fdcan_topic_publish_control({bus, frame});
                    }
                }
            }
            const LiftMotorStatus snapshot{feedback_fresh,
                                           has_feedback ? now_ms - feedback.timestamp_ms : UINT32_MAX,
                                           has_feedback ? feedback.valid_count : 0U,
                                           target_rad, measured_rad, torque_nm, result,
                                           0U, 0U, 0U, 0U};
            const k_spinlock_key_t key = k_spin_lock(&status_lock);
            motor_status[index] = snapshot;
            k_spin_unlock(&status_lock, key);
        }
    }

    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&lift_sem, K_FOREVER);
            control_step(static_cast<float>(period_ms) * 0.001F);
        }
    }
}

void app_liftup_init()
{
    if (initialized) {
        return;
    }
    if (liftup_config::max_relative_angle_rad <= 0.0F || liftup_config::motor_angle_max_rad <= 0.0F || liftup_config::motor_angle_max_rad > 12.5F || liftup_config::max_speed_rad_s <= 0.0F || liftup_config::max_speed_rad_s > 50.0F || liftup_config::max_torque_nm <= 0.0F || liftup_config::max_torque_nm > 65.0F) {
        return;
    }
    for (float motor_direction : direction) {
        if (motor_direction != 1.0F && motor_direction != -1.0F) {
            return;
        }
    }

    alg::PidConfig position_pid{};
    position_pid.kp = liftup_config::position_kp;
    position_pid.ki = liftup_config::position_ki;
    position_pid.kd = liftup_config::position_kd;
    position_pid.output_limit = liftup_config::max_speed_rad_s;
    position_pid.dt = 0.001F;

    alg::PidConfig speed_pid{};
    speed_pid.kp = liftup_config::speed_kp;
    speed_pid.ki = liftup_config::speed_ki;
    speed_pid.kd = liftup_config::speed_kd;
    speed_pid.integral_limit = liftup_config::max_torque_nm * 0.5F;
    speed_pid.output_limit = liftup_config::max_torque_nm;
    speed_pid.dt = 0.001F;

    for (uint8_t index = 0U; index < 4U; ++index) {
        if (!motor_present[index]) {
            continue;
        }
        const fdcan_device bus = static_cast<fdcan_device>(index);
        int result = motors[index].init(bus, liftup_config::motor_angle_max_rad, liftup_config::max_speed_rad_s, liftup_config::max_torque_nm);
        if (result != 0) {
            return;
        }
        result = fdcan_port_bind(motors[index]);
        if (result != 0) {
            return;
        }
        loops[index].configure(position_pid, speed_pid);
    }
    for (uint8_t index = 0U; index < 4U; ++index) {
        if (!motor_present[index]) {
            continue;
        }
        const int result = fdcan_port_enable(motors[index]);
        if (result != 0) {
            return;
        }
    }
    initialized = true;
    k_timer_init(&lift_timer, lift_timer_callback, nullptr);
    k_timer_start(&lift_timer, K_MSEC(period_ms), K_MSEC(period_ms));
    k_thread_create(&lift_thread, lift_task, K_THREAD_STACK_SIZEOF(lift_task), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

void app_liftup_set_target(uint8_t can_index, float relative_angle_rad)
{
    if (!initialized) {
        return;
    }
    if (can_index >= 4U || !motor_present[can_index] || !std::isfinite(relative_angle_rad) ||
        std::fabs(relative_angle_rad) > liftup_config::max_relative_angle_rad) {
        return;
    }
    remote_channel_publish_lift(can_index, relative_angle_rad);
}

void app_liftup_get_status(uint8_t can_index, LiftMotorStatus &status)
{
    status = {};
    if (can_index >= 4U || !initialized) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&status_lock);
    status = motor_status[can_index];
    k_spin_unlock(&status_lock, key);
    fdcan_statistics can_stats{};
    bsp_fdcan_get_statistics(static_cast<fdcan_device>(can_index), &can_stats);
    status.can_tx_completed = can_stats.tx_completed;
    status.can_tx_errors = can_stats.tx_errors;
    status.can_rx_received = can_stats.rx_received;
    status.can_rx_dropped = can_stats.rx_dropped;
}
