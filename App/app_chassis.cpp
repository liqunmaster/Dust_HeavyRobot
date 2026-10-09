#include "app_chassis.hpp"

namespace
{
    struct MecanumGeometry
    {
        float wheel_radius_m;
        float half_length_m;
        float half_width_m;
    };

    // Wheel order: front left, front right, rear left, rear right.
    void mecanum_inverse(const MecanumGeometry &geometry, float vx_m_s, float vy_m_s, float yaw_rad_s, float wheel_rad_s[4])
    {
        const float arm = geometry.half_length_m + geometry.half_width_m;
        const float scale = 1.0F / geometry.wheel_radius_m;
        wheel_rad_s[0] = (vx_m_s - vy_m_s - arm * yaw_rad_s) * scale;
        wheel_rad_s[1] = (vx_m_s + vy_m_s + arm * yaw_rad_s) * scale;
        wheel_rad_s[2] = (vx_m_s + vy_m_s - arm * yaw_rad_s) * scale;
        wheel_rad_s[3] = (vx_m_s - vy_m_s + arm * yaw_rad_s) * scale;
    }

    void mecanum_limit_wheel_speeds(float wheel_rad_s[4], float limit_rad_s)
    {
        float largest = 0.0F;
        for (int index = 0; index < 4; ++index) {
            largest = std::fmax(largest, std::fabs(wheel_rad_s[index]));
        }
        if (largest > limit_rad_s) {
            const float scale = limit_rad_s / largest;
            for (int index = 0; index < 4; ++index) {
                wheel_rad_s[index] *= scale;
            }
        }
    }

    constexpr float gear_ratio   = 3591.0f / 187.0f;
    constexpr uint32_t period_ms = 1;
    constexpr uint32_t command_timeout_ms  = 100;
    constexpr uint32_t feedback_timeout_ms = 100;
    constexpr uint32_t offline_command_period_ms = 20;

    c620 motors[4];
    SpeedLoop speed_loops[4];
    constexpr MecanumGeometry geometry{chassis_config::wheel_radius_m, 409.0f / 2, 234.0f / 2};
    constexpr float wheel_direction[4]{chassis_config::wheel_direction_fl, chassis_config::wheel_direction_fr, chassis_config::wheel_direction_rl, chassis_config::wheel_direction_rr};
    K_THREAD_STACK_DEFINE(chassis_task, 2048);
    struct k_thread chassis_thread;
    K_SEM_DEFINE(chassis_sem, 0, 1);
    struct k_timer chassis_timer;
    bool initialized = false;
    bool offline_command_sent = false;
    uint32_t last_offline_command_ms = 0U;

    void chassis_timer_callback(struct k_timer *)
    {
        k_sem_give(&chassis_sem);
    }

    void control_step(float dt_s)
    {
        ChassisVelocityTopicData request{};
        const uint32_t now_ms = k_uptime_get_32();
        const bool command_fresh = remote_channel_latest_chassis(request) == 0 && now_ms - request.timestamp_ms < command_timeout_ms;
        if (!command_fresh) {
            if (offline_command_sent && now_ms - last_offline_command_ms < offline_command_period_ms) {
                return;
            }
            offline_command_sent = true;
            last_offline_command_ms = now_ms;
            for (int index = 0; index < 4; ++index) {
                speed_loops[index].reset();
                (void)motors[index].set_current(0.0F);
            }
            fdcan_frame frame{};
            if (motors[0].build_control_frame(frame) == 0) {
                fdcan_topic_publish_control({FDCAN_DEVICE_CAN0, frame});
            }
            return;
        }
        bool any_feedback_fresh = false;
        float targets[4]{};
        mecanum_inverse(geometry, request.vx_m_s, request.vy_m_s, request.yaw_rad_s, targets);
        mecanum_limit_wheel_speeds(targets, chassis_config::max_wheel_rad_s);

        FdcanFeedbackKey feedback_keys[4]{};
        FdcanFeedbackTopicData feedback[4]{};
        bool has_feedback[4]{};
        for (int index = 0; index < 4; ++index) {
            feedback_keys[index] = {FDCAN_DEVICE_CAN0, motors[index].feedback_id(), FdcanMotorKind::c620};
        }
        fdcan_topic_latest_feedback_batch(feedback_keys, 4U, feedback, has_feedback);
        for (int index = 0; index < 4; ++index) {
            const bool feedback_fresh = has_feedback[index] && now_ms - feedback[index].timestamp_ms < feedback_timeout_ms;
            any_feedback_fresh |= feedback_fresh;
            float current_a = 0.0f;
            if (feedback_fresh) {
                current_a = speed_loops[index].update(wheel_direction[index] * targets[index], feedback[index].speed_rad_s, dt_s);
            } else {
                speed_loops[index].reset();
            }
            (void)motors[index].set_current(current_a);
        }

        if (!any_feedback_fresh) {
            if (offline_command_sent && now_ms - last_offline_command_ms < offline_command_period_ms) {
                return;
            }
            offline_command_sent = true;
            last_offline_command_ms = now_ms;
        } else {
            offline_command_sent = false;
        }

        fdcan_frame frame{};
        if (motors[0].build_control_frame(frame) == 0) {
            fdcan_topic_publish_control({FDCAN_DEVICE_CAN0, frame});
        }
    }

    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&chassis_sem, K_FOREVER);
            control_step(static_cast<float>(period_ms) * 0.001f);
        }
    }
}

void app_chassis_init()
{
    if (initialized) {
        return;
    }
    for (float direction : wheel_direction) {
        if (direction != 1.0F && direction != -1.0F) {
            return;
        }
    }

    const C620_ID ids[4] = {C620_ID_0x201, C620_ID_0x202, C620_ID_0x203, C620_ID_0x204};
    alg::PidConfig speed_pid{};
    speed_pid.kp = 3.0f;
    speed_pid.ki = 0.0f;
    speed_pid.kd = 0.0f;
    speed_pid.integral_limit = 20.0f * 0.5F;
    speed_pid.output_limit   = 20.0f;
    speed_pid.dt = 0.001F;
    for (int index = 0; index < 4; ++index) {
        int result = motors[index].init(FDCAN_DEVICE_CAN0, ids[index], gear_ratio);
        if (result != 0) {
            return;
        }
        result = fdcan_port_bind(motors[index]);
        if (result != 0) {
            return;
        }
        speed_loops[index].configure(speed_pid);
        (void)motors[index].set_current(0.0F);
    }
    initialized = true;
    k_timer_init(&chassis_timer, chassis_timer_callback, nullptr);
    k_timer_start(&chassis_timer, K_MSEC(period_ms), K_MSEC(period_ms));
    k_thread_create(&chassis_thread, chassis_task, K_THREAD_STACK_SIZEOF(chassis_task), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
    if (!initialized) {
        return;
    }
    remote_channel_publish_chassis(vx_m_s, vy_m_s, yaw_rad_s);
}
