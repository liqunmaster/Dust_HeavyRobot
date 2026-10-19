#include "app_chassis.hpp"

#include <string.h>

namespace
{
    Chassis chassis;

    K_THREAD_STACK_DEFINE(chassis_task, 2048);

    K_SEM_DEFINE(chassis_sem, 0, 1);
}

void Chassis::MecanumInverse(float vx_m_s, float vy_m_s, float yaw_rad_s, float wheel_rad_s[4])
{
    const float arm = 409.0f / 2 + 234.0f / 2;
    const float scale = 1.0f / 0.075f;
    wheel_rad_s[0] = (vx_m_s - vy_m_s - arm * yaw_rad_s) * scale;
    wheel_rad_s[1] = (vx_m_s + vy_m_s + arm * yaw_rad_s) * scale;
    wheel_rad_s[2] = (vx_m_s + vy_m_s - arm * yaw_rad_s) * scale;
    wheel_rad_s[3] = (vx_m_s - vy_m_s + arm * yaw_rad_s) * scale;
}

void Chassis::MecanumLimitWheelSpeeds(float wheel_rad_s[4], float limit_rad_s)
{
    float largest = 0.0f;
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

void Chassis::TimerCallback(struct k_timer *arg1)
{
    k_sem_give(&chassis_sem);
}

void Chassis::ControlStep(float dt_s)
{
    ChassisSetpoint request{};
    const uint32_t now_ms = k_uptime_get_32();
    const bool command_fresh = chassis_channel_latest_setpoint(request) == 0 && now_ms - request.timestamp_ms < 100;
    ChassisStatus status{};
    status.timestamp_ms = now_ms;
    status.command_fresh = command_fresh;
    if (!command_fresh) {
        chassis_channel_publish_status(status);
        if (offline_command_sent_ && now_ms - last_offline_command_ms_ < 20) {
            return;
        }
        offline_command_sent_ = true;
        last_offline_command_ms_ = now_ms;
        for (int index = 0; index < 4; ++index) {
            speed_loops_[index].Reset();
            (void)motors_[index].SetCurrent(0.0f);
        }
        CanfdFrame frame{};
        if (motors_[0].BuildControlFrame(frame) == 0) {
            motor_topic_publish_control({CANFD_DEVICE_CAN0, frame});
        }
        return;
    }
    bool any_feedback_fresh = false;
    float targets[4]{};
    MecanumInverse(request.vx_m_s, request.vy_m_s, request.yaw_rad_s, targets);
    MecanumLimitWheelSpeeds(targets, (32767.0f * 6.28318530718f / (60.0f * (3591.0f / 187.0f))));

    for (int index = 0; index < 4; ++index) {
        PlatformCanfdSnapshot raw_feedback{};
        MotorFeedbackTopicData feedback{};
        bool has_feedback = false;
        if (platform_canfd_latest(feedback_handles_[index], raw_feedback) == 0 &&
            now_ms - raw_feedback.timestamp_ms < 100) {
            CanfdFrame frame{};
            frame.id = motors_[index].GetFeedbackId();
            frame.id_type = CANFD_ID_STANDARD;
            frame.protocol = CANFD_PROTOCOL_CLASSIC;
            frame.length = raw_feedback.length;
            memcpy(frame.data, raw_feedback.data, raw_feedback.length);
            if (raw_feedback.sequence != feedback_sequences_[index]) {
                feedback_sequences_[index] = raw_feedback.sequence;
                feedback_valid_[index] = motors_[index].ProcessFeedback(frame) == 0;
            }
            const C620Data data = motors_[index].GetData();
            feedback.bus = CANFD_DEVICE_CAN0;
            feedback.id = frame.id;
            feedback.kind = MotorChannelKind::c620;
            feedback.timestamp_ms = raw_feedback.timestamp_ms;
            feedback.speed_rad_s = data.now_omega;
            feedback.angle_rad = data.now_angle;
            feedback.current_a = data.now_current;
            feedback.valid_count = motors_[index].GetFeedbackCount();
            has_feedback = feedback_valid_[index];
        }
        const bool feedback_fresh = has_feedback;
        any_feedback_fresh |= feedback_fresh;
        status.feedback_fresh[index] = feedback_fresh;
        status.wheel_speed_rad_s[index] = feedback.speed_rad_s;
        float current_a = 0.0f;
        if (feedback_fresh) {
            current_a = speed_loops_[index].Update(targets[index], feedback.speed_rad_s, dt_s);
        } else {
            speed_loops_[index].Reset();
        }
        status.command_current_a[index] = current_a;
        (void)motors_[index].SetCurrent(current_a);
    }

    chassis_channel_publish_status(status);
    if (!any_feedback_fresh) {
        if (offline_command_sent_ && now_ms - last_offline_command_ms_ < 20) {
            return;
        }
        offline_command_sent_ = true;
        last_offline_command_ms_ = now_ms;
    } else {
        offline_command_sent_ = false;
    }

    CanfdFrame frame{};
    if (motors_[0].BuildControlFrame(frame) == 0) {
        motor_topic_publish_control({CANFD_DEVICE_CAN0, frame});
    }
}

void Chassis::ThreadEntry(void *context, void *, void *)
{
    static_cast<Chassis *>(context)->Task();
}

void Chassis::Task()
{
    while (1) {
        k_sem_take(&chassis_sem, K_FOREVER);
        ControlStep(0.001f);
    }
}

void Chassis::Init()
{
    if (initialized_) {
        return;
    }

    const C620Id ids[4] = {C620_ID_0x201, C620_ID_0x202, C620_ID_0x203, C620_ID_0x204};
    alg::PIDConfig speed_pid{};
    speed_pid.kp = 3.0f;
    speed_pid.ki = 0.0f;
    speed_pid.kd = 0.0f;
    speed_pid.integral_limit = 20.0f * 0.5f;
    speed_pid.output_limit = 20.0f;
    speed_pid.dt = 0.001f;
    for (int index = 0; index < 4; ++index) {
        int result = motors_[index].Init(CANFD_DEVICE_CAN0, ids[index], (3591.0f / 187.0f));
        if (result != 0) {
            return;
        }
        result = canfd_port_bind(motors_[index]);
        if (result != 0) {
            return;
        }
        result = platform_canfd_subscribe(CANFD_DEVICE_CAN0, motors_[index].GetFeedbackId(),
                                          CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC,
                                          feedback_handles_[index]);
        if (result != 0) {
            return;
        }
        speed_loops_[index].Configure(speed_pid);
        (void)motors_[index].SetCurrent(0.0f);
    }
    initialized_ = true;
    k_timer_init(&chassis_timer_, TimerCallback, nullptr);
    k_timer_start(&chassis_timer_, K_MSEC(1), K_MSEC(1));
    k_thread_create(&chassis_thread_, chassis_task, K_THREAD_STACK_SIZEOF(chassis_task), ThreadEntry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

void Chassis::SetVelocity(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
    if (!initialized_) {
        return;
    }
    remote_channel_publish_chassis(vx_m_s, vy_m_s, yaw_rad_s);
}

void app_chassis_init()
{
    chassis.Init();
}

void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
    chassis.SetVelocity(vx_m_s, vy_m_s, yaw_rad_s);
}
