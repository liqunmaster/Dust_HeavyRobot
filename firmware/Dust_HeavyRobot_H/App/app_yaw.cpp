#include "app_yaw.hpp"

#include <string.h>
#include "type_math.hpp"

#if defined(ROBOT_BOARD_H)

namespace
{
    Yaw yaw;
    K_THREAD_STACK_DEFINE(yaw_stack, 2048);
    K_SEM_DEFINE(yaw_sem, 0, 1);
}

void Yaw::TimerCallback(struct k_timer *)
{
    k_sem_give(&yaw_sem);
}

void Yaw::ThreadEntry(void *context, void *, void *)
{
    static_cast<Yaw *>(context)->Task();
}

float Yaw::RightStickYToAngle(uint16_t raw)
{
    float normalized = (static_cast<float>(raw) - 1024.0f) / 660.0f;
    if (normalized > 1.0f) {
        normalized = 1.0f;
    } else if (normalized < -1.0f) {
        normalized = -1.0f;
    }
    return normalized * math::pi;
}

bool Yaw::UpdateYawFeedback(uint32_t now_ms)
{
    PlatformCanfdSnapshot raw_feedback{};
    if (platform_canfd_latest(feedback_handle_, raw_feedback) != 0 ||
        now_ms - raw_feedback.timestamp_ms >= 100 ||
        raw_feedback.length > canfd_max_data_length) {
        feedback_valid_ = false;
        return false;
    }

    if (raw_feedback.sequence != feedback_sequence_) {
        CanfdFrame frame{};
        frame.id = motor_.GetFeedbackId();
        frame.id_type = CANFD_ID_STANDARD;
        frame.protocol = CANFD_PROTOCOL_CLASSIC;
        frame.length = raw_feedback.length;
        memcpy(frame.data, raw_feedback.data, raw_feedback.length);
        feedback_sequence_ = raw_feedback.sequence;
        feedback_valid_ = motor_.ProcessFeedback(frame) == 0;
    }
    return feedback_valid_;
}

void Yaw::ControlStep()
{
    const uint32_t now_ms = k_uptime_get_32();
    InputSample sample{};
    const bool command_fresh = input_get_sample(RemoteProtocol::dt7, sample) == 0 && now_ms - sample.timestamp_ms < 100;
    const bool feedback_fresh = UpdateYawFeedback(now_ms);

    InsEulerAngles angles{};
    const bool imu_fresh = ins_get_euler_angles(angles) && now_ms - angles.timestamp_ms < 20U;
    const bool active = command_fresh && feedback_fresh && imu_fresh;
    const uint32_t elapsed_ms = last_control_ms_ == 0 ? 1U : now_ms - last_control_ms_;
    const float dt_s = static_cast<float>(elapsed_ms < 1U ? 1U : elapsed_ms > 20U ? 20U : elapsed_ms) * 0.001f;
    last_control_ms_ = now_ms;
    float current = 0.0f;
    float target_angle = 0.0f;
    float target_speed = 0.0f;
    const C620Data data = motor_.GetData();
    if (command_fresh) {
        target_angle = RightStickYToAngle(sample.dt7_data.channel[0]);
    }
    if (active) {
        target_speed = position_pid_.UpdateAngle(target_angle, angles.yaw_rad, dt_s);
        current = speed_pid_.Update(target_speed, data.now_omega, dt_s);
    } else {
        position_pid_.Reset();
        speed_pid_.Reset();
    }
    (void)motor_.SetCurrent(current);

    CanfdFrame frame{};
    if ((active || control_active_ != active || now_ms - last_send_ms_ >= 20U) &&
        motor_.BuildControlFrame(frame) == 0) {
        (void)motor_topic_publish_control({CANFD_DEVICE_CAN1, frame});
        last_send_ms_ = now_ms;
    }
    control_active_ = active;
}

void Yaw::Task()
{
    while (1) {
        k_sem_take(&yaw_sem, K_FOREVER);
        ControlStep();
    }
}

void Yaw::Init()
{
    if (initialized_) {
        return;
    }

    if (motor_.Init(CANFD_DEVICE_CAN1, C620_ID_0x202, (3591.0f / 187.0f)) != 0 ||
        canfd_port_bind(motor_) != 0 ||
        platform_canfd_subscribe(CANFD_DEVICE_CAN1, motor_.GetFeedbackId(),
                                 CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC,
                                 feedback_handle_) != 0) {
        return;
    }

    alg::PIDConfig config{};
    config.kp = 7.5f;
    config.ki = 0.0f;
    config.kd = 0.20f;
    config.output_limit = 20.0f;
    config.dt = 0.001f;
    config.derivative_on_measurement = alg::DFirst::Enable;
    config.derivative_filter_tau = 0.01f;
    position_pid_.Configure(config);

    alg::PIDConfig speed_config{};
    speed_config.kp = 3.75f;
    speed_config.ki = 0.0f;
    speed_config.kd = 0.0f;
    speed_config.output_limit = 20.0f;
    speed_config.dt = 0.001f;
    speed_pid_.Configure(speed_config);
    (void)motor_.SetCurrent(0.0f);

    k_timer_init(&timer_, TimerCallback, nullptr);
    k_timer_start(&timer_, K_MSEC(1), K_MSEC(1));
    k_thread_create(&thread_, yaw_stack, K_THREAD_STACK_SIZEOF(yaw_stack),
                    ThreadEntry, this, nullptr, nullptr,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(&thread_, "yaw_c620");
    initialized_ = true;
}

void app_yaw_init()
{
    yaw.Init();
}

#endif
