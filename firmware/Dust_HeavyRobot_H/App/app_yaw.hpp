#pragma once

#if defined(ROBOT_BOARD_H)
#include "dji_c620.hpp"
#include "input.hpp"
#include "motor_channel.hpp"
#include "motor_port.hpp"
#include "canfd_bus.hpp"
#include "pid.hpp"
#include "ins.hpp"
struct YawDebugSample
{
    float target_speed;
    float actual_speed;
    float current;
    float target_angle_deg;
    float actual_angle_deg;
    bool command_fresh;
    bool feedback_fresh;
    bool imu_fresh;
};

class Yaw
{
    public:
    void Init();

    void DebugPoll();

    private:
    static void TimerCallback(struct k_timer *timer);
    static void ThreadEntry(void *context, void *, void *);
    void Task();

    static void DebugThreadEntry(void *context, void *, void *);
    void DebugTask();
    float RightStickYToAngle(uint16_t raw);
    bool UpdateYawFeedback(uint32_t now_ms);
    void ControlStep();

    C620 motor_{};
    PlatformCanfdRxHandle feedback_handle_{};
    uint32_t feedback_sequence_ = 0;
    bool feedback_valid_ = false;
    struct k_thread debug_thread_{};
    alg::Pid position_pid_{};
    alg::Pid speed_pid_{};
    bool control_active_ = false;
    uint32_t last_control_ms_ = 0;
    uint32_t last_send_ms_ = 0;

    struct k_thread thread_{};
    struct k_timer timer_{};
    uint32_t last_debug_ms_ = 0;
    bool initialized_ = false;
};

void app_yaw_init();
void app_yaw_debug_poll();

#endif
