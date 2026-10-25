#pragma once

#if defined(ROBOT_BOARD_H)
#include "dji_c620.hpp"
#include "input.hpp"
#include "motor_channel.hpp"
#include "motor_port.hpp"
#include "canfd_bus.hpp"
#include "pid.hpp"
#include "ins.hpp"

class Yaw
{
    public:
    void Init();

    private:
    static void TimerCallback(struct k_timer *timer);
    static void ThreadEntry(void *context, void *, void *);
    void Task();

    float RightStickYToAngle(uint16_t raw);
    bool UpdateYawFeedback(uint32_t now_ms);
    void ControlStep();

    C620 motor_{};
    PlatformCanfdRxHandle feedback_handle_{};
    uint32_t feedback_sequence_ = 0;
    bool feedback_valid_ = false;
    alg::Pid position_pid_{};
    alg::Pid speed_pid_{};
    bool control_active_ = false;
    uint32_t last_control_ms_ = 0;
    uint32_t last_send_ms_ = 0;

    struct k_thread thread_{};
    struct k_timer timer_{};
    bool initialized_ = false;
};

void app_yaw_init();

#endif
