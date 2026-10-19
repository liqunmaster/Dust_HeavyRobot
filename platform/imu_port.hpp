#pragma once

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "icm42688p_hxy.hpp"
#include "ins.hpp"
#include "type_math.hpp"

using ImuSample = Icm42688pHxySample;

class ImuPort
{
    public:
    void Init();

    int GetSample(ImuSample &sample);

    uint32_t FeedbackCount();

    private:
    struct ImuState
    {
        struct k_spinlock lock;
        ImuSample sample{};
        atomic_t feedback_count{};
        bool has_sample{false};
    };

    static void TimerCallback(struct k_timer *arg1);

    void StoreSample(const ImuSample &sample);

    void ProcessSample(const ImuSample &sample);

    void UpdateHeater(const ImuSample &sample, uint32_t now_ms);

    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    ImuState state_{};
    Icm42688pHxy imu_device_{};
    struct k_thread imu_thread_{};
    struct k_timer imu_timer_{};
    bool heater_ready_ = false;
    uint32_t last_heater_update_ms_ = 0;
    uint32_t last_valid_sample_ms_ = 0;
    float heater_integral_ = 0.0f;
    float heater_output_ = 0.0f;
    uint32_t last_sample_cycles_ = 0;
    bool have_sample_time_ = false;
};

void imu_port_init();

int imu_port_get_sample(ImuSample &sample);

uint32_t imu_port_feedback_count();
