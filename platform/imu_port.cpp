#include "imu_port.hpp"

#include <stdio.h>
#include "bsp_pwm.hpp"

namespace
{
    ImuPort imu_port;

    K_THREAD_STACK_DEFINE(imu_stack, 8192);

    K_SEM_DEFINE(imu_sem, 0, 1);
}

void ImuPort::TimerCallback(struct k_timer *arg1)
{
    k_sem_give(&imu_sem);
}

void ImuPort::StoreSample(const ImuSample &sample)
{
    const k_spinlock_key_t key = k_spin_lock(&state_.lock);
    state_.sample = sample;
    state_.has_sample = true;
    k_spin_unlock(&state_.lock, key);
}

void ImuPort::ProcessSample(const ImuSample &sample)
{
    const uint32_t now_cycles = k_cycle_get_32();
    float dt_seconds = (1.0f / 1000.0f);
    if (have_sample_time_) {
        dt_seconds = static_cast<float>(now_cycles - last_sample_cycles_) / static_cast<float>(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
        if (dt_seconds <= 0.0f || dt_seconds > 0.1f) {
            dt_seconds = (1.0f / 1000.0f);
        }
    }
    last_sample_cycles_ = now_cycles;
    have_sample_time_ = true;

    if (!ins_process(sample.accel_raw[0], sample.accel_raw[1], sample.accel_raw[2], sample.gyro_raw[0], sample.gyro_raw[1], sample.gyro_raw[2], dt_seconds)) {
        return;
    }
}

void ImuPort::UpdateHeater(const ImuSample &sample, uint32_t now_ms)
{
    if (!heater_ready_ || now_ms - last_heater_update_ms_ < 100) {
        return;
    }
    const float dt = static_cast<float>(now_ms - last_heater_update_ms_) * 0.001f;
    last_heater_update_ms_ = now_ms;
    const float temperature_c = 23.0f + static_cast<float>(sample.temperature_raw) / 512.0f;
    if (temperature_c < -40.0f || temperature_c > 85.0f || temperature_c >= 45.0f) {
        heater_integral_ = 0.0f;
        heater_output_ = 0.0f;
        bsp_pwm_heater_set_duty(0.0f);
        return;
    }

    const float error = 40.0f - temperature_c;
    const float proportional = 0.08f * error;
    const float candidate = proportional + heater_integral_;
    if (!((candidate >= 0.95f && error > 0.0f) || (candidate <= 0.0f && error < 0.0f))) {
        heater_integral_ += 0.02f * error * dt;
    }
    const float duty = proportional + heater_integral_;
    heater_output_ = duty < 0.0f ? 0.0f : (duty > 0.95f ? 0.95f : duty);
    bsp_pwm_heater_set_duty(heater_output_);
}

void ImuPort::ThreadEntry(void *context, void *, void *)
{
    static_cast<ImuPort *>(context)->Task();
}

void ImuPort::Task()
{
    while (1) {
        k_sem_take(&imu_sem, K_FOREVER);

        ImuSample sample{};
        if (imu_device_.ReadSample(sample) != 0) {
            if (heater_ready_ && heater_output_ > 0.0f &&
                k_uptime_get_32() - last_valid_sample_ms_ >= 200) {
                heater_integral_ = 0.0f;
                heater_output_ = 0.0f;
                bsp_pwm_heater_set_duty(0.0f);
            }
            continue;
        }

        const uint32_t now_ms = k_uptime_get_32();
        last_valid_sample_ms_ = now_ms;
        StoreSample(sample);
        atomic_inc(&state_.feedback_count);
        ProcessSample(sample);
        UpdateHeater(sample, now_ms);
    }
}

void ImuPort::Init()
{
    const int ret = imu_device_.Init();
    if (ret != 0) {
        return;
    }

    heater_ready_ = bsp_pwm_heater_init() == 0;
    last_valid_sample_ms_ = k_uptime_get_32();
    last_heater_update_ms_ = last_valid_sample_ms_;
    k_thread_create(&imu_thread_, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), ThreadEntry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_timer_init(&imu_timer_, TimerCallback, nullptr);
    k_timer_start(&imu_timer_, K_MSEC(1), K_MSEC(1));
}

int ImuPort::GetSample(ImuSample &sample)
{
    const k_spinlock_key_t key = k_spin_lock(&state_.lock);
    if (!state_.has_sample) {
        k_spin_unlock(&state_.lock, key);
        return -ENODATA;
    }
    sample = state_.sample;
    k_spin_unlock(&state_.lock, key);
    return 0;
}

uint32_t ImuPort::FeedbackCount()
{
    return static_cast<uint32_t>(atomic_get(&state_.feedback_count));
}

void imu_port_init()
{
    imu_port.Init();
}

int imu_port_get_sample(ImuSample &sample)
{
    return imu_port.GetSample(sample);
}

uint32_t imu_port_feedback_count()
{
    return imu_port.FeedbackCount();
}
