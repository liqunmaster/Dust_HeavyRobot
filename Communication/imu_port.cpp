#include "imu_port.hpp"

namespace
{
    struct imu_state
    {
        struct k_spinlock lock;
        imu_sample sample{};
        atomic_t feedback_count{};
        bool has_sample{false};
    };

    imu_state state{};
    icm42688phxy imu_device{};

    K_THREAD_STACK_DEFINE(imu_stack, 8192);
    struct k_thread imu_thread;
    constexpr float default_dt_seconds = 1.0F / 200.0F;

    uint32_t last_sample_cycles = 0U;
    bool have_sample_time = false;

    void store_sample(const imu_sample &sample)
    {
        const k_spinlock_key_t key = k_spin_lock(&state.lock);
        state.sample = sample;
        state.has_sample = true;
        k_spin_unlock(&state.lock, key);
    }

    void process_sample(const imu_sample &sample)
    {
        const uint32_t now_cycles = k_cycle_get_32();
        float dt_seconds = default_dt_seconds;
        if (have_sample_time) {
            dt_seconds = static_cast<float>(now_cycles - last_sample_cycles) / static_cast<float>(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
            if (dt_seconds <= 0.0F || dt_seconds > 0.1F) {
                dt_seconds = default_dt_seconds;
            }
        }
        last_sample_cycles = now_cycles;
        have_sample_time = true;

        if (!ins_process(sample.accel_raw[0], sample.accel_raw[1], sample.accel_raw[2], sample.gyro_raw[0], sample.gyro_raw[1], sample.gyro_raw[2], dt_seconds)) {
            return;
        }
    }
}

void imu_thread_entry(void *, void *, void *)
{
    while (true) {
        (void)imu_device.wait_data_ready(K_MSEC(20));

        imu_sample sample{};
        if (imu_device.read_sample(sample) != 0) {
            continue;
        }

        store_sample(sample);
        atomic_inc(&state.feedback_count);
        process_sample(sample);
    }
}

void imu_port_init()
{
    const int ret = imu_device.init();
    if (ret != 0) {
        return;
    }

    k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

int imu_port_get_sample(imu_sample &sample)
{
    const k_spinlock_key_t key = k_spin_lock(&state.lock);
    if (!state.has_sample) {
        k_spin_unlock(&state.lock, key);
        return -ENODATA;
    }
    sample = state.sample;
    k_spin_unlock(&state.lock, key);
    return 0;
}

uint32_t imu_port_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&state.feedback_count));
}
