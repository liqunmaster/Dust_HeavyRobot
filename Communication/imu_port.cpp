#include "imu_port.hpp"

namespace
{
    /**
     * @brief IMU 端口运行状态 保存最新样例与反馈计数
     *
     */
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
    K_SEM_DEFINE(imu_sem, 0, 1);
    struct k_timer imu_timer;
    constexpr float default_dt_seconds = 1.0f / 1000.0f;

    void imu_timer_callback(struct k_timer *)
    {
        k_sem_give(&imu_sem);
    }

    uint32_t last_sample_cycles = 0;
    bool have_sample_time = false;

    /**
     * @brief 在自旋锁保护下保存最新 IMU 采样
     *
     * @param sample 待保存的 IMU 样例
     */
    void store_sample(const imu_sample &sample)
    {
        const k_spinlock_key_t key = k_spin_lock(&state.lock);
        state.sample = sample;
        state.has_sample = true;
        k_spin_unlock(&state.lock, key);
    }

    /**
     * @brief 计算采样间隔并送入惯导解算
     *
     * @param sample 待处理的 IMU 样例
     */
    void process_sample(const imu_sample &sample)
    {
        const uint32_t now_cycles = k_cycle_get_32();
        float dt_seconds = default_dt_seconds;
        if (have_sample_time) {
            dt_seconds = static_cast<float>(now_cycles - last_sample_cycles) / static_cast<float>(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
            if (dt_seconds <= 0.0f || dt_seconds > 0.1f) {
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

/**
 * @brief IMU 读取线程入口：等待数据就绪、读取样例、保存并处理
 *
 */
void imu_thread_entry(void *, void *, void *)
{
    while (true) {
        k_sem_take(&imu_sem, K_FOREVER);

        imu_sample sample{};
        if (imu_device.read_sample(sample) != 0) {
            continue;
        }

        store_sample(sample);
        atomic_inc(&state.feedback_count);
        process_sample(sample);
    }
}

/**
 * @brief 初始化 IMU 设备并启动读取线程
 *
 */
void imu_port_init()
{
    const int ret = imu_device.init();
    if (ret != 0) {
        return;
    }

    k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_timer_init(&imu_timer, imu_timer_callback, nullptr);
    k_timer_start(&imu_timer, K_MSEC(1), K_MSEC(1));
}

/**
 * @brief 读取最新的 IMU 采样；尚无数据时返回错误
 *
 * @param sample 输出最新 IMU 样例
 * @return 成功返回 0 暂无数据返回 -ENODATA
 */
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

/**
 * @brief 获取累计读取并保存的 IMU 样例数量
 *
 * @return 累计样例数量
 */
uint32_t imu_port_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&state.feedback_count));
}

void imu_port_debug_poll()
{
    ins_euler_angles angles{};
    (void)ins_get_euler_angles(angles);
}
