#include "app_liftup.hpp"

namespace
{
    static_assert(REMOTE_LIFT_ANGLE_RANGE_RAD <= liftup_config::max_relative_angle_rad, "Remote lift range exceeds the lift controller limit");
    constexpr bool motor_present[4]{liftup_config::motor_present_can0,
                                    liftup_config::motor_present_can1,
                                    liftup_config::motor_present_can2,
                                    liftup_config::motor_present_can3};
    constexpr uint32_t period_ms = 1;
    constexpr uint32_t command_timeout_ms = 100;
    constexpr uint32_t feedback_timeout_ms = 100;
    constexpr float direction[4]{liftup_config::motor_direction_can0,
                                 liftup_config::motor_direction_can1,
                                 liftup_config::motor_direction_can2,
                                 liftup_config::motor_direction_can3};

    cubemars motors[4];
    position_speed_loop loops[4];
    float origin_rad[4]{};
    bool has_origin[4]{};
    bool initialized = false;
    lift_motor_status motor_status[4]{};
    struct k_spinlock status_lock;
    K_THREAD_STACK_DEFINE(lift_task, 2048);
    struct k_thread lift_thread;
    K_SEM_DEFINE(lift_sem, 0, 1);
    struct k_timer lift_timer;

    /**
     * @brief 抬升定时器回调 唤醒控制线程
     *
     * @param timer 触发回调的定时器
     */
    void lift_timer_callback(struct k_timer *)
    {
        k_sem_give(&lift_sem);
    }

    /**
     * @brief 抬升单次控制步进 对四路电机执行位置控制
     *
     * @param dt_s 控制周期
     */
    void control_step(float dt_s)
    {
        const uint32_t now_ms = k_uptime_get_32();
        for (uint8_t index = 0; index < 4; ++index) {
            if (!motor_present[index]) {
                continue;
            }
            const fdcan_device bus = static_cast<fdcan_device>(index);
            fdcan_feedback_topic_data feedback{};
            const bool has_feedback = fdcan_topic_latest_feedback(bus, motors[index].feedback_id(), fdcan_motor_kind::cubemars, feedback) == 0;
            const bool feedback_fresh = has_feedback && now_ms - feedback.timestamp_ms < feedback_timeout_ms;
            lift_angle_topic_data request{};
            const bool command_fresh = remote_channel_latest_lift(index, request) == 0 && now_ms - request.timestamp_ms < command_timeout_ms;
            const float target_rad = command_fresh ? request.relative_angle_rad : 0.0f;
            float torque_nm = 0.0f;
            float measured_rad = 0.0f;
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
                result = motors[index].set_mit(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
                if (result == 0) {
                    fdcan_frame frame{};
                    result = motors[index].build_control_frame(frame);
                    if (result == 0) {
                        result = fdcan_topic_publish_control({bus, frame});
                    }
                }
            }
            const lift_motor_status snapshot{feedback_fresh,
                                           has_feedback ? now_ms - feedback.timestamp_ms : UINT32_MAX,
                                           has_feedback ? feedback.valid_count : 0,
                                           target_rad, measured_rad, torque_nm, result,
                                           0, 0, 0, 0};
            const k_spinlock_key_t key = k_spin_lock(&status_lock);
            motor_status[index] = snapshot;
            k_spin_unlock(&status_lock, key);
        }
    }

    /**
     * @brief 抬升控制线程入口 等待信号量并循环执行控制步进
     *
     * @param arg1 线程参数 1
     * @param arg2 线程参数 2
     * @param arg3 线程参数 3
     */
    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&lift_sem, K_FOREVER);
            control_step(static_cast<float>(period_ms) * 0.001f);
        }
    }
}

/**
 * @brief 初始化抬升模块 配置四路电机及位置-速度环并启动控制线程
 *
 */
void app_liftup_init()
{
    if (initialized) {
        return;
    }
    if (liftup_config::max_relative_angle_rad <= 0.0f || liftup_config::motor_angle_max_rad <= 0.0f || liftup_config::motor_angle_max_rad > 12.5f || liftup_config::max_speed_rad_s <= 0.0f || liftup_config::max_speed_rad_s > 50.0f || liftup_config::max_torque_nm <= 0.0f || liftup_config::max_torque_nm > 65.0f) {
        return;
    }
    for (float motor_direction : direction) {
        if (motor_direction != 1.0f && motor_direction != -1.0f) {
            return;
        }
    }

    alg::pid_config position_pid{};
    position_pid.kp = liftup_config::position_kp;
    position_pid.ki = liftup_config::position_ki;
    position_pid.kd = liftup_config::position_kd;
    position_pid.output_limit = liftup_config::max_speed_rad_s;
    position_pid.dt = 0.001f;

    alg::pid_config speed_pid{};
    speed_pid.kp = liftup_config::speed_kp;
    speed_pid.ki = liftup_config::speed_ki;
    speed_pid.kd = liftup_config::speed_kd;
    speed_pid.integral_limit = liftup_config::max_torque_nm * 0.5f;
    speed_pid.output_limit = liftup_config::max_torque_nm;
    speed_pid.dt = 0.001f;

    for (uint8_t index = 0; index < 4; ++index) {
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
    for (uint8_t index = 0; index < 4; ++index) {
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

/**
 * @brief 设置单个抬升电机的目标相对角度并发布给控制线程
 *
 * @param can_index 目标电机所在的 CAN 通道索引
 * @param relative_angle_rad 相对原点角度
 */
void app_liftup_set_target(uint8_t can_index, float relative_angle_rad)
{
    if (!initialized) {
        return;
    }
    if (can_index >= 4 || !motor_present[can_index] || !std::isfinite(relative_angle_rad) ||
        std::fabs(relative_angle_rad) > liftup_config::max_relative_angle_rad) {
        return;
    }
    remote_channel_publish_lift(can_index, relative_angle_rad);
}

/**
 * @brief 获取单个抬升电机的当前状态信息
 *
 * @param can_index 目标电机所在的 CAN 通道索引
 * @param status 用于接收状态信息的输出参数
 */
void app_liftup_get_status(uint8_t can_index, lift_motor_status &status)
{
    status = {};
    if (can_index >= 4 || !initialized) {
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
