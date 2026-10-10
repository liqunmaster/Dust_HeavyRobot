#include "app_chassis.hpp"

namespace
{
    // 麦克纳姆轮底盘几何参数
    struct MecanumGeometry
    {
        float wheel_radius_m;
        float half_length_m;
        float half_width_m;
    };

    // Wheel order: front left, front right, rear left, rear right.
    /**
     * @brief 将车体速度逆解为四个麦克纳姆轮的角速度
     *
     * @param geometry 底盘几何参数
     * @param vx_m_s 车体前进方向速度（米/秒）
     * @param vy_m_s 车体左右方向速度（米/秒）
     * @param yaw_rad_s 车体自转角速度（弧度/秒）
     * @param wheel_rad_s[4] 输出的四个轮子角速度（弧度/秒）
    */
    void mecanum_inverse(const MecanumGeometry &geometry, float vx_m_s, float vy_m_s, float yaw_rad_s, float wheel_rad_s[4])
    {
        const float arm = geometry.half_length_m + geometry.half_width_m;
        const float scale = 1.0f / geometry.wheel_radius_m;
        wheel_rad_s[0] = (vx_m_s - vy_m_s - arm * yaw_rad_s) * scale;
        wheel_rad_s[1] = (vx_m_s + vy_m_s + arm * yaw_rad_s) * scale;
        wheel_rad_s[2] = (vx_m_s + vy_m_s - arm * yaw_rad_s) * scale;
        wheel_rad_s[3] = (vx_m_s - vy_m_s + arm * yaw_rad_s) * scale;
    }

    /**
     * @brief 将四个轮子的角速度统一限幅，保持各轮速度比例不变
     *
     * @param wheel_rad_s[4] 四个轮子的角速度（弧度/秒），将被就地限幅
     * @param limit_rad_s 角速度上限值（弧度/秒）
    */
    void mecanum_limit_wheel_speeds(float wheel_rad_s[4], float limit_rad_s)
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
    uint32_t last_offline_command_ms = 0;

    /**
     * @brief 底盘定时器回调，唤醒控制线程
     *
     * @param timer 触发回调的定时器
    */
    void chassis_timer_callback(struct k_timer *)
    {
        k_sem_give(&chassis_sem);
    }

    /**
     * @brief 底盘单次控制步进，读取指令与反馈并驱动四个电机
     *
     * @param dt_s 控制周期（秒）
    */
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
                (void)motors[index].set_current(0.0f);
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
        fdcan_topic_latest_feedback_batch(feedback_keys, 4, feedback, has_feedback);
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

    /**
     * @brief 底盘控制线程入口，等待信号量并循环执行控制步进
     *
     * @param arg1 线程参数 1
     * @param arg2 线程参数 2
     * @param arg3 线程参数 3
    */
    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&chassis_sem, K_FOREVER);
            control_step(static_cast<float>(period_ms) * 0.001f);
        }
    }
}

/**
 * @brief 初始化底盘模块，配置四个麦克纳姆轮电机及速度环并启动控制线程
 *
*/
void app_chassis_init()
{
    if (initialized) {
        return;
    }
    for (float direction : wheel_direction) {
        if (direction != 1.0f && direction != -1.0f) {
            return;
        }
    }

    const C620_ID ids[4] = {C620_ID_0x201, C620_ID_0x202, C620_ID_0x203, C620_ID_0x204};
    alg::PidConfig speed_pid{};
    speed_pid.kp = 3.0f;
    speed_pid.ki = 0.0f;
    speed_pid.kd = 0.0f;
    speed_pid.integral_limit = 20.0f * 0.5f;
    speed_pid.output_limit   = 20.0f;
    speed_pid.dt = 0.001f;
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
        (void)motors[index].set_current(0.0f);
    }
    initialized = true;
    k_timer_init(&chassis_timer, chassis_timer_callback, nullptr);
    k_timer_start(&chassis_timer, K_MSEC(period_ms), K_MSEC(period_ms));
    k_thread_create(&chassis_thread, chassis_task, K_THREAD_STACK_SIZEOF(chassis_task), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

/**
 * @brief 设置底盘目标运动速度（车体坐标系），发布给底盘控制线程
 *
 * @param vx_m_s 车体前进方向速度（米/秒）
 * @param vy_m_s 车体左右方向速度（米/秒）
 * @param yaw_rad_s 车体自转角速度（弧度/秒）
*/
void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
    if (!initialized) {
        return;
    }
    remote_channel_publish_chassis(vx_m_s, vy_m_s, yaw_rad_s);
}