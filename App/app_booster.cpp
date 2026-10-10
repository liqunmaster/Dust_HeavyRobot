#include "app_booster.hpp"
#include "input.hpp"

namespace
{
    constexpr fdcan_device bus = FDCAN_DEVICE_CAN1;
    constexpr uint8_t  motor_id    = 0x07;
    constexpr uint16_t tx_id_base  = 0x07;
    constexpr uint16_t feedback_id = 0x08;
    constexpr uint32_t period_ms   = 1;
    constexpr uint32_t active_feedback_timeout_ms = 12;
    constexpr uint32_t disabled_feedback_timeout_ms = 30;
    constexpr uint32_t disabled_keepalive_period_ms = 5;
    constexpr uint16_t offline_probe_ticks = 500;
    constexpr uint8_t fault_probe_ticks = 100;
    constexpr uint32_t feeding_timeout_ms = 1000;
    constexpr uint32_t recovery_timeout_ms = 1000;
    constexpr uint32_t recovery_wait_ms = 100;
    constexpr uint8_t  jam_confirm_ticks = 5;
    constexpr uint8_t  recovery_confirm_ticks = 5;
    constexpr uint8_t  max_jam_retries = 3;
    constexpr float pi = 3.14159265358979323846f;
    constexpr float shot_step_rad = -pi / 3.0f;
    constexpr float feeding_travel_limit_rad = 2.0f * -shot_step_rad;
    constexpr float jam_torque_nm = 4.0f;
    constexpr float low_speed_rad_s = 0.5f;
    constexpr float angle_tolerance_rad = 0.05f;

    dm_motor motor;
    alg::pid angle_pid;
    booster_status status{};
    struct k_spinlock status_lock;
    bool trigger_armed = false;
    bool initialized  = false;
    uint8_t jam_count = 0;
    uint8_t recovery_count  = 0;
    uint8_t jam_retries     = 0;
    uint32_t state_since_ms = 0;
    uint32_t last_motion_update_ms = 0;
    float feeding_travel_rad = 0.0f;
    uint32_t last_keepalive_ms = 0;
    uint16_t offline_ticks = offline_probe_ticks - 1;
    uint8_t fault_ticks = 0;
    int last_error = 0;
    float    target_angle_rad = 0.0f;
    booster_state state = booster_state::disabled;

    K_THREAD_STACK_DEFINE(booster_stack, 2048);
    struct k_thread booster_thread;
    K_SEM_DEFINE(booster_sem, 0, 1);
    struct k_timer booster_timer;
    uint32_t timer_period_ms = period_ms;

    /**
     * @brief 将角度限制到 [-pi, pi) 区间
     *
     * @param angle 输入角度
     * @return 限制后的角度
     */
    float wrap_angle(float angle)
    {
        return std::remainder(angle, 2.0f * pi);
    }

    /**
     * @brief 切换发射机工作状态并记录进入该状态的时刻
     *
     * @param next 目标状态
     * @param now_ms 当前运行时间
     */
    void set_state(booster_state next, uint32_t now_ms)
    {
        state = next;
        state_since_ms = now_ms;
        if (next == booster_state::feeding) {
            last_motion_update_ms = now_ms;
            feeding_travel_rad = 0.0f;
        }
    }

    /**
     * @brief 发送电机恢复帧
     *
     * @param clear_error 为 true 时发送清错帧 否则发送使能帧
     * @return 发送结果 0 表示成功
     */
    int send_recovery(bool clear_error)
    {
        fdcan_frame frame{};
        const int result = clear_error ? motor.build_clear_error_frame(frame) : motor.build_enable_frame(frame);
        return result == 0 ? fdcan_port_send_once(bus, frame) : result;
    }

    /**
     * @brief 向发射机电机发送力矩指令
     *
     * @param torque_nm 目标力矩
     * @return 发送结果 0 表示成功
     */
    int send_torque(float torque_nm)
    {
        int result = motor.set_mit(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
        if (result == 0) {
            result = fdcan_port_submit(motor);
        }
        return result;
    }

    /**
     * @brief 复位发射机运动状态并与给定角度对齐
     *
     * @param angle 当前角度
     * @param now_ms 当前运行时间
     * @param next 复位后切换到的目标状态
     */
    void reset_motion(float angle, uint32_t now_ms, booster_state next)
    {
        angle_pid.reset();
        target_angle_rad = angle;
        jam_count = 0;
        recovery_count = 0;
        jam_retries = 0;
        set_state(next, now_ms);
    }

    /**
     * @brief 根据输入与电机反馈更新发射机的运动状态机
     *
     * @param angle 当前电机角度
     * @param speed 当前电机速度
     * @param torque 当前电机力矩
     * @param now_ms 当前运行时间
     * @param armed 是否处于触发使能状态
     * @param rising 是否为一次新的发射触发沿
     */
    void update_motion(float angle, float speed, float torque, uint32_t now_ms, bool armed, bool rising)
    {
        if (!armed) {
            if (state != booster_state::disabled) {
                reset_motion(angle, now_ms, booster_state::disabled);
            }
            return;
        }
        if (state == booster_state::disabled || state == booster_state::motor_fault) {
            reset_motion(angle, now_ms, booster_state::ready);
        }

        switch (state) {
            case booster_state::ready:
                if (rising) {
                    target_angle_rad = wrap_angle(angle + shot_step_rad);
                    jam_count = 0;
                    jam_retries = 0;
                    set_state(booster_state::feeding, now_ms);
                }
                break;
            case booster_state::feeding:
                feeding_travel_rad += std::fabs(speed) * static_cast<float>(now_ms - last_motion_update_ms) * 0.001f;
                last_motion_update_ms = now_ms;
                if (now_ms - state_since_ms >= feeding_timeout_ms ||
                    feeding_travel_rad >= feeding_travel_limit_rad) {
                    angle_pid.reset();
                    set_state(booster_state::jam_fault, now_ms);
                    break;
                }
                if (std::fabs(torque) > jam_torque_nm && std::fabs(speed) < low_speed_rad_s) {
                    if (++jam_count >= jam_confirm_ticks) {
                        ++jam_retries;
                        recovery_count = 0;
                        target_angle_rad = wrap_angle(angle - shot_step_rad);
                        set_state(booster_state::reversing, now_ms);
                    }
                } else {
                    jam_count = 0;
                    if (std::fabs(wrap_angle(target_angle_rad - angle)) < angle_tolerance_rad &&
                        std::fabs(speed) < low_speed_rad_s) {
                        set_state(booster_state::ready, now_ms);
                    }
                }
                break;
            case booster_state::reversing:
                if (std::fabs(wrap_angle(target_angle_rad - angle)) < angle_tolerance_rad &&
                    std::fabs(speed) < low_speed_rad_s) {
                    if (++recovery_count >= recovery_confirm_ticks) {
                        set_state(booster_state::settling, now_ms);
                    }
                } else {
                    recovery_count = 0;
                }
                if (now_ms - state_since_ms >= recovery_timeout_ms) {
                    set_state(booster_state::settling, now_ms);
                }
                break;
            case booster_state::settling:
                if (now_ms - state_since_ms >= recovery_wait_ms) {
                    if (jam_retries >= max_jam_retries) {
                        angle_pid.reset();
                        set_state(booster_state::jam_fault, now_ms);
                    } else {
                        target_angle_rad = wrap_angle(angle + shot_step_rad);
                        jam_count = 0;
                        set_state(booster_state::feeding, now_ms);
                    }
                }
                break;
            case booster_state::jam_fault:
            case booster_state::disabled:
            case booster_state::motor_fault:
                break;
        }
    }

    /**
     * @brief 发射机任务的单次控制步进 处理反馈、状态机与力矩输出
     *
     */
    void control_step()
    {
        const uint32_t now_ms = k_uptime_get_32();
        fdcan_feedback_topic_data feedback{};
        const bool has_feedback = fdcan_topic_latest_feedback(bus, feedback_id, fdcan_motor_kind::dm, feedback) == 0;
        const uint32_t age_ms = has_feedback ? now_ms - feedback.timestamp_ms : UINT32_MAX;
        const dm_data data = motor.get_data();
        input_sample remote{};
        const bool armed = input_get_sample(remote_protocol::dt7, remote) == 0;
        const bool fresh = age_ms < (armed ? active_feedback_timeout_ms : disabled_feedback_timeout_ms);
        const bool motor_ready = fresh && data.status == 0x01;
        // DT7 S1 is the right switch. Require a fresh 1/3 position before each 2.
        const uint8_t s1 = armed ? remote.dt7_data.switch_right : 0;
        if (!armed) {
            trigger_armed = false;
        } else if (s1 == 1 || s1 == 3) {
            trigger_armed = true;
        }
        const bool rising = s1 == 2 && trigger_armed;
        if (rising) {
            trigger_armed = false;
        }
        int result = 0;
        float command_torque = 0.0f;

        if (!armed && state == booster_state::jam_fault) {
            reset_motion(data.now_angle, now_ms, booster_state::disabled);
        }

        if (!motor_ready) {
            if (state != booster_state::motor_fault && state != booster_state::jam_fault) {
                reset_motion(data.now_angle, now_ms, booster_state::motor_fault);
            }
            if (!fresh) {
                if (++offline_ticks >= offline_probe_ticks) {
                    offline_ticks = 0;
                    result = send_recovery(false);
                }
            } else {
                offline_ticks = 0;
                if (++fault_ticks >= fault_probe_ticks) {
                    fault_ticks = 0;
                    result = send_recovery(data.status >= 0x08);
                } else {
                    result = send_torque(0.0f);
                }
            }
        } else {
            offline_ticks = 0;
            fault_ticks = 0;
            update_motion(data.now_angle, data.now_omega, data.now_torque, now_ms, armed, rising);
            if (armed && state != booster_state::jam_fault && state != booster_state::disabled) {
                command_torque = angle_pid.update_angle(target_angle_rad, data.now_angle,  static_cast<float>(period_ms) * 0.001f);
            }
            if (armed || state != booster_state::disabled ||
                now_ms - last_keepalive_ms >= disabled_keepalive_period_ms) {
                result = send_torque(command_torque);
                last_keepalive_ms = now_ms;
            }
        }

        if (result != 0) {
            last_error = result;
        }
        const booster_status snapshot{state, fresh, data.status, age_ms, data.now_angle, target_angle_rad, data.now_omega, data.now_torque, command_torque, jam_retries, last_error};
        const k_spinlock_key_t key = k_spin_lock(&status_lock);
        status = snapshot;
        k_spin_unlock(&status_lock, key);

        const uint32_t next_period_ms = !armed && state == booster_state::disabled ?
            disabled_keepalive_period_ms : period_ms;
        if (next_period_ms != timer_period_ms) {
            timer_period_ms = next_period_ms;
            k_timer_start(&booster_timer, K_MSEC(next_period_ms), K_MSEC(next_period_ms));
        }
    }

    /**
     * @brief 定时器回调 唤醒发射机控制线程
     *
     * @param timer 触发回调的定时器
     */
    void timer_callback(struct k_timer *)
    {
        k_sem_give(&booster_sem);
    }

    /**
     * @brief 发射机控制线程入口 等待信号量并循环执行控制步进
     *
     * @param arg1 线程参数 1
     * @param arg2 线程参数 2
     * @param arg3 线程参数 3
     */
    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&booster_sem, K_FOREVER);
            control_step();
        }
    }
}

/**
 * @brief 初始化发射机模块 配置电机、PID 并启动控制线程
 *
 */
void app_booster_init()
{
    if (initialized) {
        return;
    }
    const dm_mit_limits limits{12.5f, 30.0f, 10.0f};
    int result = motor.init(bus, motor_id, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT, limits, FDCAN_PROTOCOL_CLASSIC, feedback_id, tx_id_base);
    if (result != 0) {
        return;
    }
    result = fdcan_port_bind(motor, false);
    if (result != 0) {
        return;
    }
    alg::pid_config pid{};
    pid.kp = 8.0f;
    pid.ki = 1.0f;
    pid.kd = 0.3f;
    pid.output_limit = 5.0f;
    pid.dt = 0.001f;
    pid.derivative_on_measurement = alg::d_first::Enable;
    pid.derivative_filter_tau = 0.01f;
    angle_pid.configure(pid);
    initialized = true;
    k_timer_init(&booster_timer, timer_callback, nullptr);
    k_timer_start(&booster_timer, K_MSEC(period_ms), K_MSEC(period_ms));
    k_thread_create(&booster_thread, booster_stack, K_THREAD_STACK_SIZEOF(booster_stack), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

/**
 * @brief 获取发射机当前状态信息
 *
 * @param snapshot 用于接收状态信息的输出参数
 */
void app_booster_get_status(booster_status &snapshot)
{
    snapshot = {};
    if (!initialized) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&status_lock);
    snapshot = status;
    k_spin_unlock(&status_lock, key);
}
