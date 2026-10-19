#include "mcu_common.hpp"

namespace
{
    McuCommon mcu_common;

    K_THREAD_STACK_DEFINE(mcu_stack, 1536);

    K_SEM_DEFINE(mcu_sem, 0, 1);
}

void McuCommon::TimerCallback(struct k_timer *arg1)
{
    k_sem_give(&mcu_sem);
}

int McuCommon::SendFrame(uint32_t id, const uint8_t payload[8])
{
    CanfdFrame frame{};
    frame.id = id;
    frame.id_type = CANFD_ID_STANDARD;
    frame.protocol = CANFD_PROTOCOL_CLASSIC;
    frame.bitrate_switch = CANFD_BRS_DISABLED;
    frame.length = 8;
    for (uint8_t index = 0; index < 8; ++index) {
        frame.data[index] = payload[index];
    }
    const int result = platform_canfd_send_once(CANFD_DEVICE_CAN1, frame);
    if (result != 0) {
        atomic_inc(&transmit_error_count_);
    }
    return result;
}

void McuCommon::SendHCommandFrames()
{
    InputSample sample{};
    const bool remote_online = input_get_sample(RemoteProtocol::dt7, sample) == 0 &&
        k_uptime_get_32() - sample.timestamp_ms < 100U;
    McuChassisCommand chassis_command{1024, 1024, 1024, 0};
    uint16_t x = 1024;
    uint16_t y = 1024;
    uint16_t rotation = 1024;
    uint8_t flags = 0;
    if (remote_online) {
        x = sample.dt7_data.channel[2];
        y = sample.dt7_data.channel[3];

        rotation = 1024;
        flags = static_cast<uint8_t>((sample.dt7_data.switch_left & 0x03u) << 1);
        if (sample.dt7_data.switch_right == 2) {
            flags |= static_cast<uint8_t>(1u << 5);
        }
        if (sample.dt7_data.switch_left == 2) {
            flags |= static_cast<uint8_t>(1u << 6);
        }
    }
    chassis_command.x = x;
    chassis_command.y = y;
    chassis_command.rotation = rotation;
    chassis_command.flags = flags;
    uint8_t chassis_payload[8];
    mcu_encode_chassis(chassis_command, chassis_payload);

    const uint8_t round = transmit_round_++ % 4U;
    if ((round % 2U) == 0U) {
        (void)SendFrame(0x02, chassis_payload);
    } else if (round == 1U) {
        McuCommandCommand command{};
        command.mouse_lr = remote_online ? static_cast<uint8_t>(sample.dt7_data.switch_right == 2 ? 1u : 0u) : 0;
        InsEulerAngles angles{};
        command.imu_yaw_rad = ins_get_euler_angles(angles) &&
            k_uptime_get_32() - angles.timestamp_ms < 20U ? angles.yaw_rad : 0.0f;
        uint8_t command_payload[8];
        mcu_encode_command(command, command_payload);
        (void)SendFrame(0x02, command_payload);
    } else {
        uint8_t autoaim_payload[8];
        mcu_encode_autoaim(autoaim_payload);
        (void)SendFrame(0x02, autoaim_payload);
    }
}

void McuCommon::ThreadEntry(void *context, void *, void *)
{
    static_cast<McuCommon *>(context)->Task();
}

void McuCommon::Task()
{
    while (1) {
        k_sem_take(&mcu_sem, K_FOREVER);
        const uint32_t now_ms = k_uptime_get_32();
        PlatformCanfdSnapshot snapshot{};
        if (platform_canfd_latest(feedback_handle_, snapshot) == 0 &&
            snapshot.sequence != last_rx_sequence_[0]) {
            last_rx_sequence_[0] = snapshot.sequence;
            McuDecodedFrame frame{};
            if (mcu_decode_frame(snapshot.data, snapshot.length, frame) && frame.type == MCU_FRAME_REFEREE) {
                last_rx_timestamp_ms_ = snapshot.timestamp_ms;
                atomic_inc(&received_count_);
            }
        }
        atomic_set(&online_, atomic_get(&received_count_) != 0 && now_ms - last_rx_timestamp_ms_ < 50U);
        SendHCommandFrames();
    }
}

void McuCommon::Init()
{
    if (initialized_) {
        return;
    }
    if (platform_canfd_init() != 0) {
        return;
    }
    if (platform_canfd_subscribe(CANFD_DEVICE_CAN1, 0x03,
                                 CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC,
                                 feedback_handle_) != 0) {
        return;
    }
    k_timer_init(&mcu_timer_, TimerCallback, nullptr);
    k_timer_start(&mcu_timer_, K_MSEC(5), K_MSEC(5));
    k_thread_create(&mcu_thread_, mcu_stack, K_THREAD_STACK_SIZEOF(mcu_stack),
                    ThreadEntry, this, nullptr, nullptr,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(&mcu_thread_, "mcu_link");
    initialized_ = true;
}

bool McuCommon::Online()
{
    return atomic_get(&online_) != 0;
}

uint32_t McuCommon::ReceivedCount()
{
    return static_cast<uint32_t>(atomic_get(&received_count_));
}

uint32_t McuCommon::TransmitErrorCount()
{
    return static_cast<uint32_t>(atomic_get(&transmit_error_count_));
}

void mcu_common_init()
{
    mcu_common.Init();
}

bool mcu_common_online()
{
    return mcu_common.Online();
}

uint32_t mcu_common_received_count()
{
    return mcu_common.ReceivedCount();
}

uint32_t mcu_common_transmit_error_count()
{
    return mcu_common.TransmitErrorCount();
}
