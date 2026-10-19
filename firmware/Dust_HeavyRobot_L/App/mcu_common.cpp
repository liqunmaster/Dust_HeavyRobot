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

void McuCommon::ProcessLFrame(const PlatformCanfdSnapshot &snapshot)
{
    McuDecodedFrame frame{};
    if (!mcu_decode_frame(snapshot.data, snapshot.length, frame)) {
        return;
    }
    switch (frame.type) {
        case MCU_FRAME_CHASSIS:
            last_chassis_timestamp_ms_ = snapshot.timestamp_ms;
            chassis_expired_ = false;
            input_.PublishChassis(frame.chassis, snapshot.timestamp_ms);
            break;
        case MCU_FRAME_COMMAND:
            last_command_timestamp_ms_ = snapshot.timestamp_ms;
            command_expired_ = false;
            input_.PublishCommand(frame.command, snapshot.timestamp_ms);
            break;
        case MCU_FRAME_AUTOAIM:
        default:
            break;
    }
}

void McuCommon::SendLFeedback()
{
    uint8_t feedback_payload[8];
    mcu_encode_referee(feedback_payload);
    (void)SendFrame(0x03, feedback_payload);
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

        for (uint8_t index = 0; index < 3; ++index) {
            if (platform_canfd_latest(command_handles_[index], snapshot) == 0 &&
                snapshot.sequence != last_rx_sequence_[index]) {
                last_rx_sequence_[index] = snapshot.sequence;
                McuDecodedFrame frame{};
                if (mcu_decode_frame(snapshot.data, snapshot.length, frame)) {
                    last_rx_timestamp_ms_ = snapshot.timestamp_ms;
                    atomic_inc(&received_count_);
                    ProcessLFrame(snapshot);
                }
            }
        }
        if (!chassis_expired_ && now_ms - last_chassis_timestamp_ms_ >= 30U) {
            chassis_expired_ = true;
            input_.PublishChassis({1024, 1024, 1024, 0}, last_chassis_timestamp_ms_);
        }
        if (!command_expired_ && now_ms - last_command_timestamp_ms_ >= 30U) {
            command_expired_ = true;
            input_.PublishCommand({0, 0, 0.0f}, last_command_timestamp_ms_);
        }
        atomic_set(&online_, !chassis_expired_ && !command_expired_);
        SendLFeedback();
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
    for (uint8_t index = 0; index < 3; ++index) {
        if (platform_canfd_subscribe(CANFD_DEVICE_CAN1, 0x02,
                                    CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC,
                                    command_handles_[index], MCU_FRAME_CHASSIS + index) != 0) {
            return;
        }
    }
    k_timer_init(&mcu_timer_, TimerCallback, nullptr);
    k_timer_start(&mcu_timer_, K_MSEC(10), K_MSEC(10));
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
