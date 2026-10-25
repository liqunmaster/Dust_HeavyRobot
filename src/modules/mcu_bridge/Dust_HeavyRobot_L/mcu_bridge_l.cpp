#include "mcu_bridge_l.hpp"

namespace
{
McuBridgeL mcu_common;

K_THREAD_STACK_DEFINE(mcu_stack, 1536);

K_SEM_DEFINE(mcu_sem, 0, 1);
}

void McuBridgeL::TimerCallback(struct k_timer *arg1)
{
	k_sem_give(&mcu_sem);
}

int McuBridgeL::SendFrame(uint32_t id, const uint8_t payload[8])
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
		atomic_inc(&_transmit_error_count);
	}

	return result;
}

void McuBridgeL::ProcessLFrame(const PlatformCanfdSnapshot &snapshot)
{
	McuDecodedFrame frame{};

	if (!mcu_decode_frame(snapshot.data, snapshot.length, frame)) {
		return;
	}

	switch (frame.type) {
	case MCU_FRAME_CHASSIS:
		_last_chassis_timestamp_ms = snapshot.timestamp_ms;
		_chassis_expired = false;
		_input.PublishChassis(frame.chassis, snapshot.timestamp_ms);
		break;

	case MCU_FRAME_COMMAND:
		_last_command_timestamp_ms = snapshot.timestamp_ms;
		_command_expired = false;
		_input.PublishCommand(frame.command, snapshot.timestamp_ms);
		break;

	case MCU_FRAME_AUTOAIM:
	default:
		break;
	}
}

void McuBridgeL::SendLFeedback()
{
	uint8_t feedback_payload[8];
	mcu_encode_referee(feedback_payload);
	(void)SendFrame(0x101, feedback_payload);
}

void McuBridgeL::ThreadEntry(void *context, void *, void *)
{
	static_cast<McuBridgeL *>(context)->Task();
}

void McuBridgeL::Task()
{
	while (1) {
		k_sem_take(&mcu_sem, K_FOREVER);
		const uint32_t now_ms = k_uptime_get_32();
		PlatformCanfdSnapshot snapshot{};

		for (uint8_t index = 0; index < 3; ++index) {
			if (platform_canfd_latest(_command_handles[index], snapshot) == 0 &&
			    snapshot.sequence != _last_rx_sequence[index]) {
				_last_rx_sequence[index] = snapshot.sequence;
				McuDecodedFrame frame{};

				if (mcu_decode_frame(snapshot.data, snapshot.length, frame)) {
					_last_rx_timestamp_ms = snapshot.timestamp_ms;
					atomic_inc(&_received_count);
					ProcessLFrame(snapshot);
				}
			}
		}

		if (!_chassis_expired && now_ms - _last_chassis_timestamp_ms >= 30U) {
			_chassis_expired = true;
			_input.PublishChassis({1024, 1024, 1024, 0}, _last_chassis_timestamp_ms);
		}

		if (!_command_expired && now_ms - _last_command_timestamp_ms >= 30U) {
			_command_expired = true;
			_input.PublishCommand({0, 0, 0.0f}, _last_command_timestamp_ms);
		}

		atomic_set(&_online, !_chassis_expired && !_command_expired);
		SendLFeedback();
	}
}

void McuBridgeL::Init()
{
	if (_initialized) {
		return;
	}

	if (platform_canfd_init() != 0) {
		return;
	}

	for (uint8_t index = 0; index < 3; ++index) {
		if (platform_canfd_subscribe(CANFD_DEVICE_CAN1, 0x100,
					     CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC,
					     _command_handles[index], MCU_FRAME_CHASSIS + index) != 0) {
			return;
		}
	}

	k_timer_init(&_mcu_timer, TimerCallback, nullptr);
	k_timer_start(&_mcu_timer, K_MSEC(10), K_MSEC(10));
	k_thread_create(&_mcu_thread, mcu_stack, K_THREAD_STACK_SIZEOF(mcu_stack),
			ThreadEntry, this, nullptr, nullptr,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&_mcu_thread, "mcu_link");
	_initialized = true;
}

bool McuBridgeL::Online()
{
	return atomic_get(&_online) != 0;
}

uint32_t McuBridgeL::ReceivedCount()
{
	return static_cast<uint32_t>(atomic_get(&_received_count));
}

uint32_t McuBridgeL::TransmitErrorCount()
{
	return static_cast<uint32_t>(atomic_get(&_transmit_error_count));
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
