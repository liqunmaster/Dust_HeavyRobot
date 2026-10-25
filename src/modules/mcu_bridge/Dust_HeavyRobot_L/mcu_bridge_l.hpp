#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "canfd_bus.hpp"
#include "mcu.hpp"

#include "board_input.hpp"

class McuBridgeL
{
public:
	void Init();

	bool Online();

	uint32_t ReceivedCount();

	uint32_t TransmitErrorCount();

private:
	static void TimerCallback(struct k_timer *arg1);

	int SendFrame(uint32_t id, const uint8_t payload[8]);
	void ProcessLFrame(const PlatformCanfdSnapshot &snapshot);

	void SendLFeedback();
	static void ThreadEntry(void *arg1, void *arg2, void *arg3);

	void Task();

	BoardInput _input{};

	struct k_thread _mcu_thread {};
	struct k_timer _mcu_timer {};
	atomic_t _received_count{};
	atomic_t _transmit_error_count{};
	atomic_t _online{};
	bool _initialized = false;
	uint32_t _last_rx_timestamp_ms = 0;
	uint32_t _last_rx_sequence[3] {};
	uint32_t _last_chassis_timestamp_ms = 0;
	bool _chassis_expired = true;
	bool _command_expired = true;
	uint32_t _last_command_timestamp_ms = 0;
	PlatformCanfdRxHandle _command_handles[3] {};
};

void mcu_common_init();

bool mcu_common_online();

uint32_t mcu_common_received_count();

uint32_t mcu_common_transmit_error_count();
