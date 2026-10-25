#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "canfd_bus.hpp"
#include "mcu.hpp"

#include "Subscription.hpp"
#include <uORB/topics/vehicle_attitude.h>
#include "time.hpp"
#include "input.hpp"

class McuBridgeH
{
public:
	void Init();

	bool Online();

	uint32_t ReceivedCount();

	uint32_t TransmitErrorCount();

private:
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	static void TimerCallback(struct k_timer *arg1);

	int SendFrame(uint32_t id, const uint8_t payload[8]);
	void SendHCommandFrames();
	static void ThreadEntry(void *arg1, void *arg2, void *arg3);

	void Task();

	struct k_thread _mcu_thread {};
	struct k_timer _mcu_timer {};
	atomic_t _received_count{};
	atomic_t _transmit_error_count{};
	atomic_t _online{};
	bool _initialized = false;
	uint32_t _last_rx_timestamp_ms = 0;
	uint32_t _last_rx_sequence[3] {};
	uint8_t _transmit_round = 0;
	PlatformCanfdRxHandle _feedback_handle{};
};

void mcu_common_init();

bool mcu_common_online();

uint32_t mcu_common_received_count();

uint32_t mcu_common_transmit_error_count();
