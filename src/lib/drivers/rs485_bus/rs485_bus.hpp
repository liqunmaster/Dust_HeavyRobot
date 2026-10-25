#pragma once

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>

constexpr size_t PLATFORM_RS485_SNAPSHOT_CAPACITY = 64;

struct PlatformRs485Snapshot {
	uint8_t data[PLATFORM_RS485_SNAPSHOT_CAPACITY] {};
	uint8_t length = 0;
	uint32_t timestamp_ms = 0;
	uint32_t sequence = 0;
};

struct PlatformRs485RxHandle {
	uint8_t slot = 0xFF;
};

class Rs485Bus
{
public:
	int Init();
	int Subscribe(const void *request, size_t request_length, size_t response_length,
		      PlatformRs485RxHandle &handle, uint32_t period_ms = 10,
		      uint32_t timeout_ms = 20);
	int Latest(const PlatformRs485RxHandle &handle, PlatformRs485Snapshot &snapshot);

private:
	struct Subscription {
		uint8_t request[PLATFORM_RS485_SNAPSHOT_CAPACITY] {};
		PlatformRs485Snapshot snapshot{};
		uint32_t period_ms = 0;
		uint32_t timeout_ms = 0;
		uint32_t next_query_ms = 0;
		uint32_t error_count = 0;
		int last_error = 0;
		uint8_t request_length = 0;
		uint8_t response_length = 0;
		bool used = false;
	};

	static void Work(struct k_work *work);
	static void Complete(void *context, int result, const uint8_t *data, size_t length);
	void Step();
	void OnComplete(int result, const uint8_t *data, size_t length);
	bool TakeDueQuery(size_t &slot, uint32_t &wait_ms);

	Subscription _subscriptions[4] {};
	struct k_spinlock _lock {};
	struct k_work_delayable _work {};
	size_t _next_slot = 0;
	uint8_t _active_slot = 0xFF;
	atomic_t _init_state = 0;
};

int platform_rs485_init();

int platform_rs485_subscribe(const void *request, size_t request_length,
			     size_t response_length, PlatformRs485RxHandle &handle,
			     uint32_t period_ms = 10, uint32_t timeout_ms = 20);

int platform_rs485_latest(const PlatformRs485RxHandle &handle,
			  PlatformRs485Snapshot &snapshot);
