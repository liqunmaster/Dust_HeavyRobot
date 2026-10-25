#pragma once
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include "dvc_uart.hpp"

using Rs485Complete = void (*)(void *, int, const uint8_t *, size_t);

class Rs485
{
public:
	static constexpr size_t response_capacity = 64;
	explicit constexpr Rs485(Uart &port) : _port(&port) {}
	Rs485(const Rs485 &) = delete;
	Rs485 &operator=(const Rs485 &) = delete;
	int Init();
	int Start(const void *request, size_t request_length, size_t response_length,
		  uint32_t timeout_ms, Rs485Complete complete, void *context);
	bool IsReady() const;

private:
	enum class Phase : uint8_t { idle, sending, receiving, completing };
	static void RxNotify(void *context);
	static void TxNotify(void *context, int result);
	static void LineIrq(const struct device *, void *context);
	static void Timeout(struct k_timer *timer);
	static void Work(struct k_work *work);
	void BeginReceive();
	void Finish(int result);
	void Process();
	Uart *const _port;
	struct k_work _work {};
	struct k_timer _timer {};
	atomic_t _init_state = 0;
	Phase _phase = Phase::idle;
	Rs485Complete _complete = nullptr;
	void *_context = nullptr;
	uint8_t _response[response_capacity] {};
	size_t _response_length = 0;
	size_t _received = 0;
	uint32_t _response_timeout_ms = 0;
	int _result = 0;
	uint32_t _received_bytes = 0;
	uint32_t _transaction_count = 0;
	uint32_t _timeout_count = 0;
	size_t _last_response_length = 0;
};
extern Rs485 rs485_uart5;
int bsp_rs485_init(const struct device *device = nullptr);

int bsp_rs485_start(const void *request, size_t request_length, size_t response_length,
		    uint32_t timeout_ms, Rs485Complete complete, void *context);
bool bsp_rs485_is_ready();
