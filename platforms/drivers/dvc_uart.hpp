#pragma once

#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ring_buffer.hpp"

struct UartDmaBuffers {
	uint8_t rx[2][128];
	uint8_t tx[128];
};

class Uart
{
public:
	static constexpr size_t rx_capacity = 512;
	static constexpr size_t tx_capacity = 128;
	static constexpr size_t dma_capacity = 128;

	constexpr Uart(const struct device *device, UartDmaBuffers &dma)
		: _device(device), _dma(dma) {}

	Uart(const Uart &) = delete;
	Uart &operator=(const Uart &) = delete;
	Uart(Uart &&) = delete;
	Uart &operator=(Uart &&) = delete;

	int Init();
	int Receive(void *data, size_t length);
	int Transmit(const void *data, size_t length);
	int TransmitAsync(const void *data, size_t length);
	int AbortTransmit();
	int ClearRx();
	int SetRxNotify(void (*notify)(void *), void *context);
	int SetTxNotify(void (*notify)(void *, int), void *context);
	bool IsReady() const;

	const struct device *GetDevice() const { return _device; }

private:
	static void UartCallback(const struct device *device, struct uart_event *event, void *user_data);
	void HandleEvent(const struct uart_event &event);
	int StartRx();

	const struct device *const _device;
	UartDmaBuffers &_dma;
	RingBuffer _rx_buffer{};
	RingBuffer _tx_buffer{};
	uint8_t _rx_storage[rx_capacity] {};
	uint8_t _tx_storage[tx_capacity] {};
	bool _rx_owned[2] {};
	bool _rx_active = false;
	void (*rx_notify_)(void *) = nullptr;
	void *_rx_notify_context = nullptr;
	void (*tx_notify_)(void *, int) = nullptr;
	void *_tx_notify_context = nullptr;

	atomic_t _init_state = 0;
	atomic_t _rx_overflow = 0;
	atomic_t _tx_busy = 0;
	size_t _tx_count = 0;
	int _tx_error = 0;
	struct k_mutex _tx_lock {};
	struct k_sem _tx_done {};
};

extern Uart uart0;
extern Uart uart1;
extern Uart uart2;
extern Uart uart3;
extern Uart uart4;
extern Uart uart5;
extern Uart uart6;

Uart *bsp_uart_get(const struct device *device);

void bsp_uart_init(const struct device *device = nullptr);

int bsp_uart_receive(void *data, size_t length, const struct device *device = nullptr);

int bsp_uart_transmit(const void *data, size_t length, const struct device *device = nullptr);
