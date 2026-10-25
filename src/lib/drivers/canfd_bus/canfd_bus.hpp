#pragma once

#include <stddef.h>
#include <stdint.h>
#include "dvc_canfd.hpp"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

struct PlatformCanfdRxHandle {
	uint8_t slot = 0xFF;
};

struct PlatformCanfdSnapshot {
	uint8_t data[canfd_max_data_length] {};
	uint8_t length = 0;
	uint32_t timestamp_ms = 0;
	uint32_t sequence = 0;
};

class CanfdBus
{
public:
	int Init();

	int Subscribe(CanfdDevice bus, uint32_t id, CanfdIdType id_type,
		      CanfdProtocol protocol, PlatformCanfdRxHandle &handle,
		      int16_t payload_type = -1);

	int Latest(const PlatformCanfdRxHandle &handle, PlatformCanfdSnapshot &snapshot);

	int Submit(CanfdDevice bus, const CanfdFrame &frame);

	int SendOnce(CanfdDevice bus, const CanfdFrame &frame);

	uint32_t RxDroppedCount();

	uint32_t TxDroppedCount();

private:
	struct Subscription {
		bool used = false;
		CanfdDevice bus = CANFD_DEVICE_COUNT;
		uint32_t id = 0;
		CanfdIdType id_type = CANFD_ID_STANDARD;
		CanfdProtocol protocol = CANFD_PROTOCOL_CLASSIC;
		int16_t payload_type = -1;
		uint8_t next = 0; // 槽位索引加 1；零表示桶链结束。
		PlatformCanfdSnapshot snapshot{};
	};

	struct TxMailbox {
		struct PeriodicEntry {
			bool used = false;
			bool pending = false;
			CanfdDevice bus = CANFD_DEVICE_COUNT;
			CanfdFrame frame{};
		};
		size_t next_periodic = 0;
		CanfdFrame admin[8] {};
		size_t admin_head = 0;
		size_t admin_tail = 0;
		size_t admin_count = 0;
		struct k_sem wake;
		struct k_thread thread;
		bool ready = false;
	};

	bool ValidBus(CanfdDevice bus);

	bool ValidFrame(const CanfdFrame &frame);

	static void rx_callback(CanfdDevice bus, const CanfdRxView *frame, void *arg3);

	void OnRxCallback(CanfdDevice bus, const CanfdRxView *frame, void *arg3);

	bool TakeNextFrame(CanfdDevice bus, CanfdFrame &frame);

	static void TxWorker(void *p1, void *arg2, void *arg3);

	void OnTxWorker(void *p1, void *arg2, void *arg3);

	int EnsureBus(CanfdDevice bus);
	Subscription _subscriptions[64] {};
	uint8_t _subscription_heads[CANFD_DEVICE_COUNT][32] {};

	TxMailbox::PeriodicEntry _periodic[32] {};
	TxMailbox _mailboxes[CANFD_DEVICE_COUNT] {};
	struct k_spinlock _subscription_lock {};
	struct k_spinlock _tx_lock {};
	struct k_mutex _init_lock {};
	atomic_t _initialized{};
	atomic_t _rx_dropped{};
	atomic_t _tx_dropped{};
};

int platform_canfd_init();

int platform_canfd_subscribe(CanfdDevice bus,
			     uint32_t id,
			     CanfdIdType id_type,
			     CanfdProtocol protocol,
			     PlatformCanfdRxHandle &handle,
			     int16_t payload_type = -1);

int platform_canfd_latest(const PlatformCanfdRxHandle &handle,
			  PlatformCanfdSnapshot &snapshot);

int platform_canfd_submit(CanfdDevice bus, const CanfdFrame &frame);

int platform_canfd_send_once(CanfdDevice bus, const CanfdFrame &frame);

uint32_t platform_canfd_rx_dropped_count();

uint32_t platform_canfd_tx_dropped_count();
