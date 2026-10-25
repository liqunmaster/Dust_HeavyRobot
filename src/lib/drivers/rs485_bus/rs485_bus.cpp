#include "rs485_bus.hpp"

#include <errno.h>
#include <limits.h>
#include <string.h>
#include "dvc_rs485.hpp"

namespace
{
Rs485Bus rs485_bus;
}

int Rs485Bus::Init()
{
	if (atomic_get(&_init_state) == 2) {
		return 0;
	}

	if (!atomic_cas(&_init_state, 0, 1)) {
		return -EBUSY;
	}

	k_work_init_delayable(&_work, Work);
	const int ret = bsp_rs485_init();

	if (ret != 0) {
		atomic_clear(&_init_state);
		return ret;
	}

	atomic_set(&_init_state, 2);
	return 0;
}

int Rs485Bus::Subscribe(const void *request, size_t request_length,
			size_t response_length, PlatformRs485RxHandle &handle,
			uint32_t period_ms, uint32_t timeout_ms)
{
	if (request == nullptr || request_length == 0 ||
	    request_length > PLATFORM_RS485_SNAPSHOT_CAPACITY || response_length == 0 ||
	    response_length > PLATFORM_RS485_SNAPSHOT_CAPACITY ||
	    period_ms == 0 || period_ms > INT32_MAX || timeout_ms == 0 || timeout_ms > INT32_MAX) {
		return -EINVAL;
	}

	const int ret = Init();

	if (ret != 0) {
		return ret;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	for (size_t index = 0; index < 4; ++index) {
		const Subscription &entry = _subscriptions[index];

		if (entry.used && entry.request_length == request_length &&
		    memcmp(entry.request, request, request_length) == 0) {
			if (entry.response_length != response_length || entry.period_ms != period_ms ||
			    entry.timeout_ms != timeout_ms) {
				k_spin_unlock(&_lock, key);
				return -EINVAL;
			}

			handle.slot = static_cast<uint8_t>(index);
			k_spin_unlock(&_lock, key);
			return 0;
		}
	}

	for (size_t index = 0; index < 4; ++index) {
		Subscription &entry = _subscriptions[index];

		if (!entry.used) {
			memcpy(entry.request, request, request_length);
			entry.request_length = static_cast<uint8_t>(request_length);
			entry.response_length = static_cast<uint8_t>(response_length);
			entry.period_ms = period_ms;
			entry.timeout_ms = timeout_ms;
			entry.next_query_ms = k_uptime_get_32();
			entry.used = true;
			handle.slot = static_cast<uint8_t>(index);
			k_spin_unlock(&_lock, key);
			k_work_reschedule(&_work, K_NO_WAIT);
			return 0;
		}
	}

	k_spin_unlock(&_lock, key);
	return -ENOSPC;
}

int Rs485Bus::Latest(const PlatformRs485RxHandle &handle, PlatformRs485Snapshot &snapshot)
{
	if (handle.slot >= 4) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);
	const Subscription &entry = _subscriptions[handle.slot];

	if (!entry.used) {
		k_spin_unlock(&_lock, key);
		return -ENOENT;
	}

	snapshot = entry.snapshot;
	k_spin_unlock(&_lock, key);
	return snapshot.sequence == 0 ? -ENODATA : 0;
}

bool Rs485Bus::TakeDueQuery(size_t &slot, uint32_t &wait_ms)
{
	wait_ms = UINT32_MAX;
	const uint32_t now_ms = k_uptime_get_32();
	const k_spinlock_key_t key = k_spin_lock(&_lock);

	for (size_t offset = 0; offset < 4; ++offset) {
		const size_t index = (_next_slot + offset) % 4;
		const Subscription &entry = _subscriptions[index];

		if (!entry.used) {
			continue;
		}

		if (static_cast<int32_t>(now_ms - entry.next_query_ms) >= 0) {
			slot = index;
			_next_slot = (index + 1) % 4;
			wait_ms = 0;
			k_spin_unlock(&_lock, key);
			return true;
		}

		const uint32_t remaining_ms = entry.next_query_ms - now_ms;

		if (remaining_ms < wait_ms) {
			wait_ms = remaining_ms;
		}
	}

	k_spin_unlock(&_lock, key);
	return false;
}

void Rs485Bus::Work(struct k_work *work)
{
	CONTAINER_OF(k_work_delayable_from_work(work), Rs485Bus, _work)->Step();
}

void Rs485Bus::Step()
{
	if (_active_slot != 0xFF) {
		return;
	}

	size_t slot = 0;
	uint32_t wait_ms = 0;

	if (!TakeDueQuery(slot, wait_ms)) {
		if (wait_ms != UINT32_MAX) {
			k_work_reschedule(&_work, K_MSEC(wait_ms));
		}

		return;
	}

	_active_slot = static_cast<uint8_t>(slot);
	const Subscription &entry = _subscriptions[slot];
	const int ret = bsp_rs485_start(entry.request, entry.request_length, entry.response_length,
					entry.timeout_ms, Complete, this);

	if (ret != 0) {
		OnComplete(ret, nullptr, 0);
	}
}

void Rs485Bus::Complete(void *context, int result, const uint8_t *data, size_t length)
{
	static_cast<Rs485Bus *>(context)->OnComplete(result, data, length);
}

void Rs485Bus::OnComplete(int result, const uint8_t *data, size_t length)
{
	const uint32_t now_ms = k_uptime_get_32();
	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (_active_slot >= 4) {
		k_spin_unlock(&_lock, key);
		return;
	}

	Subscription &entry = _subscriptions[_active_slot];
	entry.next_query_ms = now_ms + entry.period_ms;

	if (result != 0 || data == nullptr || length != entry.response_length) {
		entry.last_error = result != 0 ? result : -EMSGSIZE;
		++entry.error_count;

	} else {
		memcpy(entry.snapshot.data, data, length);
		memset(entry.snapshot.data + length, 0, sizeof(entry.snapshot.data) - length);
		entry.snapshot.length = static_cast<uint8_t>(length);
		entry.snapshot.timestamp_ms = now_ms;
		++entry.snapshot.sequence;
		entry.last_error = 0;
	}

	_active_slot = 0xFF;
	k_spin_unlock(&_lock, key);
	k_work_reschedule(&_work, K_NO_WAIT);
}

int platform_rs485_init()
{
	return rs485_bus.Init();
}
int platform_rs485_subscribe(const void *request, size_t request_length,
			     size_t response_length, PlatformRs485RxHandle &handle,
			     uint32_t period_ms, uint32_t timeout_ms)
{
	return rs485_bus.Subscribe(request, request_length, response_length,
				   handle, period_ms, timeout_ms);
}
int platform_rs485_latest(const PlatformRs485RxHandle &handle, PlatformRs485Snapshot &snapshot)
{
	return rs485_bus.Latest(handle, snapshot);
}
