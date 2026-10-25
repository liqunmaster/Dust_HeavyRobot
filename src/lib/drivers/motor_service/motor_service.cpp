#include "motor_service.hpp"

namespace
{
MotorService motor_topic;
}

void MotorService::Init()
{
	_control_sink = nullptr;
}

void MotorService::PublishFeedback(const MotorFeedbackCacheEntry &data)
{
	const k_spinlock_key_t key = k_spin_lock(&_latest_lock);
	LatestEntry *slot = nullptr;

	for (LatestEntry &entry : _latest) {
		if (entry.used && entry.feedback.bus == data.bus && entry.feedback.id == data.id &&
		    entry.feedback.kind == data.kind) {
			slot = &entry;
			break;
		}

		if (!entry.used && slot == nullptr) {
			slot = &entry;
		}
	}

	if (slot != nullptr) {
		slot->used = true;
		slot->feedback = data;

	} else {
		atomic_inc(&_feedback_dropped);
	}

	k_spin_unlock(&_latest_lock, key);
}

int MotorService::LatestFeedback(CanfdDevice bus, uint32_t id, MotorChannelKind kind, MotorFeedbackCacheEntry &data)
{
	if (_feedback_refresh != nullptr) {
		_feedback_refresh(bus, id, kind);
	}

	const k_spinlock_key_t key = k_spin_lock(&_latest_lock);

	for (const LatestEntry &entry : _latest) {
		if (entry.used && entry.feedback.bus == bus && entry.feedback.id == id &&
		    entry.feedback.kind == kind) {
			data = entry.feedback;
			k_spin_unlock(&_latest_lock, key);
			return 0;
		}
	}

	k_spin_unlock(&_latest_lock, key);
	return -ENODATA;
}

void MotorService::LatestFeedbackBatch(const MotorFeedbackKey *keys, size_t count, MotorFeedbackCacheEntry *data,
				       bool *found)
{
	if (keys == nullptr || data == nullptr || found == nullptr) {
		return;
	}

	if (_feedback_refresh_batch != nullptr) {
		_feedback_refresh_batch(keys, count);

	} else if (_feedback_refresh != nullptr) {
		for (size_t index = 0; index < count; ++index) {
			_feedback_refresh(keys[index].bus, keys[index].id, keys[index].kind);
		}
	}

	const k_spinlock_key_t key = k_spin_lock(&_latest_lock);

	for (size_t index = 0; index < count; ++index) {
		found[index] = false;

		for (const LatestEntry &entry : _latest) {
			if (entry.used && entry.feedback.bus == keys[index].bus &&
			    entry.feedback.id == keys[index].id && entry.feedback.kind == keys[index].kind) {
				data[index] = entry.feedback;
				found[index] = true;
				break;
			}
		}
	}

	k_spin_unlock(&_latest_lock, key);
}

int MotorService::PublishControl(const MotorControlRequest &data)
{
	const int result = _control_sink != nullptr ? _control_sink(data) : -ENODEV;

	if (result != 0) {
		atomic_inc(&_control_dropped);
	}

	return result;
}

void MotorService::SetControlSink(MotorControlSink sink)
{
	_control_sink = sink;
}

void MotorService::SetFeedbackRefresh(MotorFeedbackRefresh refresh)
{
	_feedback_refresh = refresh;
}

void MotorService::SetFeedbackRefreshBatch(MotorFeedbackRefreshBatch refresh)
{
	_feedback_refresh_batch = refresh;
}

uint32_t MotorService::FeedbackDroppedCount()
{
	return static_cast<uint32_t>(atomic_get(&_feedback_dropped));
}

uint32_t MotorService::ControlDroppedCount()
{
	return static_cast<uint32_t>(atomic_get(&_control_dropped));
}

void motor_service_init()
{
	motor_topic.Init();
}

void motor_service_publish_feedback(const MotorFeedbackCacheEntry &data)
{
	motor_topic.PublishFeedback(data);
}

int motor_service_latest_feedback(CanfdDevice bus, uint32_t id, MotorChannelKind kind, MotorFeedbackCacheEntry &data)
{
	return motor_topic.LatestFeedback(bus, id, kind, data);
}

void motor_service_latest_feedback_batch(const MotorFeedbackKey *keys, size_t count, MotorFeedbackCacheEntry *data,
		bool *found)
{
	motor_topic.LatestFeedbackBatch(keys, count, data, found);
}

int motor_service_publish_control(const MotorControlRequest &data)
{
	return motor_topic.PublishControl(data);
}

void motor_service_set_control_sink(MotorControlSink sink)
{
	motor_topic.SetControlSink(sink);
}

void motor_service_set_feedback_refresh(MotorFeedbackRefresh refresh)
{
	motor_topic.SetFeedbackRefresh(refresh);
}

void motor_service_set_feedback_refresh_batch(MotorFeedbackRefreshBatch refresh)
{
	motor_topic.SetFeedbackRefreshBatch(refresh);
}

uint32_t motor_service_feedback_dropped_count()
{
	return motor_topic.FeedbackDroppedCount();
}

uint32_t motor_service_control_dropped_count()
{
	return motor_topic.ControlDroppedCount();
}
