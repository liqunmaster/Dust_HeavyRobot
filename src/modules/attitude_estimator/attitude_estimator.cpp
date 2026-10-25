#include "attitude_estimator.hpp"

#include <stdio.h>
#include "dvc_pwm.hpp"

namespace
{
AttitudeEstimator imu_port;

K_THREAD_STACK_DEFINE(imu_stack, 8192);

K_SEM_DEFINE(imu_sem, 0, 1);
}

void AttitudeEstimator::timer_callback(struct k_timer *arg1)
{
	k_sem_give(&imu_sem);
}

void AttitudeEstimator::StoreSample(const ImuSample &sample)
{
	const k_spinlock_key_t key = k_spin_lock(&_state.lock);
	_state.sample = sample;
	_state.has_sample = true;
	k_spin_unlock(&_state.lock, key);
}

void AttitudeEstimator::ProcessSample(const ImuSample &sample)
{
	const uint32_t now_cycles = k_cycle_get_32();
	float dt_seconds = (1.0f / 1000.0f);

	if (_have_sample_time) {
		dt_seconds = static_cast<float>(now_cycles - _last_sample_cycles) / static_cast<float>
			     (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

		if (dt_seconds <= 0.0f || dt_seconds > 0.1f) {
			dt_seconds = (1.0f / 1000.0f);
		}
	}

	_last_sample_cycles = now_cycles;
	_have_sample_time = true;

	sensor_imu_s physical{};
	physical.timestamp = robot::now_us();
	physical.timestamp_sample = physical.timestamp;
	physical.temperature_c = 23.0f + static_cast<float>(sample.temperature_raw) / 512.0f;

	for (int axis = 0; axis < 3; ++axis) {
		physical.acceleration_m_s2[axis] = sample.accel_raw[axis] * _config.accel_m_s2_per_count;
		physical.angular_velocity_rad_s[axis] = sample.gyro_raw[axis] * _config.gyro_rad_s_per_count;
	}

	_sample_pub.publish(physical);

	if (_estimator.Process(physical.acceleration_m_s2, physical.angular_velocity_rad_s, dt_seconds,
			       physical.timestamp_sample)) {
		AttitudeEstimate estimate{};

		if (_estimator.GetEulerAngles(estimate)) {
			vehicle_attitude_s attitude{};
			attitude.timestamp = physical.timestamp;
			attitude.timestamp_sample = estimate.timestamp_sample;
			attitude.roll_rad = estimate.roll_rad;
			attitude.pitch_rad = estimate.pitch_rad;
			attitude.yaw_rad = estimate.yaw_rad;
			attitude.yaw_unwrapped_rad = estimate.yaw_unwrapped_rad;
			attitude.valid = estimate.valid;
			_attitude_pub.publish(attitude);
		}
	}
}

void AttitudeEstimator::UpdateHeater(const ImuSample &sample, uint32_t now_ms)
{
	if (now_ms - _last_heater_update_ms < 100) {
		return;
	}

	if (!_heater_ready) {
		_last_heater_update_ms = now_ms;
		_heater_ready = bsp_pwm_heater_init() == 0;
		return;
	}

	const float dt = static_cast<float>(now_ms - _last_heater_update_ms) * 0.001f;
	_last_heater_update_ms = now_ms;
	const float temperature_c = 23.0f + static_cast<float>(sample.temperature_raw) / 512.0f;

	if (temperature_c < -40.0f || temperature_c > 85.0f || temperature_c >= 45.0f) {
		_heater_integral = 0.0f;
		_heater_output = 0.0f;
		bsp_pwm_heater_set_duty(0.0f);
		return;
	}

	const float error = 40.0f - temperature_c;
	const float proportional = 0.08f * error;
	const float candidate = proportional + _heater_integral;

	if (!((candidate >= 0.95f && error > 0.0f) || (candidate <= 0.0f && error < 0.0f))) {
		_heater_integral += 0.02f * error * dt;
	}

	const float duty = proportional + _heater_integral;
	_heater_output = duty < 0.0f ? 0.0f : (duty > 0.95f ? 0.95f : duty);
	bsp_pwm_heater_set_duty(_heater_output);
}

void AttitudeEstimator::task_trampoline(void *context, void *, void *)
{
	static_cast<AttitudeEstimator *>(context)->Run();
}

void AttitudeEstimator::Run()
{
	while (1) {
		k_sem_take(&imu_sem, K_FOREVER);

		ImuSample sample{};

		if (_imu_device.ReadSample(sample) != 0) {
			if (_heater_ready && _heater_output > 0.0f &&
			    k_uptime_get_32() - _last_valid_sample_ms >= 200) {
				_heater_integral = 0.0f;
				_heater_output = 0.0f;
				bsp_pwm_heater_set_duty(0.0f);
			}

			continue;
		}

		const uint32_t now_ms = k_uptime_get_32();
		_last_valid_sample_ms = now_ms;
		StoreSample(sample);
		atomic_inc(&_state.feedback_count);
		ProcessSample(sample);
		UpdateHeater(sample, now_ms);
	}
}

void AttitudeEstimator::init(const AttitudeEstimatorConfig &config)
{
	_config = config;
	_estimator.Init(_config.ekf);
	const int ret = _imu_device.Init();

	if (ret != 0) {
		return;
	}

	_heater_ready = bsp_pwm_heater_init() == 0;
	_last_valid_sample_ms = k_uptime_get_32();
	_last_heater_update_ms = _last_valid_sample_ms;
	k_thread_create(&_imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), task_trampoline, this, nullptr, nullptr,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_timer_init(&_imu_timer, timer_callback, nullptr);
	k_timer_start(&_imu_timer, K_MSEC(1), K_MSEC(1));
}

int AttitudeEstimator::GetSample(ImuSample &sample)
{
	const k_spinlock_key_t key = k_spin_lock(&_state.lock);

	if (!_state.has_sample) {
		k_spin_unlock(&_state.lock, key);
		return -ENODATA;
	}

	sample = _state.sample;
	k_spin_unlock(&_state.lock, key);
	return 0;
}

uint32_t AttitudeEstimator::FeedbackCount()
{
	return static_cast<uint32_t>(atomic_get(&_state.feedback_count));
}

void imu_port_init(const AttitudeEstimatorConfig &config)
{
	imu_port.init(config);
}

int imu_port_get_sample(ImuSample &sample)
{
	return imu_port.GetSample(sample);
}

uint32_t imu_port_feedback_count()
{
	return imu_port.FeedbackCount();
}
