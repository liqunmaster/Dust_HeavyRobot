#pragma once
#include "Publication.hpp"
#include <uORB/topics/sensor_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include "time.hpp"

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "icm42688p_hxy.hpp"
#include "ins.hpp"
#include "type_math.hpp"

using ImuSample = Icm42688pHxySample;

struct AttitudeEstimatorConfig {
	alg::attitude::QuaternionEKF::Config ekf{};
	float accel_m_s2_per_count{0.0f};
	float gyro_rad_s_per_count{0.0f};
};

class AttitudeEstimator
{
public:
	void init(const AttitudeEstimatorConfig &config);

	int GetSample(ImuSample &sample);

	uint32_t FeedbackCount();

private:
	AttitudeEstimatorConfig _config{};
	Ins _estimator{};
	uORB::Publication<sensor_imu_s> _sample_pub{ORB_ID(sensor_imu)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	struct ImuState {
		struct k_spinlock lock;
		ImuSample sample{};
		atomic_t feedback_count{};
		bool has_sample{false};
	};

	static void timer_callback(struct k_timer *arg1);

	void StoreSample(const ImuSample &sample);

	void ProcessSample(const ImuSample &sample);

	void UpdateHeater(const ImuSample &sample, uint32_t now_ms);

	static void task_trampoline(void *arg1, void *arg2, void *arg3);

	void Run();

	ImuState _state{};
	Icm42688pHxy _imu_device{};
	struct k_thread _imu_thread {};
	struct k_timer _imu_timer {};
	bool _heater_ready = false;
	uint32_t _last_heater_update_ms = 0;
	uint32_t _last_valid_sample_ms = 0;
	float _heater_integral = 0.0f;
	float _heater_output = 0.0f;
	uint32_t _last_sample_cycles = 0;
	bool _have_sample_time = false;
};

void imu_port_init(const AttitudeEstimatorConfig &config);

int imu_port_get_sample(ImuSample &sample);

uint32_t imu_port_feedback_count();
