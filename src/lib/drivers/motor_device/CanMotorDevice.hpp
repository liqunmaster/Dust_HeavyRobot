#pragma once

#include "MotorDevice.hpp"
#include "motor_port.hpp"
#include "time.hpp"
#include "PublicationInstance.hpp"
#include <type_traits>
#include <string.h>

struct CanMotorBinding {
	CanfdDevice bus;
	uint8_t instance;
	uint8_t motor_id{1};
	float gear_ratio{1.0f};
	uint32_t feedback_id{0};
	uint32_t control_id{0};
	DmMitLimits limits{12.5f, 30.0f, 10.0f};
};

// 由板级持有的驱动适配器；协议代码位于控制模块之外。
template<typename Driver>
class CanMotorDevice final : public MotorDevice
{
public:
	CanMotorDevice(const CanMotorBinding &binding) :
		_binding(binding), _publication(ORB_ID(motor_feedback), binding.instance) {}

	int init() override
	{
		if (_initialized) { return 0; }

		int result;

		if constexpr(std::is_same_v<Driver, C620>) {
			result = _driver.Init(_binding.bus, static_cast<C620Id>(_binding.motor_id), _binding.gear_ratio);

		} else if constexpr(std::is_same_v<Driver, Cubemars>) {
			result = _driver.Init(_binding.bus, _binding.limits.position, _binding.limits.velocity, _binding.limits.torque);

		} else {
			result = _driver.Init(_binding.bus, _binding.motor_id, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,
					      _binding.limits, CANFD_PROTOCOL_CLASSIC, _binding.feedback_id, _binding.control_id);
		}

		if (result != 0) { return result; }

		if constexpr(std::is_same_v<Driver, DmMotor>) {
			result = canfd_port_bind(_driver, false);

		} else {
			result = canfd_port_bind(_driver);
		}

		if (result != 0) { return result; }

		result = platform_canfd_subscribe(_binding.bus, _driver.GetFeedbackId(),
						  CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC, _handle);
		_initialized = result == 0;
		return result;
	}

	bool refresh(uint32_t max_age_ms) override
	{
		PlatformCanfdSnapshot raw{};

		if (!_initialized || platform_canfd_latest(_handle, raw) != 0 || raw.length > canfd_max_data_length) {
			return false;
		}

		if (raw.sequence != _sequence) {
			_sequence = raw.sequence;
			CanfdFrame frame{};
			frame.id = _driver.GetFeedbackId();
			frame.id_type = CANFD_ID_STANDARD;
			frame.protocol = CANFD_PROTOCOL_CLASSIC;
			frame.length = raw.length;
			memcpy(frame.data, raw.data, raw.length);
			_valid = _driver.ProcessFeedback(frame) == 0;

			if (_valid) {
				const auto data = _driver.GetData();
				_measurement.timestamp = robot::source_time_us(raw.timestamp_ms);
				_measurement.device_id = frame.id;
				_measurement.bus_index = static_cast<uint8_t>(_binding.bus);
				_measurement.speed_rad_s = data.now_omega;
				_measurement.valid_count = _driver.GetFeedbackCount();

				if constexpr(std::is_same_v<Driver, C620>) {
					_measurement.protocol_kind = 1;
					_measurement.angle_rad = data.now_angle;
					_measurement.current_a = data.now_current;

				} else {
					_measurement.protocol_kind = std::is_same_v<Driver, Cubemars> ? 3 : 2;

					if constexpr(std::is_same_v<Driver, Cubemars>) {
						_measurement.angle_rad = data.now_total_angle;

					} else {
						_measurement.angle_rad = data.now_angle;
						_measurement.motor_status = data.status;
					}

					_measurement.torque_nm = data.now_torque;
				}

				_publication.publish(_measurement);
			}
		}

		return _valid && k_uptime_get_32() - raw.timestamp_ms < max_age_ms;
	}

	const motor_feedback_s &measurement() const override { return _measurement; }
	MotorCounters counters() const override
	{
		CanfdStatistics stats{};
		bsp_canfd_get_statistics(_binding.bus, &stats);
		return {stats.tx_completed, stats.tx_errors, stats.rx_received, stats.rx_dropped};
	}
	int set_current(float current_a) override
	{
		if constexpr(std::is_same_v<Driver, C620>) { return _driver.SetCurrent(current_a); }

		return -ENOTSUP;
	}
	int send() override { return canfd_port_submit(_driver); }
	int send_torque(float torque_nm) override
	{
		if constexpr(!std::is_same_v<Driver, C620>) {
			const int result = _driver.SetMit(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
			return result == 0 ? send() : result;
		}

		return -ENOTSUP;
	}
	int enable() override
	{
		if constexpr(std::is_same_v<Driver, Cubemars>) { return canfd_port_enable(_driver); }

		return -ENOTSUP;
	}
	int recover(bool clear_error) override
	{
		if constexpr(std::is_same_v<Driver, DmMotor>) {
			CanfdFrame frame{};
			const int result = clear_error ? _driver.BuildClearErrorFrame(frame) : _driver.BuildEnableFrame(frame);
			return result == 0 ? canfd_port_send_once(_binding.bus, frame) : result;
		}

		return -ENOTSUP;
	}

private:
	CanMotorBinding _binding;
	Driver _driver{};
	PlatformCanfdRxHandle _handle{};
	uORB::PublicationInstance<motor_feedback_s> _publication;
	motor_feedback_s _measurement{};
	uint32_t _sequence{0};
	bool _valid{false};
	bool _initialized{false};
};
