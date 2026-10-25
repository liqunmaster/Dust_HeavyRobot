#include "quaternion.hpp"

namespace
{

float WrapDegrees(float value)
{
	value = std::remainder(value, 360.0f);
	return value == -180.0f ? 180.0f : value;
}
}

namespace alg::attitude
{

void QuaternionEKF::Init(const Config &config)
{
	_config = config;
	_config.lambda = std::clamp(_config.lambda, 0.01f, 1.0f);
	_state = {};
	_state.init = true;
	_static_count = 0;
	_static_z_sum = 0.0f;
	_static_z_count = 0;
	_previous_yaw = 0.0f;
	_yaw_turns = 0;
	_previous_q[0] = 1.0f;
	_previous_q[1] = _previous_q[2] = _previous_q[3] = 0.0f;
	_have_previous_q = false;

	EKF::State initial = EKF::State::Zero();
	initial(0) = 1.0f;
	EKF::Cov covariance = EKF::Cov::Identity();
	covariance.topLeftCorner<4, 4>() *= 1e-3f;
	covariance.bottomRightCorner<2, 2>() *= 0.1f;
	_ekf.Init(initial, covariance);
	_ekf.SetSystemFunc(SystemFunc);
	_ekf.SetObserveFunc(ObserveFunc);
	_ekf.SetGainRowScale(3, 0.0f);
}

void QuaternionEKF::InitFromAccel(const Sample &sample)
{
	const float norm = math::sqrt(sample.accel[0] * sample.accel[0] + sample.accel[1] * sample.accel[1] + sample.accel[2] *
				      sample.accel[2]);

	if (!std::isfinite(norm) || norm < 1e-3f) {
		return;
	}

	const float roll = math::atan2(sample.accel[1], sample.accel[2]);
	const float pitch = -math::asin_clamped(sample.accel[0] / norm);
	const float cr = math::cos(0.5f * roll), sr = math::sin(0.5f * roll);
	const float cp = math::cos(0.5f * pitch), sp = math::sin(0.5f * pitch);
	EKF::State initial = EKF::State::Zero();
	initial << cp *cr, cp *sr, sp *cr, -sp *sr, 0.0f, 0.0f;
	_ekf.SetState(initial);

	for (int axis = 0; axis < 3; ++axis) {
		_state.a[axis] = sample.accel[axis];
	}
}

void QuaternionEKF::Update(const Sample &sample)
{
	if (!_state.init) {
		Init(Config{});
	}

	if (!std::isfinite(sample.dt) || sample.dt <= 0.0f || sample.dt > 0.1f) {
		return;
	}

	for (int axis = 0; axis < 3; ++axis) {
		if (!std::isfinite(sample.gyro[axis]) || !std::isfinite(sample.accel[axis])) {
			return;
		}
	}

	_state.dt = sample.dt;

	if (_state.upd_cnt == 0) {
		InitFromAccel(sample);
	}

	const float accel_weight = sample.dt / (sample.dt + std::max(_config.alpha, 0.0f));

	for (int axis = 0; axis < 3; ++axis) {
		_state.a[axis] += accel_weight * (sample.accel[axis] - _state.a[axis]);
		_state.w[axis] = sample.gyro[axis] - _state.bg[axis];
	}

	_state.a_norm = math::sqrt(_state.a[0] * _state.a[0] + _state.a[1] * _state.a[1] + _state.a[2] * _state.a[2]);
	_state.w_norm = math::sqrt(_state.w[0] * _state.w[0] + _state.w[1] * _state.w[1] + _state.w[2] * _state.w[2]);
	_state.stable = _state.w_norm < _config.w_stable_th && std::fabs(_state.a_norm - _config.a_ref) < _config.a_tol;

	if (_state.stable && std::fabs(_state.w[2]) < 0.02f) {
		if (_static_count < 100) {
			++_static_count;
		}

		if (_static_count == 100) {
			_static_z_sum += sample.gyro[2];

			if (++_static_z_count == 500) {
				_state.bg[2] = _static_z_sum / _static_z_count;
				_static_z_sum = 0.0f;
				_static_z_count = 0;
			}
		}

	} else {
		_static_count = 0;
		_static_z_sum = 0.0f;
		_static_z_count = 0;
	}

	_state.w[2] = sample.gyro[2] - _state.bg[2];

	if (_state.upd_cnt > 0) {
		EKF::Ctrl control;
		control << _state.w[0] * sample.dt, _state.w[1] * sample.dt,
			_state.w[2] * sample.dt, sample.dt;
		EKF::Cov process_noise = EKF::Cov::Zero();
		process_noise.diagonal() << _config.qq *sample.dt, _config.qq *sample.dt,
				       _config.qq *sample.dt, _config.qq *sample.dt,
				       _config.qb *sample.dt, _config.qb *sample.dt;

		if (!_ekf.Predict(control, process_noise)) {
			return;
		}

		_ekf.FadePredictedVariance(4, _config.lambda, _config.pb_limit);
		_ekf.FadePredictedVariance(5, _config.lambda, _config.pb_limit);
		_ekf.SetCorrectionLimit(4, _config.bias_limit * sample.dt);
		_ekf.SetCorrectionLimit(5, _config.bias_limit * sample.dt);

		if (std::isfinite(_state.a_norm) && _state.a_norm > 1e-3f && std::fabs(_state.a_norm - _config.a_ref) < _config.a_tol) {
			EKF::Obs gravity;
			gravity << _state.a[0] / _state.a_norm,
				_state.a[1] / _state.a_norm,
				_state.a[2] / _state.a_norm;
			const EKF::ObsCov measurement_noise = EKF::ObsCov::Identity() * _config.r;
			const float gate = _state.converg ? _config.chi2_th : std::numeric_limits<float>::infinity();
			bool corrected = _ekf.Update(gravity, measurement_noise, gate);
			_state.chi2 = _ekf.GetChi2();

			if (corrected) {
				_state.converg = true;
				_state.err_cnt = 0;

			} else if (_state.converg && _state.stable && std::isfinite(_state.chi2) && _state.chi2 > _config.chi2_th) {
				if (++_state.err_cnt > _config.div_limit) {
					_state.converg = false;
					_state.err_cnt = 0;
				}
			}
		}
	}

	EKF::State corrected = _ekf.GetX();

	if (!Normalize(corrected)) {
		return;
	}

	_ekf.SetState(corrected);

	for (int axis = 0; axis < 4; ++axis) {
		_state.q[axis] = corrected(axis);
	}

	_state.bg[0] = corrected(4);
	_state.bg[1] = corrected(5);
	UpdateAngles();
	++_state.upd_cnt;
}

void QuaternionEKF::SystemFunc(const EKF::State &state, const EKF::Ctrl &control, EKF::State &predicted,
			       EKF::Cov &jacobian)
{
	const float q0 = state(0), q1 = state(1), q2 = state(2), q3 = state(3);
	const float hx = 0.5f * control(0), hy = 0.5f * control(1);
	const float hz = 0.5f * control(2), hdt = 0.5f * control(3);
	predicted = state;
	predicted(0) = q0 - q1 * hx - q2 * hy - q3 * hz;
	predicted(1) = q1 + q0 * hx + q2 * hz - q3 * hy;
	predicted(2) = q2 + q0 * hy - q1 * hz + q3 * hx;
	predicted(3) = q3 + q0 * hz + q1 * hy - q2 * hx;
	Normalize(predicted);

	jacobian.setIdentity();
	jacobian(0, 1) = -hx;
	jacobian(0, 2) = -hy;
	jacobian(0, 3) = -hz;
	jacobian(1, 0) = hx;
	jacobian(1, 2) = hz;
	jacobian(1, 3) = -hy;
	jacobian(2, 0) = hy;
	jacobian(2, 1) = -hz;
	jacobian(2, 3) = hx;
	jacobian(3, 0) = hz;
	jacobian(3, 1) = hy;
	jacobian(3, 2) = -hx;
	jacobian(0, 4) = q1 * hdt;
	jacobian(0, 5) = q2 * hdt;
	jacobian(1, 4) = -q0 * hdt;
	jacobian(1, 5) = q3 * hdt;
	jacobian(2, 4) = -q3 * hdt;
	jacobian(2, 5) = -q0 * hdt;
	jacobian(3, 4) = q2 * hdt;
	jacobian(3, 5) = -q1 * hdt;
}

void QuaternionEKF::ObserveFunc(const EKF::State &state, EKF::Obs &predicted, EKF::ObsMat &jacobian)
{
	const float q0 = state(0), q1 = state(1), q2 = state(2), q3 = state(3);
	predicted << 2.0f * (q1 * q3 - q0 * q2),
		  2.0f * (q0 * q1 + q2 * q3),
		  q0 *q0 - q1 *q1 - q2 *q2 + q3 *q3;
	jacobian.setZero();
	jacobian(0, 0) = -2.0f * q2;
	jacobian(0, 1) = 2.0f * q3;
	jacobian(0, 2) = -2.0f * q0;
	jacobian(0, 3) = 2.0f * q1;
	jacobian(1, 0) = 2.0f * q1;
	jacobian(1, 1) = 2.0f * q0;
	jacobian(1, 2) = 2.0f * q3;
	jacobian(1, 3) = 2.0f * q2;
	jacobian(2, 0) = 2.0f * q0;
	jacobian(2, 1) = -2.0f * q1;
	jacobian(2, 2) = -2.0f * q2;
	jacobian(2, 3) = 2.0f * q3;
}

bool QuaternionEKF::Normalize(EKF::State &state)
{
	const float length = math::sqrt(state.head<4>().squaredNorm());

	if (!std::isfinite(length) || length < 1e-6f) {
		return false;
	}

	state.head<4>() /= length;
	return true;
}

void QuaternionEKF::UpdateAngles()
{
	const float q0 = _state.q[0], q1 = _state.q[1];
	const float q2 = _state.q[2], q3 = _state.q[3];
	const float r11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
	const float r21 = 2.0f * (q1 * q2 + q0 * q3);
	const float r31 = 2.0f * (q1 * q3 - q0 * q2);
	const float r32 = 2.0f * (q2 * q3 + q0 * q1);
	const float r33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

	const float yaw = WrapDegrees(math::radians_to_degrees(math::atan2(r21, r11)));
	const float pitch = math::radians_to_degrees(math::asin_clamped(-r31));
	const float roll = WrapDegrees(math::radians_to_degrees(math::atan2(r32, r33)));
	const float alternate_yaw = WrapDegrees(yaw + 180.0f);
	const float alternate_pitch = pitch >= 0.0f ? 180.0f - pitch : -180.0f - pitch;
	const float alternate_roll = WrapDegrees(roll + 180.0f);

	bool use_alternate = false;
	float current_q[4] {q0, q1, q2, q3};

	if (_have_previous_q) {
		float dot = 0.0f;

		for (int axis = 0; axis < 4; ++axis) {
			dot += _previous_q[axis] * current_q[axis];
		}

		if (dot < 0.0f) {
			for (float &component : current_q) {
				component = -component;
			}

			dot = -dot;
		}

		const float delta_y = _previous_q[0] * current_q[2] + _previous_q[1] * current_q[3] - _previous_q[2] * current_q[0] -
				      _previous_q[3] * current_q[1];
		const float expected_pitch = WrapDegrees(_state.pitch + math::radians_to_degrees(2.0f * math::atan2(delta_y, dot)));
		const float direct_error = std::fabs(WrapDegrees(pitch - expected_pitch));
		const float alternate_error = std::fabs(WrapDegrees(alternate_pitch - expected_pitch));

		if (std::fabs(direct_error - alternate_error) > 0.05f) {
			use_alternate = alternate_error < direct_error;

		} else {
			const float direct_yaw = WrapDegrees(yaw - _state.yaw);
			const float direct_roll = WrapDegrees(roll - _state.roll);
			const float other_yaw = WrapDegrees(alternate_yaw - _state.yaw);
			const float other_roll = WrapDegrees(alternate_roll - _state.roll);
			use_alternate = other_yaw * other_yaw + other_roll * other_roll < direct_yaw * direct_yaw + direct_roll * direct_roll;
		}
	}

	_state.yaw = use_alternate ? alternate_yaw : yaw;
	_state.pitch = use_alternate ? alternate_pitch : pitch;
	_state.roll = use_alternate ? alternate_roll : roll;

	for (int axis = 0; axis < 4; ++axis) {
		_previous_q[axis] = current_q[axis];
	}

	_have_previous_q = true;

	const float difference = _state.yaw - _previous_yaw;

	if (difference > 180.0f) {
		--_yaw_turns;
	}

	if (difference < -180.0f) {
		++_yaw_turns;
	}

	_state.yaw_sum = 360.0f * static_cast<float>(_yaw_turns) + _state.yaw;
	_previous_yaw = _state.yaw;
}
}
