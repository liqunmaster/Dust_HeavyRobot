#include "kalman.hpp"

KalmanAxis::KalmanAxis(float process_noise, float measurement_noise)
{
	Configure(process_noise, measurement_noise);
}

void KalmanAxis::Configure(float process_noise, float measurement_noise)
{
	_process_noise = process_noise > 0.0f ? process_noise : 0.001f;
	_measurement_noise = measurement_noise > 0.0f ? measurement_noise : 0.05f;
	Reset();
}

void KalmanAxis::Reset(float value)
{
	_estimate = value;
	_covariance = 1.0f;
	_initialized = false;
}

float KalmanAxis::Update(float measurement)
{
	if (!std::isfinite(measurement)) {
		return _estimate;
	}

	if (!_initialized) {
		_estimate = measurement;
		_initialized = true;
		return _estimate;
	}

	_covariance += _process_noise;
	const float gain = _covariance / (_covariance + _measurement_noise);
	_estimate += gain * (measurement - _estimate);
	_covariance *= 1.0f - gain;
	return _estimate;
}

KalmanVector3::KalmanVector3(float process_noise, float measurement_noise)
{
	Configure(process_noise, measurement_noise);
}

void KalmanVector3::Configure(float process_noise, float measurement_noise)
{
	for (KalmanAxis &axis : _axis) {
		axis.Configure(process_noise, measurement_noise);
	}
}

void KalmanVector3::Reset(const float value[3])
{
	for (size_t axis = 0; axis < 3; ++axis) {
		_axis[axis].Reset(value ? value[axis] : 0.0f);
	}
}

void KalmanVector3::Update(const float measurement[3], float filtered[3])
{
	if (!measurement || !filtered) {
		return;
	}

	for (size_t axis = 0; axis < 3; ++axis) {
		filtered[axis] = _axis[axis].Update(measurement[axis]);
	}
}

namespace alg::filter
{
template <int NX, int NZ, int NU>

void ExtendedKalman<NX, NZ, NU>::Init(const State &initial_state, const Cov &initial_covariance)
{
	_state = _predicted_state = initial_state;
	_covariance = _predicted_covariance = initial_covariance;
	_gain_row_scale.setOnes();
	_correction_limit.setConstant(std::numeric_limits<float>::infinity());
	_chi2 = 0.0f;
}

template <int NX, int NZ, int NU>

void ExtendedKalman<NX, NZ, NU>::SetGainRowScale(int row, float scale)
{
	if (row >= 0 && row < NX && std::isfinite(scale)) {
		_gain_row_scale(row) = std::clamp(scale, 0.0f, 1.0f);
	}
}

template <int NX, int NZ, int NU>

void ExtendedKalman<NX, NZ, NU>::SetCorrectionLimit(int row, float limit)
{
	if (row >= 0 && row < NX && limit >= 0.0f) {
		_correction_limit(row) = limit;
	}
}

template <int NX, int NZ, int NU>

bool ExtendedKalman<NX, NZ, NU>::Predict(const Ctrl &control, const Cov &process_noise)
{
	if (!_system_func || !control.allFinite() || !process_noise.allFinite()) {
		return false;
	}

	Cov jacobian;
	_system_func(_state, control, _predicted_state, jacobian);

	if (!_predicted_state.allFinite() || !jacobian.allFinite()) {
		return false;
	}

	_predicted_covariance = jacobian * _covariance * jacobian.transpose() + process_noise;
	_predicted_covariance = 0.5f * (_predicted_covariance + _predicted_covariance.transpose().eval());

	if (!_predicted_covariance.allFinite()) {
		return false;
	}

	UsePrediction();
	return true;
}

template <int NX, int NZ, int NU>

bool ExtendedKalman<NX, NZ, NU>::Update(const Obs &measurement, const ObsCov &measurement_noise, float max_chi2,
					float gain_scale)
{
	if (!_observe_func || !measurement.allFinite() || !measurement_noise.allFinite()) {
		return false;
	}

	Obs predicted_measurement;
	ObsMat jacobian;
	_observe_func(_predicted_state, predicted_measurement, jacobian);

	if (!predicted_measurement.allFinite() || !jacobian.allFinite()) {
		return false;
	}

	const ObsCov innovation_covariance = jacobian * _predicted_covariance * jacobian.transpose() + measurement_noise;
	const Eigen::LDLT<ObsCov> solver(innovation_covariance);

	if (solver.info() != Eigen::Success || !solver.vectorD().allFinite() || (solver.vectorD().array() <= 0.0f).any()) {
		return false;
	}

	const Obs innovation = measurement - predicted_measurement;
	_chi2 = innovation.dot(solver.solve(innovation));

	if (!std::isfinite(_chi2) || _chi2 < 0.0f || _chi2 > max_chi2) {
		return false;
	}

	Eigen::Matrix<float, NX, NZ> gain = solver.solve((_predicted_covariance *
					    jacobian.transpose()).transpose()).transpose();
	gain *= std::clamp(gain_scale, 0.0f, 1.0f);

	for (int row = 0; row < NX; ++row) {
		gain.row(row) *= _gain_row_scale(row);
		const float correction = gain.row(row).dot(innovation);

		if (std::fabs(correction) > _correction_limit(row)) {
			gain.row(row) *= _correction_limit(row) / std::fabs(correction);
		}
	}

	const State updated_state = _predicted_state + gain * innovation;
	const Cov residual = Cov::Identity() - gain * jacobian;
	Cov updated_covariance = residual * _predicted_covariance * residual.transpose() + gain * measurement_noise *
				 gain.transpose();
	updated_covariance = 0.5f * (updated_covariance + updated_covariance.transpose().eval());

	if (!updated_state.allFinite() || !updated_covariance.allFinite()) {
		return false;
	}

	_state = updated_state;
	_covariance = updated_covariance;
	return true;
}

template <int NX, int NZ, int NU>

void ExtendedKalman<NX, NZ, NU>::UsePrediction()
{
	_state = _predicted_state;
	_covariance = _predicted_covariance;
}

template <int NX, int NZ, int NU>

void ExtendedKalman<NX, NZ, NU>::FadePredictedVariance(int index, float lambda, float max_variance)
{
	if (index >= 0 && index < NX && lambda > 0.0f && lambda <= 1.0f && max_variance > 0.0f) {
		_predicted_covariance(index, index) = std::min(_predicted_covariance(index, index) / lambda, max_variance);
		UsePrediction();
	}
}

template class ExtendedKalman<6, 3, 4>;
}
