#pragma once

#pragma push_macro("__R")
#undef __R

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>

#pragma pop_macro("__R")

class KalmanAxis final
{
public:
	KalmanAxis() = default;

	KalmanAxis(float process_noise, float measurement_noise);

	void Configure(float process_noise, float measurement_noise);

	void Reset(float value = 0.0f);

	float Update(float measurement);

private:
	float _process_noise{0.001f};

	float _measurement_noise{0.05f};

	float _estimate{0.0f};

	float _covariance{1.0f};

	bool _initialized{false};
};

class KalmanVector3 final
{
public:
	KalmanVector3() = default;

	KalmanVector3(float process_noise, float measurement_noise);

	void Configure(float process_noise, float measurement_noise);

	void Reset(const float value[3] = nullptr);

	void Update(const float measurement[3], float filtered[3]);

private:
	KalmanAxis _axis[3];
};

namespace alg::filter
{

template <int NX, int NZ, int NU>
class ExtendedKalman final
{
public:
	using State = Eigen::Matrix<float, NX, 1>;

	using Cov = Eigen::Matrix<float, NX, NX>;

	using Obs = Eigen::Matrix<float, NZ, 1>;

	using ObsCov = Eigen::Matrix<float, NZ, NZ>;

	using Ctrl = Eigen::Matrix<float, NU, 1>;

	using ObsMat = Eigen::Matrix<float, NZ, NX>;

	using SystemFunc = void (*)(const State &, const Ctrl &, State &, Cov &);

	using ObserveFunc = void (*)(const State &, Obs &, ObsMat &);

	void Init(const State &initial_state, const Cov &initial_covariance);

	void SetSystemFunc(SystemFunc function)
	{
		_system_func = function;
	}

	void SetObserveFunc(ObserveFunc function)
	{
		_observe_func = function;
	}

	void SetState(const State &state)
	{
		_state = state;
		_predicted_state = state;
	}

	void SetCovariance(const Cov &covariance)
	{
		_covariance = covariance;
		_predicted_covariance = covariance;
	}

	void SetGainRowScale(int row, float scale);
	void SetCorrectionLimit(int row, float limit);

	bool Predict(const Ctrl &control, const Cov &process_noise);

	bool Update(const Obs &measurement, const ObsCov &measurement_noise,
		    float max_chi2 = std::numeric_limits<float>::infinity(), float gain_scale = 1.0f);

	void UsePrediction();

	void FadePredictedVariance(int index, float lambda, float max_variance);

	const State &GetX() const
	{
		return _state;
	}

	const Cov &GetP() const
	{
		return _covariance;
	}

	float GetChi2() const
	{
		return _chi2;
	}

private:
	SystemFunc _system_func{nullptr};

	ObserveFunc _observe_func{nullptr};

	State _state{State::Zero()};

	State _predicted_state{State::Zero()};

	Cov _covariance{Cov::Identity()};

	Cov _predicted_covariance{Cov::Identity()};

	State _gain_row_scale{State::Ones()};

	State _correction_limit{State::Constant(std::numeric_limits<float>::infinity())};

	float _chi2{0.0f};
};

extern template class ExtendedKalman<6, 3, 4>;
}
