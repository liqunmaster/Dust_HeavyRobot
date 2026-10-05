#include "kalman.hpp"

kalman_axis::kalman_axis(float process_noise, float measurement_noise)
{
    configure(process_noise, measurement_noise);
}

void kalman_axis::configure(float process_noise, float measurement_noise)
{
    process_noise_ = process_noise > 0.0F ? process_noise : 0.001F;
    measurement_noise_ = measurement_noise > 0.0F ? measurement_noise : 0.05F;
    reset();
}

void kalman_axis::reset(float value)
{
    estimate_ = value;
    covariance_ = 1.0F;
    initialized_ = false;
}

float kalman_axis::update(float measurement)
{
    if (!std::isfinite(measurement)) {
        return estimate_;
    }
    if (!initialized_) {
        estimate_ = measurement;
        initialized_ = true;
        return estimate_;
    }
    covariance_ += process_noise_;
    const float gain = covariance_ / (covariance_ + measurement_noise_);
    estimate_ += gain * (measurement - estimate_);
    covariance_ *= 1.0F - gain;
    return estimate_;
}

kalman_vector3::kalman_vector3(float process_noise, float measurement_noise)
{
    configure(process_noise, measurement_noise);
}

void kalman_vector3::configure(float process_noise, float measurement_noise)
{
    for (kalman_axis &axis : axis_) {
        axis.configure(process_noise, measurement_noise);
    }
}

void kalman_vector3::reset(const float value[3])
{
    for (size_t axis = 0; axis < 3; ++axis) {
        axis_[axis].reset(value ? value[axis] : 0.0F);
    }
}

void kalman_vector3::update(const float measurement[3], float filtered[3])
{
    if (!measurement || !filtered) {
        return;
    }
    for (size_t axis = 0; axis < 3; ++axis) {
        filtered[axis] = axis_[axis].update(measurement[axis]);
    }
}

namespace alg::filter
{
    template <int NX, int NZ, int NU>

    void ExtendedKalman<NX, NZ, NU>::init(const State &initial_state, const Cov &initial_covariance)
    {
        state_ = predicted_state_ = initial_state;
        covariance_ = predicted_covariance_ = initial_covariance;
        gain_row_scale_.setOnes();
        correction_limit_.setConstant(std::numeric_limits<float>::infinity());
        chi2_ = 0.0F;
    }

    template <int NX, int NZ, int NU>

    void ExtendedKalman<NX, NZ, NU>::set_gain_row_scale(int row, float scale)
    {
        if (row >= 0 && row < NX && std::isfinite(scale)) {
            gain_row_scale_(row) = std::clamp(scale, 0.0F, 1.0F);
        }
    }

    template <int NX, int NZ, int NU>

    void ExtendedKalman<NX, NZ, NU>::set_correction_limit(int row, float limit)
    {
        if (row >= 0 && row < NX && limit >= 0.0F) {
            correction_limit_(row) = limit;
        }
    }

    template <int NX, int NZ, int NU>

    bool ExtendedKalman<NX, NZ, NU>::predict(const Ctrl &control, const Cov &process_noise)
    {
        if (!system_func_ || !control.allFinite() || !process_noise.allFinite()) {
            return false;
        }
        Cov jacobian;
        system_func_(state_, control, predicted_state_, jacobian);
        if (!predicted_state_.allFinite() || !jacobian.allFinite()) {
            return false;
        }
        predicted_covariance_ = jacobian * covariance_ * jacobian.transpose() + process_noise;
        predicted_covariance_ = 0.5F * (predicted_covariance_ + predicted_covariance_.transpose().eval());
        if (!predicted_covariance_.allFinite()) {
            return false;
        }
        use_prediction();
        return true;
    }

    template <int NX, int NZ, int NU>

    bool ExtendedKalman<NX, NZ, NU>::update(const Obs &measurement, const ObsCov &measurement_noise, float max_chi2, float gain_scale)
    {
        if (!observe_func_ || !measurement.allFinite() || !measurement_noise.allFinite()) {
            return false;
        }
        Obs predicted_measurement;
        ObsMat jacobian;
        observe_func_(predicted_state_, predicted_measurement, jacobian);
        if (!predicted_measurement.allFinite() || !jacobian.allFinite()) {
            return false;
        }

        const ObsCov innovation_covariance = jacobian * predicted_covariance_ * jacobian.transpose() + measurement_noise;
        const Eigen::LDLT<ObsCov> solver(innovation_covariance);
        if (solver.info() != Eigen::Success || !solver.vectorD().allFinite() || (solver.vectorD().array() <= 0.0F).any()) {
            return false;
        }

        const Obs innovation = measurement - predicted_measurement;
        chi2_ = innovation.dot(solver.solve(innovation));
        if (!std::isfinite(chi2_) || chi2_ < 0.0F || chi2_ > max_chi2) {
            return false;
        }

        Eigen::Matrix<float, NX, NZ> gain = solver.solve((predicted_covariance_ * jacobian.transpose()).transpose()).transpose();
        gain *= std::clamp(gain_scale, 0.0F, 1.0F);
        for (int row = 0; row < NX; ++row) {
            gain.row(row) *= gain_row_scale_(row);
            const float correction = gain.row(row).dot(innovation);
            if (std::fabs(correction) > correction_limit_(row)) {
                gain.row(row) *= correction_limit_(row) / std::fabs(correction);
            }
        }

        const State updated_state = predicted_state_ + gain * innovation;
        const Cov residual = Cov::Identity() - gain * jacobian;
        Cov updated_covariance = residual * predicted_covariance_ * residual.transpose() + gain * measurement_noise * gain.transpose();
        updated_covariance = 0.5F * (updated_covariance + updated_covariance.transpose().eval());
        if (!updated_state.allFinite() || !updated_covariance.allFinite()) {
            return false;
        }
        state_ = updated_state;
        covariance_ = updated_covariance;
        return true;
    }

    template <int NX, int NZ, int NU>

    void ExtendedKalman<NX, NZ, NU>::use_prediction()
    {
        state_ = predicted_state_;
        covariance_ = predicted_covariance_;
    }

    template <int NX, int NZ, int NU>

    void ExtendedKalman<NX, NZ, NU>::fade_predicted_variance(int index, float lambda, float max_variance)
    {
        if (index >= 0 && index < NX && lambda > 0.0F && lambda <= 1.0F && max_variance > 0.0F) {
            predicted_covariance_(index, index) = std::min(predicted_covariance_(index, index) / lambda, max_variance);
            use_prediction();
        }
    }

    template class ExtendedKalman<6, 3, 4>;
}
