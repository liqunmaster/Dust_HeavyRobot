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
    float process_noise_{0.001f};

    float measurement_noise_{0.05f};

    float estimate_{0.0f};

    float covariance_{1.0f};

    bool initialized_{false};
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
    KalmanAxis axis_[3];
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
            system_func_ = function;
        }

        void SetObserveFunc(ObserveFunc function)
        {
            observe_func_ = function;
        }

        void SetState(const State &state)
        {
            state_ = state;
            predicted_state_ = state;
        }

        void SetCovariance(const Cov &covariance)
        {
            covariance_ = covariance;
            predicted_covariance_ = covariance;
        }

        void SetGainRowScale(int row, float scale);
        void SetCorrectionLimit(int row, float limit);

        bool Predict(const Ctrl &control, const Cov &process_noise);

        bool Update(const Obs &measurement, const ObsCov &measurement_noise, float max_chi2 = std::numeric_limits<float>::infinity(), float gain_scale = 1.0f);

        void UsePrediction();

        void FadePredictedVariance(int index, float lambda, float max_variance);

        const State &GetX() const
        {
            return state_;
        }

        const Cov &GetP() const
        {
            return covariance_;
        }

        float GetChi2() const
        {
            return chi2_;
        }

        private:
        SystemFunc system_func_{nullptr};

        ObserveFunc observe_func_{nullptr};

        State state_{State::Zero()};

        State predicted_state_{State::Zero()};

        Cov covariance_{Cov::Identity()};

        Cov predicted_covariance_{Cov::Identity()};

        State gain_row_scale_{State::Ones()};

        State correction_limit_{State::Constant(std::numeric_limits<float>::infinity())};

        float chi2_{0.0f};
    };

    extern template class ExtendedKalman<6, 3, 4>;
}
