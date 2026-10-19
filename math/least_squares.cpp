#include "least_squares.hpp"

#include <cmath>
#include <cstring>

namespace alg
{

    void LeastSquares::Init(uint16_t order)
    {
        Order = order > kMaxOrder ? kMaxOrder : order;
        Count = 0;
        k = 0.0f;
        b = 0.0f;
        StandardDeviation = 0.0f;
        std::memset(x, 0, sizeof(x));
        std::memset(y, 0, sizeof(y));
    }

    void LeastSquares::ShiftAndAppend(float deltax, float y_sample)
    {
        if (Order == 0 || !std::isfinite(deltax) || !std::isfinite(y_sample)) {
            return;
        }
        if (Count == 0) {
            x[0] = 0.0f;
            y[0] = y_sample;
            Count = 1;
            return;
        }
        const float step = deltax > 0.0f ? deltax : 1.0f;
        if (Count < Order) {
            const uint16_t last = static_cast<uint16_t>(Count);
            x[last] = x[last - 1] + step;
            y[last] = y_sample;
            ++Count;
            return;
        }
        for (uint16_t index = 0; index + 1 < Order; ++index) {
            x[index] = x[index + 1] - x[1];
            y[index] = y[index + 1];
        }
        x[Order - 1] = x[Order - 2] + step;
        y[Order - 1] = y_sample;
    }

    bool LeastSquares::Fit()
    {
        if (Count < 2 || Order < 2) {
            k = 0.0f;
            b = Count == 1 ? y[0] : 0.0f;
            StandardDeviation = 0.0f;
            return false;
        }
        const uint16_t count = static_cast<uint16_t>(Count < Order ? Count : Order);
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_xx = 0.0f;
        float sum_xy = 0.0f;
        const uint16_t first = 0;
        for (uint16_t index = first; index < count; ++index) {
            sum_x += x[index];
            sum_y += y[index];
            sum_xx += x[index] * x[index];
            sum_xy += x[index] * y[index];
        }
        const float denominator = static_cast<float>(count) * sum_xx - sum_x * sum_x;
        if (std::fabs(denominator) < 1.0e-12f) {
            k = 0.0f;
            b = sum_y / static_cast<float>(count);
            StandardDeviation = 0.0f;
            return false;
        }
        k = (static_cast<float>(count) * sum_xy - sum_x * sum_y) / denominator;
        b = (sum_xx * sum_y - sum_x * sum_xy) / denominator;
        StandardDeviation = 0.0f;
        for (uint16_t index = first; index < count; ++index) {
            StandardDeviation += std::fabs(k * x[index] + b - y[index]);
        }
        StandardDeviation /= static_cast<float>(count);
        return std::isfinite(k) && std::isfinite(b);
    }

    void LeastSquares::Add(float deltax, float y_sample)
    {
        ShiftAndAppend(deltax, y_sample);
        Fit();
    }

    float LeastSquares::Derivative(float deltax, float y_sample)
    {
        Add(deltax, y_sample);
        return k;
    }

    float LeastSquares::Smooth(float deltax, float y_sample)
    {
        Add(deltax, y_sample);
        return LastSmooth();
    }

    float LeastSquares::LastSmooth() const noexcept
    {
        if (Count == 0) {
            return 0.0f;
        }
        const uint16_t index = static_cast<uint16_t>((Count < Order ? Count : Order) - 1);
        return k * x[index] + b;
    }

    void OLS_Init(LeastSquares *least_squares, uint16_t order)
    {
        if (least_squares != nullptr) {
            least_squares->Init(order);
        }
    }

    void OLS_Update(LeastSquares *least_squares, float deltax, float y)
    {
        if (least_squares != nullptr) {
            least_squares->Update(deltax, y);
        }
    }

    float OLS_Derivative(LeastSquares *least_squares, float deltax, float y)
    {
        return least_squares != nullptr ? least_squares->Derivative(deltax, y) : 0.0f;
    }

    float OLS_Smooth(LeastSquares *least_squares, float deltax, float y)
    {
        return least_squares != nullptr ? least_squares->Smooth(deltax, y) : 0.0f;
    }

    float Get_OLS_Derivative(const LeastSquares *least_squares)
    {
        return least_squares != nullptr ? least_squares->LastDerivative() : 0.0f;
    }

    float Get_OLS_Smooth(const LeastSquares *least_squares)
    {
        return least_squares != nullptr ? least_squares->LastSmooth() : 0.0f;
    }
}
