#include "least_squares.hpp"

#include <cmath>
#include <cstring>

namespace alg
{

    void LeastSquares::init(uint16_t order)
    {
        Order = order > kMaxOrder ? kMaxOrder : order;
        Count = 0U;
        k = 0.0F;
        b = 0.0F;
        StandardDeviation = 0.0F;
        std::memset(x, 0, sizeof(x));
        std::memset(y, 0, sizeof(y));
    }

    void LeastSquares::shift_and_append(float deltax, float y_sample)
    {
        if (Order == 0U || !std::isfinite(deltax) || !std::isfinite(y_sample)) {
            return;
        }
        if (Count == 0U) {
            x[0] = 0.0F;
            y[0] = y_sample;
            Count = 1U;
            return;
        }
        const float step = deltax > 0.0F ? deltax : 1.0F;
        if (Count < Order) {
            const uint16_t last = static_cast<uint16_t>(Count);
            x[last] = x[last - 1U] + step;
            y[last] = y_sample;
            ++Count;
            return;
        }
        for (uint16_t index = 0U; index + 1U < Order; ++index) {
            x[index] = x[index + 1U] - x[1U];
            y[index] = y[index + 1U];
        }
        x[Order - 1U] = x[Order - 2U] + step;
        y[Order - 1U] = y_sample;
    }

    bool LeastSquares::fit()
    {
        if (Count < 2U || Order < 2U) {
            k = 0.0F;
            b = Count == 1U ? y[0] : 0.0F;
            StandardDeviation = 0.0F;
            return false;
        }
        const uint16_t count = static_cast<uint16_t>(Count < Order ? Count : Order);
        float sum_x = 0.0F;
        float sum_y = 0.0F;
        float sum_xx = 0.0F;
        float sum_xy = 0.0F;
        const uint16_t first = 0U;
        for (uint16_t index = first; index < count; ++index) {
            sum_x += x[index];
            sum_y += y[index];
            sum_xx += x[index] * x[index];
            sum_xy += x[index] * y[index];
        }
        const float denominator = static_cast<float>(count) * sum_xx - sum_x * sum_x;
        if (std::fabs(denominator) < 1.0e-12F) {
            k = 0.0F;
            b = sum_y / static_cast<float>(count);
            StandardDeviation = 0.0F;
            return false;
        }
        k = (static_cast<float>(count) * sum_xy - sum_x * sum_y) / denominator;
        b = (sum_xx * sum_y - sum_x * sum_xy) / denominator;
        StandardDeviation = 0.0F;
        for (uint16_t index = first; index < count; ++index) {
            StandardDeviation += std::fabs(k * x[index] + b - y[index]);
        }
        StandardDeviation /= static_cast<float>(count);
        return std::isfinite(k) && std::isfinite(b);
    }

    void LeastSquares::add(float deltax, float y_sample)
    {
        shift_and_append(deltax, y_sample);
        fit();
    }

    float LeastSquares::derivative(float deltax, float y_sample)
    {
        add(deltax, y_sample);
        return k;
    }

    float LeastSquares::smooth(float deltax, float y_sample)
    {
        add(deltax, y_sample);
        return last_smooth();
    }

    float LeastSquares::last_smooth() const noexcept
    {
        if (Count == 0U) {
            return 0.0F;
        }
        const uint16_t index = static_cast<uint16_t>((Count < Order ? Count : Order) - 1U);
        return k * x[index] + b;
    }

    void OLS_Init(LeastSquares *least_squares, uint16_t order)
    {
        if (least_squares != nullptr) {
            least_squares->init(order);
        }
    }

    void OLS_Update(LeastSquares *least_squares, float deltax, float y)
    {
        if (least_squares != nullptr) {
            least_squares->update(deltax, y);
        }
    }

    float OLS_Derivative(LeastSquares *least_squares, float deltax, float y)
    {
        return least_squares != nullptr ? least_squares->derivative(deltax, y) : 0.0F;
    }

    float OLS_Smooth(LeastSquares *least_squares, float deltax, float y)
    {
        return least_squares != nullptr ? least_squares->smooth(deltax, y) : 0.0F;
    }

    float Get_OLS_Derivative(const LeastSquares *least_squares)
    {
        return least_squares != nullptr ? least_squares->last_derivative() : 0.0F;
    }

    float Get_OLS_Smooth(const LeastSquares *least_squares)
    {
        return least_squares != nullptr ? least_squares->last_smooth() : 0.0F;
    }

}
