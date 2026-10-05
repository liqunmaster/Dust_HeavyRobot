#pragma once

#include <cstdint>

namespace alg
{
    class LeastSquares final
    {
        public:
        static constexpr uint16_t kMaxOrder = 16U;

        uint16_t Order = 0U;

        uint32_t Count = 0U;

        float x[kMaxOrder]{};

        float y[kMaxOrder]{};

        float k = 0.0F;

        float b = 0.0F;

        float StandardDeviation = 0.0F;

        void init(uint16_t order);

        void reset() { init(Order); }

        void add(float deltax, float y_sample);

        void update(float deltax, float y_sample) { add(deltax, y_sample); }

        bool fit();
        float derivative(float deltax, float y_sample);

        float smooth(float deltax, float y_sample);

        float last_derivative() const noexcept { return k; }

        float last_smooth() const noexcept;

        uint16_t order() const noexcept { return Order; }

        uint32_t count() const noexcept { return Count; }

        private:
        void shift_and_append(float deltax, float y_sample);
    };

    using OrdinaryLeastSquares = LeastSquares;

    void OLS_Init(LeastSquares *least_squares, uint16_t order);

    void OLS_Update(LeastSquares *least_squares, float deltax, float y);

    float OLS_Derivative(LeastSquares *least_squares, float deltax, float y);

    float OLS_Smooth(LeastSquares *least_squares, float deltax, float y);

    float Get_OLS_Derivative(const LeastSquares *least_squares);

    float Get_OLS_Smooth(const LeastSquares *least_squares);

}

using Ordinary_Least_Squares_t = alg::LeastSquares;

using LeastSquares = alg::LeastSquares;

using alg::Get_OLS_Derivative;

using alg::Get_OLS_Smooth;

using alg::OLS_Derivative;

using alg::OLS_Init;

using alg::OLS_Smooth;

using alg::OLS_Update;
