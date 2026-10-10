#pragma once

#include <cstdint>

namespace alg
{
    // 最小二乘线性回归：滑窗拟合直线 y = kx + b，用于求平滑值与斜率
    class LeastSquares final
    {
        public:
        static constexpr uint16_t kMaxOrder = 16;

        uint16_t Order = 0;

        uint32_t Count = 0;

        float x[kMaxOrder]{};

        float y[kMaxOrder]{};

        float k = 0.0f;

        float b = 0.0f;

        float StandardDeviation = 0.0f;

        void init(uint16_t order);

        /**
         * @brief 复位为初始状态（沿用当前阶数）
        */
        void reset() { init(Order); }

        void add(float deltax, float y_sample);

        /**
         * @brief 加入采样点并重新拟合
         *
         * @param deltax 相邻采样点的横坐标间隔
         * @param y_sample 新采样点的纵坐标值
        */
        void update(float deltax, float y_sample) { add(deltax, y_sample); }

        bool fit();
        float derivative(float deltax, float y_sample);

        float smooth(float deltax, float y_sample);

        /**
         * @brief 获取最近一次拟合斜率
         *
         * @return 拟合斜率 k
        */
        float last_derivative() const noexcept { return k; }

        float last_smooth() const noexcept;

        /**
         * @brief 获取回归阶数
         *
         * @return 阶数 Order
        */
        uint16_t order() const noexcept { return Order; }

        /**
         * @brief 获取已累计采样点数
         *
         * @return 采样点数 Count
        */
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