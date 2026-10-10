#include "least_squares.hpp"

#include <cmath>
#include <cstring>

namespace alg
{

    /**
     * @brief 初始化最小二乘回归 清空采样数据
     *
     * @param order 回归阶数
     */
    void least_squares::init(uint16_t order)
    {
        Order = order > kMaxOrder ? kMaxOrder : order;
        Count = 0;
        k = 0.0f;
        b = 0.0f;
        StandardDeviation = 0.0f;
        std::memset(x, 0, sizeof(x));
        std::memset(y, 0, sizeof(y));
    }

    /**
     * @brief 将新采样点以等间距移窗方式加入数据窗口
     *
     * @param deltax 相邻采样点的横坐标间隔
     * @param y_sample 新采样点的纵坐标值
     */
    void least_squares::shift_and_append(float deltax, float y_sample)
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

    /**
     * @brief 使用当前数据窗拟合直线 y = kx + b 并计算平均绝对标准差
     *
     * @return 拟合是否成功
     */
    bool least_squares::fit()
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

    /**
     * @brief 加入一个采样点并重新拟合
     *
     * @param deltax 相邻采样点的横坐标间隔
     * @param y_sample 新采样点的纵坐标值
     */
    void least_squares::add(float deltax, float y_sample)
    {
        shift_and_append(deltax, y_sample);
        fit();
    }

    /**
     * @brief 加入采样点并返回当前拟合直线的斜率
     *
     * @param deltax 相邻采样点的横坐标间隔
     * @param y_sample 新采样点的纵坐标值
     * @return 拟合斜率 k
     */
    float least_squares::derivative(float deltax, float y_sample)
    {
        add(deltax, y_sample);
        return k;
    }

    /**
     * @brief 加入采样点并返回最新拟合点
     *
     * @param deltax 相邻采样点的横坐标间隔
     * @param y_sample 新采样点的纵坐标值
     * @return 最新拟合平滑值
     */
    float least_squares::smooth(float deltax, float y_sample)
    {
        add(deltax, y_sample);
        return last_smooth();
    }

    /**
     * @brief 获取最新拟合点的平滑值
     *
     * @return 最新拟合平滑值
     */
    float least_squares::last_smooth() const noexcept
    {
        if (Count == 0) {
            return 0.0f;
        }
        const uint16_t index = static_cast<uint16_t>((Count < Order ? Count : Order) - 1);
        return k * x[index] + b;
    }

    /**
     * @brief C 风格封装：初始化最小二乘回归
     *
     * @param least_squares 目标对象指针
     * @param order 回归阶数
     */
    void OLS_Init(least_squares *least_squares, uint16_t order)
    {
        if (least_squares != nullptr) {
            least_squares->init(order);
        }
    }

    /**
     * @brief C 风格封装：加入采样点并重新拟合
     *
     * @param least_squares 目标对象指针
     * @param deltax 相邻采样点的横坐标间隔
     * @param y 新采样点的纵坐标值
     */
    void OLS_Update(least_squares *least_squares, float deltax, float y)
    {
        if (least_squares != nullptr) {
            least_squares->update(deltax, y);
        }
    }

    /**
     * @brief C 风格封装：加入采样点并返回拟合斜率
     *
     * @param least_squares 目标对象指针
     * @param deltax 相邻采样点的横坐标间隔
     * @param y 新采样点的纵坐标值
     * @return 拟合斜率 k
     */
    float OLS_Derivative(least_squares *least_squares, float deltax, float y)
    {
        return least_squares != nullptr ? least_squares->derivative(deltax, y) : 0.0f;
    }

    /**
     * @brief C 风格封装：加入采样点并返回最新拟合平滑值
     *
     * @param least_squares 目标对象指针
     * @param deltax 相邻采样点的横坐标间隔
     * @param y 新采样点的纵坐标值
     * @return 最新拟合平滑值
     */
    float OLS_Smooth(least_squares *least_squares, float deltax, float y)
    {
        return least_squares != nullptr ? least_squares->smooth(deltax, y) : 0.0f;
    }

    /**
     * @brief C 风格封装：获取最近一次拟合斜率
     *
     * @param least_squares 目标对象指针
     * @return 拟合斜率 k
     */
    float Get_OLS_Derivative(const least_squares *least_squares)
    {
        return least_squares != nullptr ? least_squares->last_derivative() : 0.0f;
    }

    /**
     * @brief C 风格封装：获取最近一次拟合平滑值
     *
     * @param least_squares 目标对象指针
     * @return 最新拟合平滑值
     */
    float Get_OLS_Smooth(const least_squares *least_squares)
    {
        return least_squares != nullptr ? least_squares->last_smooth() : 0.0f;
    }

}
