#pragma once

#include <cstdint>

namespace alg
{

class LeastSquares final
{
public:
	static constexpr uint16_t kMaxOrder = 16;

	uint16_t Order = 0;

	uint32_t Count = 0;

	float x[kMaxOrder] {};

	float y[kMaxOrder] {};

	float k = 0.0f;

	float b = 0.0f;

	float StandardDeviation = 0.0f;

	void Init(uint16_t order);

	void Reset() { Init(Order); }

	void Add(float deltax, float y_sample);

	void Update(float deltax, float y_sample) { Add(deltax, y_sample); }

	bool Fit();
	float Derivative(float deltax, float y_sample);

	float Smooth(float deltax, float y_sample);

	float LastDerivative() const noexcept { return k; }

	float LastSmooth() const noexcept;

	uint16_t GetOrder() const noexcept { return Order; }

	uint32_t GetCount() const noexcept { return Count; }

private:
	void ShiftAndAppend(float deltax, float y_sample);
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
