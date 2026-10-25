#pragma once
#include <stddef.h>
#include <stdint.h>

namespace ws2812b_encoding
{
constexpr uint32_t bit_period_ns = 1250;
constexpr uint32_t rising_edge_ns = 200;
constexpr uint32_t zero_high_ns = 350;
constexpr uint32_t one_high_ns = 700;
constexpr uint32_t dma_update_ns = 1100;
constexpr size_t data_bits = 24;
// 确保最后一个有效位结束后，DMA 完成回调才可断开引脚。
// 额外时钟对应灯链中的下一颗灯；本板仅有一颗灯。
constexpr size_t trailing_bits = 2;
constexpr size_t frame_words = data_bits + trailing_bits;

constexpr uint32_t NsToTicks(uint32_t clock_hz, uint32_t ns)
{
	return static_cast<uint32_t>((static_cast<uint64_t>(clock_hz) * ns + 500000000ULL) / 1000000000ULL);
}

// CMP1 寄存器原始值；CMP0 在每位的固定时刻拉高输出，
// CMP1 在 350/700 ns 后拉低输出。采用 GRB 顺序，高位先发。
constexpr void Encode(uint32_t *words, uint32_t clock_hz, uint8_t red, uint8_t green, uint8_t blue)
{
	const uint8_t grb[] = {green, red, blue};
	const uint32_t zero = NsToTicks(clock_hz, rising_edge_ns + zero_high_ns) - 1U;
	const uint32_t one = NsToTicks(clock_hz, rising_edge_ns + one_high_ns) - 1U;
	size_t pos = 0;

	for (uint8_t byte : grb) {
		for (int bit = 7; bit >= 0; --bit) {
			words[pos++] = (byte & (1U << bit)) != 0U ? one : zero;
		}
	}

	for (size_t i = 0; i < trailing_bits; ++i) {
		words[pos++] = zero;
	}
}
}
