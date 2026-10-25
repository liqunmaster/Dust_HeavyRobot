#pragma once
#include <stdint.h>
#include <zephyr/kernel.h>
namespace robot
{
inline uint64_t now_us() { return static_cast<uint64_t>(k_ticks_to_us_floor64(k_uptime_ticks())); }
inline uint64_t source_time_us(uint32_t source_ms)
{
	const uint64_t now = (now_us() / 1000) * 1000;
	const uint64_t age = static_cast<uint32_t>(static_cast<uint32_t>(now / 1000) - source_ms) * 1000ULL;
	return age <= now ? now - age : 0;
}
inline bool fresh(uint64_t timestamp, uint64_t max_age_us)
{
	const uint64_t now = now_us();
	return timestamp <= now && now - timestamp < max_age_us;
}
}
