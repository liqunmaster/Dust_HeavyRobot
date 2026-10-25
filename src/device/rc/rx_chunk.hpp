#pragma once
#include "remote_types.hpp"
struct RemoteRxChunk {
	RemoteUartSource source;
	uint16_t length;
	uint32_t timestamp_ms;
	bool discontinuity;
	uint8_t bytes[128];
};
