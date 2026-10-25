#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct DoubleBuffer {
	uint8_t *storage[2] {};
	size_t size{};
	uint8_t published{};
	uint8_t write_index{1};
	uint32_t generation{};
};

void double_buffer_init(DoubleBuffer *buffer, void *first, void *second, size_t size);

void *double_buffer_write_array(DoubleBuffer *buffer);

void double_buffer_publish(DoubleBuffer *buffer);

const void *double_buffer_read_array(const DoubleBuffer *buffer);

bool double_buffer_snapshot(const DoubleBuffer *buffer, void *destination);

void double_buffer_reset(DoubleBuffer *buffer, const void *value);
