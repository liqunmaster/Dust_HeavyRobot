#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// 双缓冲结构体：写缓冲与发布/读取分离 支持一致性快照
struct double_buffer_t
{
    uint8_t *storage[2]{};
    size_t size{};
    uint8_t published{};
    uint8_t write_index{1};
    uint32_t generation{};
};

void double_buffer_init(double_buffer_t *buffer, void *first, void *second, size_t size);

void *double_buffer_write_array(double_buffer_t *buffer);

void double_buffer_publish(double_buffer_t *buffer);

const void *double_buffer_read_array(const double_buffer_t *buffer);

bool double_buffer_snapshot(const double_buffer_t *buffer, void *destination);

void double_buffer_reset(double_buffer_t *buffer, const void *value);
