#pragma once

#include <stddef.h>
#include <stdint.h>

// 无锁单生产者单消费者环形缓冲区结构体
struct ring_buffer_t
{
    uint8_t *storage{};
    size_t capacity{};
    size_t head{};
    size_t tail{};
};

void ring_buffer_init(ring_buffer_t *buffer, void *storage, size_t capacity);

size_t ring_buffer_write(ring_buffer_t *buffer, const void *data, size_t length);

size_t ring_buffer_read(ring_buffer_t *buffer, void *data, size_t length);

size_t ring_buffer_size(const ring_buffer_t *buffer);

size_t ring_buffer_free(const ring_buffer_t *buffer);

bool ring_buffer_empty(const ring_buffer_t *buffer);

bool ring_buffer_full(const ring_buffer_t *buffer);

void ring_buffer_clear(ring_buffer_t *buffer);
