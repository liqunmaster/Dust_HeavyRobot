#pragma once

#include <stddef.h>
#include <stdint.h>

struct RingBuffer
{
    uint8_t *storage{};
    size_t capacity{};
    size_t head{};
    size_t tail{};
};

void ring_buffer_init(RingBuffer *buffer, void *storage, size_t capacity);

size_t ring_buffer_write(RingBuffer *buffer, const void *data, size_t length);

size_t ring_buffer_read(RingBuffer *buffer, void *data, size_t length);

size_t ring_buffer_size(const RingBuffer *buffer);

size_t ring_buffer_free(const RingBuffer *buffer);

bool ring_buffer_empty(const RingBuffer *buffer);

bool ring_buffer_full(const RingBuffer *buffer);

void ring_buffer_clear(RingBuffer *buffer);
