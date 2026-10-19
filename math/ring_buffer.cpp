#include "ring_buffer.hpp"

void ring_buffer_init(RingBuffer *buffer, void *storage, size_t capacity)
{
    if (buffer == nullptr) {
        return;
    }

    buffer->storage = static_cast<uint8_t *>(storage);
    buffer->capacity = capacity;
    __atomic_store_n(&buffer->head, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&buffer->tail, 0, __ATOMIC_RELAXED);
}

size_t ring_buffer_size(const RingBuffer *buffer)
{
    if (buffer == nullptr || buffer->capacity == 0) {
        return 0;
    }

    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_ACQUIRE);
    return head - tail;
}

size_t ring_buffer_free(const RingBuffer *buffer)
{
    if (buffer == nullptr) {
        return 0;
    }

    const size_t used = ring_buffer_size(buffer);
    return used < buffer->capacity ? buffer->capacity - used : 0;
}

size_t ring_buffer_write(RingBuffer *buffer, const void *data, size_t length)
{
    if (buffer == nullptr || buffer->storage == nullptr || data == nullptr || buffer->capacity == 0 || length == 0) {
        return 0;
    }

    const uint8_t *source = static_cast<const uint8_t *>(data);
    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_RELAXED);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_ACQUIRE);
    const size_t free = buffer->capacity - (head - tail);
    const size_t count = length < free ? length : free;

    for (size_t i = 0; i < count; ++i) {
        buffer->storage[(head + i) % buffer->capacity] = source[i];
    }
    __atomic_store_n(&buffer->head, head + count, __ATOMIC_RELEASE);
    return count;
}

size_t ring_buffer_read(RingBuffer *buffer, void *data, size_t length)
{
    if (buffer == nullptr || buffer->storage == nullptr || data == nullptr || buffer->capacity == 0 || length == 0) {
        return 0;
    }

    uint8_t *destination = static_cast<uint8_t *>(data);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_RELAXED);
    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    const size_t used = head - tail;
    const size_t count = length < used ? length : used;

    for (size_t i = 0; i < count; ++i) {
        destination[i] = buffer->storage[(tail + i) % buffer->capacity];
    }
    __atomic_store_n(&buffer->tail, tail + count, __ATOMIC_RELEASE);
    return count;
}

bool ring_buffer_empty(const RingBuffer *buffer)
{
    return ring_buffer_size(buffer) == 0;
}

bool ring_buffer_full(const RingBuffer *buffer)
{
    return buffer != nullptr && buffer->capacity != 0 && ring_buffer_size(buffer) >= buffer->capacity;
}

void ring_buffer_clear(RingBuffer *buffer)
{
    if (buffer == nullptr) {
        return;
    }

    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    __atomic_store_n(&buffer->tail, head, __ATOMIC_RELEASE);
}
