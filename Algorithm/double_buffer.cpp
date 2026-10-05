#include "double_buffer.hpp"

void double_buffer_init(double_buffer_t *buffer, void *first, void *second, size_t size)
{
    if (buffer == nullptr) {
        return;
    }

    buffer->storage[0] = static_cast<uint8_t *>(first);
    buffer->storage[1] = static_cast<uint8_t *>(second);
    buffer->size = size;
    buffer->write_index = 1U;
    __atomic_store_n(&buffer->published, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&buffer->generation, 0U, __ATOMIC_RELAXED);
}

void *double_buffer_write_array(double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return nullptr;
    }
    return buffer->storage[buffer->write_index];
}

void double_buffer_publish(double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return;
    }

    __atomic_store_n(&buffer->published, buffer->write_index, __ATOMIC_RELAXED);
    __atomic_add_fetch(&buffer->generation, 1U, __ATOMIC_RELEASE);
    buffer->write_index ^= 1U;
}

const void *double_buffer_read_array(const double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return nullptr;
    }

    (void)__atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
    const uint8_t index = __atomic_load_n(&buffer->published, __ATOMIC_RELAXED);
    return buffer->storage[index];
}

bool double_buffer_snapshot(const double_buffer_t *buffer, void *destination)
{
    if (buffer == nullptr || destination == nullptr || buffer->size == 0U) {
        return false;
    }

    for (;;) {
        const uint32_t before =
            __atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
        const uint8_t index =
            __atomic_load_n(&buffer->published, __ATOMIC_RELAXED);
        memcpy(destination, buffer->storage[index], buffer->size);
        const uint32_t after =
            __atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
        if (before == after) {
            return true;
        }
    }
}

void double_buffer_reset(double_buffer_t *buffer, const void *value)
{
    if (buffer == nullptr || value == nullptr || buffer->size == 0U) {
        return;
    }

    memcpy(buffer->storage[0], value, buffer->size);
    memcpy(buffer->storage[1], value, buffer->size);
    buffer->write_index = 1U;
    __atomic_store_n(&buffer->published, 0U, __ATOMIC_RELAXED);
    __atomic_add_fetch(&buffer->generation, 1U, __ATOMIC_RELEASE);
}
