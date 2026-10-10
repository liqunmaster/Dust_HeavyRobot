#include "ring_buffer.hpp"

/**
 * @brief 初始化环形缓冲区
 *
 * @param buffer 环形缓冲区指针
 * @param storage 数据存储区指针
 * @param capacity 缓冲区容量
 */
void ring_buffer_init(ring_buffer_t *buffer, void *storage, size_t capacity)
{
    if (buffer == nullptr) {
        return;
    }

    buffer->storage = static_cast<uint8_t *>(storage);
    buffer->capacity = capacity;
    __atomic_store_n(&buffer->head, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&buffer->tail, 0, __ATOMIC_RELAXED);
}

/**
 * @brief 获取缓冲区中已写入的字节数
 *
 * @param buffer 环形缓冲区指针
 * @return 已用字节数
 */
size_t ring_buffer_size(const ring_buffer_t *buffer)
{
    if (buffer == nullptr || buffer->capacity == 0) {
        return 0;
    }

    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_ACQUIRE);
    return head - tail;
}

/**
 * @brief 获取缓冲区剩余可用字节数
 *
 * @param buffer 环形缓冲区指针
 * @return 剩余可用字节数
 */
size_t ring_buffer_free(const ring_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return 0;
    }

    const size_t used = ring_buffer_size(buffer);
    return used < buffer->capacity ? buffer->capacity - used : 0;
}

/**
 * @brief 向缓冲区写入数据
 *
 * @param buffer 环形缓冲区指针
 * @param data 待写入数据指针
 * @param length 待写入字节数
 * @return 实际写入字节数
 */
size_t ring_buffer_write(ring_buffer_t *buffer, const void *data, size_t length)
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

/**
 * @brief 从缓冲区读取数据
 *
 * @param buffer 环形缓冲区指针
 * @param data 数据输出指针
 * @param length 请求读取字节数
 * @return 实际读取字节数
 */
size_t ring_buffer_read(ring_buffer_t *buffer, void *data, size_t length)
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

/**
 * @brief 判断缓冲区是否为空
 *
 * @param buffer 环形缓冲区指针
 * @return 空返回 true 否则 false
 */
bool ring_buffer_empty(const ring_buffer_t *buffer)
{
    return ring_buffer_size(buffer) == 0;
}

/**
 * @brief 判断缓冲区是否已满
 *
 * @param buffer 环形缓冲区指针
 * @return 已满返回 true 否则 false
 */
bool ring_buffer_full(const ring_buffer_t *buffer)
{
    return buffer != nullptr && buffer->capacity != 0 && ring_buffer_size(buffer) >= buffer->capacity;
}

/**
 * @brief 清空缓冲区
 *
 * @param buffer 环形缓冲区指针
 */
void ring_buffer_clear(ring_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return;
    }

    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    __atomic_store_n(&buffer->tail, head, __ATOMIC_RELEASE);
}
