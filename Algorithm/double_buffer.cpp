#include "double_buffer.hpp"

/**
 * @brief 初始化双缓冲区
 *
 * @param buffer 双缓冲结构体指针
 * @param first 第一个存储区指针
 * @param second 第二个存储区指针
 * @param size 单块存储区大小
 */
void double_buffer_init(double_buffer_t *buffer, void *first, void *second, size_t size)
{
    if (buffer == nullptr) {
        return;
    }

    buffer->storage[0] = static_cast<uint8_t *>(first);
    buffer->storage[1] = static_cast<uint8_t *>(second);
    buffer->size = size;
    buffer->write_index = 1;
    __atomic_store_n(&buffer->published, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&buffer->generation, 0, __ATOMIC_RELAXED);
}

/**
 * @brief 获取当前可写的缓冲数组指针
 *
 * @param buffer 双缓冲结构体指针
 * @return 可写缓冲区指针
 */
void *double_buffer_write_array(double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return nullptr;
    }
    return buffer->storage[buffer->write_index];
}

/**
 * @brief 发布当前写缓冲并切换写缓冲块
 *
 * @param buffer 双缓冲结构体指针
 */
void double_buffer_publish(double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return;
    }

    __atomic_store_n(&buffer->published, buffer->write_index, __ATOMIC_RELAXED);
    __atomic_add_fetch(&buffer->generation, 1, __ATOMIC_RELEASE);
    buffer->write_index ^= 1;
}

/**
 * @brief 获取当前已发布的缓冲数组指针
 *
 * @param buffer 双缓冲结构体指针
 * @return 已发布缓冲区指针
 */
const void *double_buffer_read_array(const double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return nullptr;
    }

    (void)__atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
    const uint8_t index = __atomic_load_n(&buffer->published, __ATOMIC_RELAXED);
    return buffer->storage[index];
}

/**
 * @brief 将当前已发布缓冲一致性快照拷贝到目标地址
 *
 * @param buffer 双缓冲结构体指针
 * @param destination 目标地址
 * @return 快照是否成功
 */
bool double_buffer_snapshot(const double_buffer_t *buffer, void *destination)
{
    if (buffer == nullptr || destination == nullptr || buffer->size == 0) {
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

/**
 * @brief 将指定值填充两块缓冲并复位发布状态
 *
 * @param buffer 双缓冲结构体指针
 * @param value 用于填充的值
 */
void double_buffer_reset(double_buffer_t *buffer, const void *value)
{
    if (buffer == nullptr || value == nullptr || buffer->size == 0) {
        return;
    }

    memcpy(buffer->storage[0], value, buffer->size);
    memcpy(buffer->storage[1], value, buffer->size);
    buffer->write_index = 1;
    __atomic_store_n(&buffer->published, 0, __ATOMIC_RELAXED);
    __atomic_add_fetch(&buffer->generation, 1, __ATOMIC_RELEASE);
}
