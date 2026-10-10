#include "input.hpp"

namespace
{
    constexpr uint32_t partial_frame_timeout_ms = 20;

    constexpr uint32_t sample_timeout_ms = 100;

    struct k_spinlock sample_lock;

    atomic_t feedback_count;

    // 单路串口数据流的解析状态
    struct stream_state
    {
        input_stream_parser parser;
        uint32_t last_rx_ms;
        bool has_rx;
    };

    stream_state streams[] = {
        {input_stream_parser(remote_uart_source::uart1), 0, false},
        {input_stream_parser(remote_uart_source::uart4), 0, false},
    };

    input_sample samples[3]{};

    bool has_sample[3]{};

    // 样本存储回调的上下文（指向可用回调及其参数）
    struct sample_sink
    {
        input_stream_parser::sample_callback callback;
        void *context;
    };

    /**
     * @brief 根据遥控协议获取其在样本数组中的索引
     *
     * @param protocol 遥控协议
     * @return 对应的索引，非法协议返回 -1
    */
    int sample_index(remote_protocol protocol)
    {
        switch (protocol) {
            case remote_protocol::dt7:
                return 0;
            case remote_protocol::vt02:
                return 1;
            case remote_protocol::vt03:
                return 2;
            default:
                return -1;
        }
    }

    /**
     * @brief 存储解析到的采样并通知外部回调
     *
     * @param sample 解析到的遥控采样
     * @param context 上下文指针（指向 sample_sink）
    */
    void store_sample(const input_sample &sample, void *context)
    {
        const int index = sample_index(sample.protocol);
        if (index < 0) {
            return;
        }
        const k_spinlock_key_t key = k_spin_lock(&sample_lock);
        samples[index] = sample;
        has_sample[index] = true;
        k_spin_unlock(&sample_lock, key);
        atomic_inc(&feedback_count);
        const auto &sink = *static_cast<sample_sink *>(context);
        if (sink.callback != nullptr) {
            sink.callback(sample, sink.context);
        }
    }
}

/**
 * @brief 处理一串串口接收到的数据块，解析出遥控器采样
 *
 * @param chunk 待处理的串口数据块
 * @param on_sample 解析到采样后调用的回调函数
 * @param context 传给回调函数的上下文指针
*/
void input_process_chunk(const remote_rx_chunk &chunk,
                         input_stream_parser::sample_callback on_sample, void *context)
{
    const size_t index = chunk.source == remote_uart_source::uart1 ? 0 : 1;
    auto &stream = streams[index];
    if (chunk.discontinuity || (stream.has_rx && chunk.timestamp_ms - stream.last_rx_ms >= partial_frame_timeout_ms)) {
        stream.parser.reset();
    }
    stream.last_rx_ms = chunk.timestamp_ms;
    stream.has_rx = chunk.length != 0;
    sample_sink sink{on_sample, context};
    stream.parser.feed(chunk.bytes, chunk.length, chunk.timestamp_ms, store_sample, &sink);
}

/**
 * @brief 使超时未完成的半帧数据失效并复位解析器
 *
 * @param now_ms 当前运行时间（毫秒）
*/
void input_expire_partial_frames(uint32_t now_ms)
{
    for (auto &stream : streams) {
        if (stream.has_rx && now_ms - stream.last_rx_ms >= partial_frame_timeout_ms) {
            stream.parser.reset();
            stream.has_rx = false;
        }
    }
}

/**
 * @brief 获取指定协议的最新遥控采样
 *
 * @param protocol 目标遥控协议
 * @param sample 用于接收采样的输出参数
 * @return 0 表示获取成功，否则返回对应错误码
*/
int input_get_sample(remote_protocol protocol, input_sample &sample)
{
    const int index = sample_index(protocol);
    if (index < 0) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&sample_lock);
    if (!has_sample[index]) {
        k_spin_unlock(&sample_lock, key);
        return -ENODATA;
    }
    const input_sample snapshot = samples[index];
    k_spin_unlock(&sample_lock, key);
    if (k_uptime_get_32() - snapshot.timestamp_ms >= sample_timeout_ms) {
        return -ETIMEDOUT;
    }
    sample = snapshot;
    return 0;
}

/**
 * @brief 获取累计收到的采样帧数
 *
 * @return 累计收到的采样帧数
*/
uint32_t input_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&feedback_count));
}