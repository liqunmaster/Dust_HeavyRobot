#include "input.hpp"

namespace
{
    constexpr uint32_t partial_frame_timeout_ms = 20U;

    constexpr uint32_t sample_timeout_ms = 100U;

    struct k_spinlock sample_lock;

    atomic_t feedback_count;

    struct stream_state
    {
        input_stream_parser parser;
        uint32_t last_rx_ms;
        bool has_rx;
    };

    stream_state streams[] = {
        {input_stream_parser(remote_uart_source::uart1), 0U, false},
        {input_stream_parser(remote_uart_source::uart4), 0U, false},
    };

    input_sample samples[3]{};

    bool has_sample[3]{};

    struct sample_sink
    {
        input_stream_parser::sample_callback callback;
        void *context;
    };

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

void input_process_chunk(const remote_rx_chunk &chunk,
                         input_stream_parser::sample_callback on_sample, void *context)
{
    const size_t index = chunk.source == remote_uart_source::uart1 ? 0U : 1U;
    auto &stream = streams[index];
    if (chunk.discontinuity || (stream.has_rx && chunk.timestamp_ms - stream.last_rx_ms >= partial_frame_timeout_ms)) {
        stream.parser.reset();
    }
    stream.last_rx_ms = chunk.timestamp_ms;
    stream.has_rx = chunk.length != 0U;
    sample_sink sink{on_sample, context};
    stream.parser.feed(chunk.bytes, chunk.length, chunk.timestamp_ms, store_sample, &sink);
}

void input_expire_partial_frames(uint32_t now_ms)
{
    for (auto &stream : streams) {
        if (stream.has_rx && now_ms - stream.last_rx_ms >= partial_frame_timeout_ms) {
            stream.parser.reset();
            stream.has_rx = false;
        }
    }
}

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

uint32_t input_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&feedback_count));
}
