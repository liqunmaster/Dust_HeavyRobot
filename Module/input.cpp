#include "input.hpp"
#include "remote_port.hpp"

namespace
{
    Input input;
}

int Input::SampleIndex(RemoteProtocol protocol)
{
    switch (protocol) {
        case RemoteProtocol::dt7:
            return 0;
        case RemoteProtocol::vt02:
            return 1;
        case RemoteProtocol::vt03:
            return 2;
        default:
            return -1;
    }
}

void Input::StoreSample(const InputSample &sample, void *context)
{
    input.OnStoreSample(sample, context);
}

void Input::OnStoreSample(const InputSample &sample, void *context)
{
    const int index = SampleIndex(sample.protocol);
    if (index < 0) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&sample_lock_);
    samples_[index] = sample;
    has_sample_[index] = true;
    k_spin_unlock(&sample_lock_, key);
    atomic_inc(&feedback_count_);
    const auto &sink = *static_cast<SampleSink *>(context);
    if (sink.callback != nullptr) {
        sink.callback(sample, sink.context);
    }
}

void Input::ProcessChunk(const RemoteRxChunk &chunk,
                         InputStreamParser::SampleCallback on_sample, void *context)
{
    const size_t index = chunk.source == RemoteUartSource::uart1 ? 0 : 1;
    auto &stream = streams_[index];
    if (chunk.discontinuity || (stream.has_rx && chunk.timestamp_ms - stream.last_rx_ms >= 20)) {
        stream.parser.Reset();
    }
    stream.last_rx_ms = chunk.timestamp_ms;
    stream.has_rx = chunk.length != 0;
    SampleSink sink{on_sample, context};
    stream.parser.Feed(chunk.bytes, chunk.length, chunk.timestamp_ms, StoreSample, &sink);
}

void Input::ExpirePartialFrames(uint32_t now_ms)
{
    for (auto &stream : streams_) {
        if (stream.has_rx && now_ms - stream.last_rx_ms >= 20) {
            stream.parser.Reset();
            stream.has_rx = false;
        }
    }
}

int Input::GetSample(RemoteProtocol protocol, InputSample &sample)
{
    const int index = SampleIndex(protocol);
    if (index < 0) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&sample_lock_);
    if (!has_sample_[index]) {
        k_spin_unlock(&sample_lock_, key);
        return -ENODATA;
    }
    const InputSample snapshot = samples_[index];
    k_spin_unlock(&sample_lock_, key);
    if (k_uptime_get_32() - snapshot.timestamp_ms >= 100) {
        return -ETIMEDOUT;
    }
    sample = snapshot;
    return 0;
}

uint32_t Input::FeedbackCount()
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

void input_process_chunk(const RemoteRxChunk &chunk,
                         InputStreamParser::SampleCallback on_sample, void *context)
{
    input.ProcessChunk(chunk, on_sample, context);
}

void input_expire_partial_frames(uint32_t now_ms)
{
    input.ExpirePartialFrames(now_ms);
}

int input_get_sample(RemoteProtocol protocol, InputSample &sample)
{
    return input.GetSample(protocol, sample);
}

uint32_t input_feedback_count()
{
    return input.FeedbackCount();
}
