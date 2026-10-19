#pragma once

#include <errno.h>
#include <zephyr/kernel.h>
#include "input_parser.hpp"

struct RemoteRxChunk;

class Input
{
    public:
    void ProcessChunk(const RemoteRxChunk &chunk,
                      InputStreamParser::SampleCallback on_sample, void *context);

    void ExpirePartialFrames(uint32_t now_ms);

    int GetSample(RemoteProtocol protocol, InputSample &sample);

    uint32_t FeedbackCount();

    private:
    struct StreamState
    {
        InputStreamParser parser;
        uint32_t last_rx_ms;
        bool has_rx;
    };

    struct SampleSink
    {
        InputStreamParser::SampleCallback callback;
        void *context;
    };

    int SampleIndex(RemoteProtocol protocol);

    static void StoreSample(const InputSample &sample, void *context);

    void OnStoreSample(const InputSample &sample, void *context);
    struct k_spinlock sample_lock_{};
    atomic_t feedback_count_{};
    StreamState streams_[2] = {
        {InputStreamParser(RemoteUartSource::uart1), 0, false},
        {InputStreamParser(RemoteUartSource::uart4), 0, false},
    };
    InputSample samples_[3]{};
    bool has_sample_[3]{};
};

void input_process_chunk(const RemoteRxChunk &chunk, InputStreamParser::SampleCallback on_sample, void *context);

void input_expire_partial_frames(uint32_t now_ms);

int input_get_sample(RemoteProtocol protocol, InputSample &sample);

uint32_t input_feedback_count();
