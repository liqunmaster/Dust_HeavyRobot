#pragma once

#include <string.h>
#include "remote_dt7.hpp"
#include "remote_vt02.hpp"
#include "remote_vt03.hpp"
#include "remote_types.hpp"

struct InputSample
{
    RemoteProtocol protocol;
    RemoteUartSource source;
    uint32_t timestamp_ms;

    union
    {
        Dt7Sample dt7_data;
        Vt02Sample vt02_data;
        Vt03Sample vt03_data;
    };
};

class InputStreamParser
{
    public:
    using SampleCallback = void (*)(const InputSample &, void *);

    explicit InputStreamParser(RemoteUartSource source) : source_(source) {}

    void Reset();

    void Feed(const uint8_t *data, size_t length, uint32_t timestamp_ms, SampleCallback callback, void *context);

    private:

    enum class ParseState : uint8_t
    {
        header,
        vt03_header,
        vt02_header,
        frame
    };

    void Consume(size_t count);

    void TryDecode(uint32_t timestamp_ms, SampleCallback callback, void *context);

    RemoteUartSource source_;

    ParseState state_ = ParseState::header;

    RemoteProtocol candidate_ = RemoteProtocol::none;

    size_t expected_length_ = 0;

    size_t length_ = 0;

    uint8_t buffer_[128]{};
};
