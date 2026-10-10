#pragma once

#include <string.h>

#include "remote_dt7.hpp"
#include "remote_vt02.hpp"
#include "remote_vt03.hpp"
#include "remote_types.hpp"

// 一次解码得到的遥控采样数据
struct input_sample
{
    remote_protocol protocol;
    remote_uart_source source;
    uint32_t timestamp_ms;

    union
    {
        dt7_sample dt7_data;
        vt02_sample vt02_data;
        vt03_sample vt03_data;
    };
};

// 遥控数据流解析器，负责从字节流中识别并解码各协议帧
class input_stream_parser
{
    public:
    using sample_callback = void (*)(const input_sample &, void *);

    /**
     * @brief 构造解析器并指定数据来源
     *
     * @param source 数据来源串口
    */
    explicit input_stream_parser(remote_uart_source source) : source_(source) {}

    void reset();

    void feed(const uint8_t *data, size_t length, uint32_t timestamp_ms, sample_callback callback, void *context);

    private:
    // 解析过程的帧状态枚举
    enum class parse_state : uint8_t
    {
        header,
        vt03_header,
        vt02_header,
        frame
    };

    void consume(size_t count);

    void try_decode(uint32_t timestamp_ms, sample_callback callback, void *context);

    remote_uart_source source_;

    parse_state state_ = parse_state::header;

    remote_protocol candidate_ = remote_protocol::none;

    size_t expected_length_ = 0;

    size_t length_ = 0;

    uint8_t buffer_[128]{};
};