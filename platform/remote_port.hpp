#pragma once

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include "bsp_uart.hpp"
#include "input.hpp"
#include "remote_types.hpp"
#include "remote_channel.hpp"

struct RemoteRxChunk
{
    RemoteUartSource source;
    uint16_t length;
    uint32_t timestamp_ms;
    bool discontinuity;
    uint8_t bytes[128];
};

extern volatile uint32_t remote_port_loop_count;
extern volatile uint32_t remote_port_total_cycles;
extern volatile uint32_t remote_port_max_cycles;

class RemotePort
{
    public:
    void Init();

    uint32_t FeedbackCount();

    private:
    struct RemoteTransport
    {
        const struct device *uart;
        RemoteUartSource source;
        ::Uart *port;
    };

    static void RxNotify(void *arg1);

    static void PartialFrameTimeout(struct k_timer *arg1);

    static void PublishSample(const InputSample &sample, void *arg2);

    int InitUart(RemoteTransport &transport);

    void ReceiveUart(RemoteTransport &transport);

    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    struct k_thread remote_thread_{};
    struct k_timer partial_frame_timer_{};
    atomic_t feedback_count_{};
    bool started_ = false;
    RemoteTransport transports_[2] = {
        {DEVICE_DT_GET(DT_ALIAS(vt03_uart)), RemoteUartSource::uart1, nullptr},
        {DEVICE_DT_GET(DT_ALIAS(dt7_uart)), RemoteUartSource::uart4, nullptr},
    };
};

void remote_port_init();

uint32_t remote_port_feedback_count();
