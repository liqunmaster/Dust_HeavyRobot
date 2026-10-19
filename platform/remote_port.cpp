#include "remote_port.hpp"
#include "input_manager.hpp"

volatile uint32_t remote_port_loop_count;
volatile uint32_t remote_port_total_cycles;
volatile uint32_t remote_port_max_cycles;

namespace
{
    RemotePort remote_port;
    K_SEM_DEFINE(remote_sem, 0, 1);
    K_THREAD_STACK_DEFINE(remote_stack, 2048);
}

void RemotePort::RxNotify(void *arg1)
{
    k_sem_give(&remote_sem);
}

void RemotePort::PartialFrameTimeout(struct k_timer *arg1)
{
    k_sem_give(&remote_sem);
}

void RemotePort::PublishSample(const InputSample &sample, void *arg2)
{
    input_manager_publish_sample(sample);
}

int RemotePort::InitUart(RemoteTransport &transport)
{
    const struct device *uart = transport.uart;
    transport.port = bsp_uart_get(uart);
    if (transport.port == nullptr) {
        return -ENODEV;
    }
    if (!device_is_ready(uart)) {
        return -ENODEV;
    }
    struct uart_config config{};
    int result = uart_config_get(uart, &config);
    if (result != 0) {
        return result;
    }
    config.baudrate = uart == transports_[0].uart ? 921600 : 100000;
    config.parity = uart == transports_[0].uart ? UART_CFG_PARITY_NONE : UART_CFG_PARITY_EVEN;
    config.data_bits = UART_CFG_DATA_BITS_8;
    config.stop_bits = UART_CFG_STOP_BITS_1;
    config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    result = uart_configure(uart, &config);
    if (result != 0) {
        return result;
    }
    result = transport.port->Init();
    return result == 0 ? transport.port->SetRxNotify(RxNotify, nullptr) : result;
}

void RemotePort::ReceiveUart(RemoteTransport &transport)
{
    for (size_t batch = 0; batch < 4; ++batch) {
        RemoteRxChunk chunk{};
        const int count = transport.port->Receive(chunk.bytes, sizeof(chunk.bytes));
        if (count == 0) {
            break;
        }
        chunk.source = transport.source;
        chunk.timestamp_ms = k_uptime_get_32();
        chunk.discontinuity = count < 0;
        chunk.length = count > 0 ? static_cast<uint16_t>(count) : 0;
        input_process_chunk(chunk, PublishSample, nullptr);
        if (count < 0) {
            break;
        }
        k_timer_start(&partial_frame_timer_, K_MSEC(20), K_NO_WAIT);
        atomic_inc(&feedback_count_);
        if (batch == 3) {
            k_sem_give(&remote_sem);
        }
    }
}

void RemotePort::ThreadEntry(void *context, void *, void *)
{
    static_cast<RemotePort *>(context)->Task();
}

void RemotePort::Task()
{
    while (1) {
        k_sem_take(&remote_sem, K_FOREVER);
        const uint32_t start = k_cycle_get_32();
        for (auto &transport : transports_) {
            ReceiveUart(transport);
        }
        input_expire_partial_frames(k_uptime_get_32());
        const uint32_t elapsed = k_cycle_get_32() - start;
        ++remote_port_loop_count;
        remote_port_total_cycles += elapsed;
        if (elapsed > remote_port_max_cycles) {
            remote_port_max_cycles = elapsed;
        }
    }
}

void RemotePort::Init()
{
    if (started_) {
        return;
    }
    k_timer_init(&partial_frame_timer_, PartialFrameTimeout, nullptr);
    for (auto &transport : transports_) {
        const int result = InitUart(transport);
        if (result != 0) {
            return;
        }
    }
    k_thread_create(&remote_thread_, remote_stack, K_THREAD_STACK_SIZEOF(remote_stack), ThreadEntry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(&remote_thread_, "remote_rx");
    started_ = true;
    k_sem_give(&remote_sem);
}

uint32_t RemotePort::FeedbackCount()
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

void remote_port_init()
{
    remote_port.Init();
}

uint32_t remote_port_feedback_count()
{
    return remote_port.FeedbackCount();
}
