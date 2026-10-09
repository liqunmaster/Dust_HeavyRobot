#include "remote_port.hpp"

#include "remote_channel.hpp"

volatile uint32_t remote_port_loop_count;
volatile uint32_t remote_port_total_cycles;
volatile uint32_t remote_port_max_cycles;

namespace
{
    K_SEM_DEFINE(remote_sem, 0, 1);
    K_THREAD_STACK_DEFINE(remote_stack, 2048);
    struct k_thread remote_thread;
    struct k_timer remote_timer;
    atomic_t feedback_count;
    bool started;

    struct remote_transport
    {
        const struct device *uart;
        remote_uart_source source;
        Uart *port;
    };

    remote_transport transports[] = {
        {DEVICE_DT_GET(DT_ALIAS(vt03_uart)), remote_uart_source::uart1, nullptr},
        {DEVICE_DT_GET(DT_ALIAS(dt7_uart)), remote_uart_source::uart4, nullptr},
    };

    void publish_sample(const input_sample &sample, void *)
    {
        remote_channel_publish_sample(sample);
    }

    int init_uart(remote_transport &transport)
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
        config.baudrate = uart == transports[0].uart ? 921600U : 100000U;
        config.parity = uart == transports[0].uart ? UART_CFG_PARITY_NONE : UART_CFG_PARITY_EVEN;
        config.data_bits = UART_CFG_DATA_BITS_8;
        config.stop_bits = UART_CFG_STOP_BITS_1;
        config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
        result = uart_configure(uart, &config);
        return result == 0 ? transport.port->init() : result;
    }

    void receive_uart(remote_transport &transport)
    {
        for (size_t batch = 0U; batch < 4U; ++batch) {
            remote_rx_chunk chunk{};
            const int count = transport.port->receive(chunk.bytes, sizeof(chunk.bytes));
            if (count == 0) {
                break;
            }
            chunk.source = transport.source;
            chunk.timestamp_ms = k_uptime_get_32();
            chunk.discontinuity = count < 0;
            chunk.length = count > 0 ? static_cast<uint16_t>(count) : 0U;
            input_process_chunk(chunk, publish_sample, nullptr);
            if (count < 0) {
                break;
            }
            atomic_inc(&feedback_count);
        }
    }

    void timer_callback(struct k_timer *)
    {
        k_sem_give(&remote_sem);
    }

    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&remote_sem, K_FOREVER);
            const uint32_t start = k_cycle_get_32();
            for (auto &transport : transports) {
                receive_uart(transport);
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
}

void remote_port_init()
{
    if (started) {
        return;
    }
    for (auto &transport : transports) {
        const int result = init_uart(transport);
        if (result != 0) {
            return;
        }
    }
    k_timer_init(&remote_timer, timer_callback, nullptr);
    k_thread_create(&remote_thread, remote_stack, K_THREAD_STACK_SIZEOF(remote_stack), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(&remote_thread, "remote_rx");
    started = true;
    k_timer_start(&remote_timer, K_MSEC(5), K_MSEC(5));
}

uint32_t remote_port_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&feedback_count));
}
