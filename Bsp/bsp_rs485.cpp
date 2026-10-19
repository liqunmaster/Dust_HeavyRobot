#include "bsp_rs485.hpp"
#include <errno.h>
#include <hpm_gpio_drv.h>
#include <hpm_iomux.h>
#include <hpm_soc.h>
#include <hpm_uart_drv.h>

Rs485 rs485_uart5{uart5};

int Rs485::Init()
{
    if (IsReady()) {
        return 0;
    }
    if (!atomic_cas(&init_state_, 0, 1)) {
        return -EBUSY;
    }
    int ret = port_ != nullptr ? port_->Init() : -ENODEV;
    if (ret == 0) {
        k_work_init(&work_, Work);
        k_timer_init(&timer_, Timeout, nullptr);
        ret = port_->SetRxNotify(RxNotify, this);
    }
    if (ret == 0) {
        ret = port_->SetTxNotify(TxNotify, this);
    }
    if (ret == 0) {
        ret = uart_irq_callback_user_data_set(port_->GetDevice(), LineIrq, this);
    }
    if (ret != 0) {
        atomic_clear(&init_state_);
        return ret;
    }
    gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_DI_GPIOA, 21);
    HPM_IOC->PAD[IOC_PAD_PA21].FUNC_CTL = IOC_PA21_FUNC_CTL_GPIO_A_21;
    uart_disable_irq(HPM_UART5, uart_intr_tx_line_idle);
    uart_disable_txline_idle_detection(HPM_UART5);
    uart_clear_txline_idle_flag(HPM_UART5);
    atomic_set(&init_state_, 2);
    return 0;
}
bool Rs485::IsReady() const
{
    return atomic_get(&init_state_) == 2;
}

int Rs485::Start(const void *request, size_t request_length, size_t response_length,
                 uint32_t timeout_ms, Rs485Complete complete, void *context)
{
    if (!IsReady()) {
        return -ENODEV;
    }
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (request == nullptr || request_length == 0 || request_length > Uart::tx_capacity ||
        response_length == 0 || response_length > response_capacity ||
        timeout_ms == 0 || complete == nullptr) {
        return -EINVAL;
    }
    const unsigned int key = irq_lock();
    if (phase_ != Phase::idle) {
        irq_unlock(key);
        return -EBUSY;
    }
    (void)port_->ClearRx();
    complete_ = complete;
    context_ = context;
    response_length_ = response_length;
    response_timeout_ms_ = timeout_ms;
    received_ = 0;
    last_response_length_ = 0;
    result_ = 0;
    phase_ = Phase::sending;
    ++transaction_count_;
    gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 1);
    k_timer_start(&timer_, K_MSEC(10), K_NO_WAIT);
    const int ret = port_->TransmitAsync(request, request_length);
    if (ret != 0) {
        k_timer_stop(&timer_);
        gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);
        phase_ = Phase::idle;
        complete_ = nullptr;
    }
    irq_unlock(key);
    return ret;
}

void Rs485::RxNotify(void *context)
{
    auto *self = static_cast<Rs485 *>(context);
    if (self->phase_ == Phase::receiving) {
        k_work_submit(&self->work_);
    }
}

void Rs485::TxNotify(void *context, int result)
{
    auto *self = static_cast<Rs485 *>(context);
    if (self->phase_ != Phase::sending) {
        return;
    }
    if (result != 0) {
        self->Finish(result);
        return;
    }
    if (uart_check_status(HPM_UART5, uart_stat_transmitter_empty)) {
        self->BeginReceive();
        return;
    }
    uart_clear_txline_idle_flag(HPM_UART5);
    uart_rxline_idle_config_t config{};
    config.detect_enable = true;
    config.detect_irq_enable = true;
    config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
    config.threshold = 1;
    (void)uart_init_txline_idle_detection(HPM_UART5, config);
}

void Rs485::LineIrq(const struct device *, void *context)
{
    auto *self = static_cast<Rs485 *>(context);
    if (uart_is_txline_idle(HPM_UART5)) {
        uart_clear_txline_idle_flag(HPM_UART5);
        if (self->phase_ == Phase::sending &&
            (HPM_UART5->IER & uart_intr_tx_line_idle) != 0U) {
            self->BeginReceive();
        }
    }
}
void Rs485::BeginReceive()
{
    uart_disable_irq(HPM_UART5, uart_intr_tx_line_idle);
    uart_disable_txline_idle_detection(HPM_UART5);
    uart_clear_txline_idle_flag(HPM_UART5);
    gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);
    phase_ = Phase::receiving;
    k_timer_start(&timer_, K_MSEC(response_timeout_ms_), K_NO_WAIT);
    k_work_submit(&work_);
}
void Rs485::Timeout(struct k_timer *timer)
{
    CONTAINER_OF(timer, Rs485, timer_)->Finish(-ETIMEDOUT);
}
void Rs485::Finish(int result)
{
    const unsigned int key = irq_lock();
    if (phase_ == Phase::idle || phase_ == Phase::completing) {
        irq_unlock(key);
        return;
    }
    const bool was_sending = phase_ == Phase::sending;
    phase_ = Phase::completing;
    result_ = result;
    if (result == -ETIMEDOUT) {
        ++timeout_count_;
    }
    k_timer_stop(&timer_);
    uart_disable_irq(HPM_UART5, uart_intr_tx_line_idle);
    uart_disable_txline_idle_detection(HPM_UART5);
    gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);
    if (was_sending && result != 0) {
        (void)port_->AbortTransmit();
    }
    k_work_submit(&work_);
    irq_unlock(key);
}
void Rs485::Work(struct k_work *work)
{
    CONTAINER_OF(work, Rs485, work_)->Process();
}

void Rs485::Process()
{
    const unsigned int key = irq_lock();
    if (phase_ == Phase::receiving) {
        const int count = port_->Receive(response_ + received_, response_length_ - received_);
        if (count > 0) {
            received_ += static_cast<size_t>(count);
            received_bytes_ += static_cast<uint32_t>(count);
            last_response_length_ = received_;
            if (received_ == response_length_) {
                Finish(0);
            }
        } else if (count < 0) {
            Finish(count);
        }
    }
    if (phase_ != Phase::completing) {
        irq_unlock(key);
        return;
    }
    const auto complete = complete_;
    void *const context = context_;
    const int result = result_;
    const size_t length = received_;
    complete_ = nullptr;
    phase_ = Phase::idle;
    irq_unlock(key);
    if (complete != nullptr) {
        complete(context, result, response_, length);
    }
}
int bsp_rs485_init(const struct device *device)
{
    return device != nullptr && device != uart5.GetDevice() ? -EINVAL : rs485_uart5.Init();
}
int bsp_rs485_start(const void *request, size_t request_length, size_t response_length,
                    uint32_t timeout_ms, Rs485Complete complete, void *context)
{
    return rs485_uart5.Start(request, request_length, response_length, timeout_ms, complete, context);
}
bool bsp_rs485_is_ready()
{
    return rs485_uart5.IsReady();
}
