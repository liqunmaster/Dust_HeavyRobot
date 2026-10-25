#include "dvc_rs485.hpp"
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

	if (!atomic_cas(&_init_state, 0, 1)) {
		return -EBUSY;
	}

	int ret = _port != nullptr ? _port->Init() : -ENODEV;

	if (ret == 0) {
		k_work_init(&_work, Work);
		k_timer_init(&_timer, Timeout, nullptr);
		ret = _port->SetRxNotify(RxNotify, this);
	}

	if (ret == 0) {
		ret = _port->SetTxNotify(TxNotify, this);
	}

	if (ret == 0) {
		ret = uart_irq_callback_user_data_set(_port->GetDevice(), LineIrq, this);
	}

	if (ret != 0) {
		atomic_clear(&_init_state);
		return ret;
	}

	gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);
	gpio_set_pin_output(HPM_GPIO0, GPIO_DI_GPIOA, 21);
	HPM_IOC->PAD[IOC_PAD_PA21].FUNC_CTL = IOC_PA21_FUNC_CTL_GPIO_A_21;
	uart_disable_irq(HPM_UART5, uart_intr_tx_line_idle);
	uart_disable_txline_idle_detection(HPM_UART5);
	uart_clear_txline_idle_flag(HPM_UART5);
	atomic_set(&_init_state, 2);
	return 0;
}
bool Rs485::IsReady() const
{
	return atomic_get(&_init_state) == 2;
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

	if (_phase != Phase::idle) {
		irq_unlock(key);
		return -EBUSY;
	}

	(void)_port->ClearRx();
	_complete = complete;
	_context = context;
	_response_length = response_length;
	_response_timeout_ms = timeout_ms;
	_received = 0;
	_last_response_length = 0;
	_result = 0;
	_phase = Phase::sending;
	++_transaction_count;
	gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 1);
	k_timer_start(&_timer, K_MSEC(10), K_NO_WAIT);
	const int ret = _port->TransmitAsync(request, request_length);

	if (ret != 0) {
		k_timer_stop(&_timer);
		gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);
		_phase = Phase::idle;
		_complete = nullptr;
	}

	irq_unlock(key);
	return ret;
}

void Rs485::RxNotify(void *context)
{
	auto *self = static_cast<Rs485 *>(context);

	if (self->_phase == Phase::receiving) {
		k_work_submit(&self->_work);
	}
}

void Rs485::TxNotify(void *context, int result)
{
	auto *self = static_cast<Rs485 *>(context);

	if (self->_phase != Phase::sending) {
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

		if (self->_phase == Phase::sending &&
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
	_phase = Phase::receiving;
	k_timer_start(&_timer, K_MSEC(_response_timeout_ms), K_NO_WAIT);
	k_work_submit(&_work);
}
void Rs485::Timeout(struct k_timer *timer)
{
	CONTAINER_OF(timer, Rs485, _timer)->Finish(-ETIMEDOUT);
}
void Rs485::Finish(int result)
{
	const unsigned int key = irq_lock();

	if (_phase == Phase::idle || _phase == Phase::completing) {
		irq_unlock(key);
		return;
	}

	const bool was_sending = _phase == Phase::sending;
	_phase = Phase::completing;
	_result = result;

	if (result == -ETIMEDOUT) {
		++_timeout_count;
	}

	k_timer_stop(&_timer);
	uart_disable_irq(HPM_UART5, uart_intr_tx_line_idle);
	uart_disable_txline_idle_detection(HPM_UART5);
	gpio_write_pin(HPM_GPIO0, GPIO_DI_GPIOA, 21, 0);

	if (was_sending && result != 0) {
		(void)_port->AbortTransmit();
	}

	k_work_submit(&_work);
	irq_unlock(key);
}
void Rs485::Work(struct k_work *work)
{
	CONTAINER_OF(work, Rs485, _work)->Process();
}

void Rs485::Process()
{
	const unsigned int key = irq_lock();

	if (_phase == Phase::receiving) {
		const int count = _port->Receive(_response + _received, _response_length - _received);

		if (count > 0) {
			_received += static_cast<size_t>(count);
			_received_bytes += static_cast<uint32_t>(count);
			_last_response_length = _received;

			if (_received == _response_length) {
				Finish(0);
			}

		} else if (count < 0) {
			Finish(count);
		}
	}

	if (_phase != Phase::completing) {
		irq_unlock(key);
		return;
	}

	const auto complete = _complete;
	void *const context = _context;
	const int result = _result;
	const size_t length = _received;
	_complete = nullptr;
	_phase = Phase::idle;
	irq_unlock(key);

	if (complete != nullptr) {
		complete(context, result, _response, length);
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
