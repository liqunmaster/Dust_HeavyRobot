#include "dvc_uart.hpp"
#include <string.h>

extern "C" void __stdout_hook_install(int (*hook)(int));

namespace
{
UartDmaBuffers dma_buffers[7] __attribute__((section("AHB_SRAM"), aligned(4)));

int StdoutCharacter(int character)
{
	const uint8_t byte = static_cast<uint8_t>(character);
	return uart3.Transmit(&byte, 1) == 1 ? byte : EOF;
}
}

Uart uart0{DEVICE_DT_GET(DT_NODELABEL(uart0)), dma_buffers[0]};
Uart uart1{DEVICE_DT_GET(DT_NODELABEL(uart1)), dma_buffers[1]};
Uart uart2{DEVICE_DT_GET(DT_NODELABEL(uart2)), dma_buffers[2]};
Uart uart3{DEVICE_DT_GET(DT_NODELABEL(uart3)), dma_buffers[3]};
Uart uart4{DEVICE_DT_GET(DT_NODELABEL(uart4)), dma_buffers[4]};
Uart uart5{DEVICE_DT_GET(DT_NODELABEL(uart5)), dma_buffers[5]};
Uart uart6{DEVICE_DT_GET(DT_NODELABEL(uart6)), dma_buffers[6]};

Uart *bsp_uart_get(const struct device *device)
{
	if (device == nullptr) {
		return &uart3;
	}

	Uart *const ports[] = {&uart0, &uart1, &uart2, &uart3, &uart4, &uart5, &uart6};

	for (auto *port : ports) {
		if (port->GetDevice() == device) {
			return port;
		}
	}

	return nullptr;
}

bool Uart::IsReady() const
{
	return atomic_get(&_init_state) == 2;
}

int Uart::SetRxNotify(void (*notify)(void *), void *context)
{
	if (!IsReady()) {
		return -ENODEV;
	}

	const unsigned int key = irq_lock();
	rx_notify_ = notify;
	_rx_notify_context = context;
	irq_unlock(key);
	return 0;
}

int Uart::SetTxNotify(void (*notify)(void *, int), void *context)
{
	if (!IsReady()) {
		return -ENODEV;
	}

	const unsigned int key = irq_lock();
	tx_notify_ = notify;
	_tx_notify_context = context;
	irq_unlock(key);
	return 0;
}

int Uart::Init()
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (IsReady()) {
		return 0;
	}

	if (!atomic_cas(&_init_state, 0, 1)) {
		return -EBUSY;
	}

	if (_device == nullptr || !device_is_ready(_device)) {
		atomic_clear(&_init_state);
		return -ENODEV;
	}

	k_mutex_init(&_tx_lock);
	k_sem_init(&_tx_done, 0, 1);
	ring_buffer_init(&_rx_buffer, _rx_storage, sizeof(_rx_storage));
	ring_buffer_init(&_tx_buffer, _tx_storage, sizeof(_tx_storage));
	atomic_clear(&_rx_overflow);
	atomic_clear(&_tx_busy);
	_rx_active = false;
	_rx_owned[0] = false;
	_rx_owned[1] = false;

	int result = uart_callback_set(_device, UartCallback, this);

	if (result != 0) {
		atomic_clear(&_init_state);
		return result;
	}

	if (this != &uart3) {
		result = StartRx();

		if (result != 0) {
			(void)uart_callback_set(_device, nullptr, nullptr);
			atomic_clear(&_init_state);
			return result;
		}
	}

	atomic_set(&_init_state, 2);

	if (this == &uart3) {
		__stdout_hook_install(StdoutCharacter);
	}

	return 0;
}

void Uart::UartCallback(const struct device *device, struct uart_event *event, void *user_data)
{
	auto *port = static_cast<Uart *>(user_data);

	if (port == nullptr || event == nullptr || device != port->_device) {
		return;
	}

	const unsigned int key = irq_lock();
	port->HandleEvent(*event);
	irq_unlock(key);
}

void Uart::HandleEvent(const struct uart_event &event)
{
	switch (event.type) {
	case UART_RX_RDY:
		if (ring_buffer_write(&_rx_buffer, event.data.rx.buf + event.data.rx.offset, event.data.rx.len) != event.data.rx.len) {
			atomic_set(&_rx_overflow, 1);
		}

		if (rx_notify_ != nullptr) {
			rx_notify_(_rx_notify_context);
		}

		break;

	case UART_RX_BUF_REQUEST:
		for (size_t i = 0; i < 2; ++i) {
			if (!_rx_owned[i]) {
				_rx_owned[i] = true;

				if (uart_rx_buf_rsp(_device, _dma.rx[i], sizeof(_dma.rx[i])) != 0) {
					_rx_owned[i] = false;
				}

				break;
			}
		}

		break;

	case UART_RX_BUF_RELEASED:
		for (size_t i = 0; i < 2; ++i) {
			if (event.data.rx_buf.buf == _dma.rx[i]) {
				_rx_owned[i] = false;
			}
		}

		break;

	case UART_RX_DISABLED:
		_rx_active = false;
		atomic_set(&_rx_overflow, 1);

		if (rx_notify_ != nullptr) {
			rx_notify_(_rx_notify_context);
		}

		break;

	case UART_RX_STOPPED:
		atomic_set(&_rx_overflow, 1);

		if (rx_notify_ != nullptr) {
			rx_notify_(_rx_notify_context);
		}

		break;

	case UART_TX_DONE:
	case UART_TX_ABORTED:
		_tx_count = event.data.tx.len;
		_tx_error = event.type == UART_TX_DONE ? 0 : -EIO;
		atomic_clear(&_tx_busy);
		k_sem_give(&_tx_done);

		if (tx_notify_ != nullptr) {
			tx_notify_(_tx_notify_context, _tx_error);
		}

		break;

	default:
		break;
	}
}

int Uart::StartRx()
{
	const unsigned int key = irq_lock();
	_rx_owned[0] = true;
	_rx_owned[1] = false;
	_rx_active = true;
	const int result = uart_rx_enable(_device, _dma.rx[0], sizeof(_dma.rx[0]), 1000);

	if (result != 0) {
		_rx_active = false;
		_rx_owned[0] = false;
	}

	irq_unlock(key);
	return result;
}

int Uart::Receive(void *data, size_t length)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (!IsReady()) {
		return -ENODEV;
	}

	if (data == nullptr && length != 0) {
		return -EINVAL;
	}

	if (length > INT_MAX) {
		return -EMSGSIZE;
	}

	if (length == 0) {
		return 0;
	}

	const unsigned int key = irq_lock();
	const int result = _rx_active ? 0 : StartRx();
	const bool overflow = atomic_set(&_rx_overflow, 0) != 0;

	if (overflow) {
		ring_buffer_clear(&_rx_buffer);
	}

	const int count = overflow ? -ENOBUFS : static_cast<int>(ring_buffer_read(&_rx_buffer, data, length));
	irq_unlock(key);
	return count != 0 ? count : result;
}

int Uart::ClearRx()
{
	if (!IsReady()) {
		return -ENODEV;
	}

	const unsigned int key = irq_lock();
	ring_buffer_clear(&_rx_buffer);
	atomic_clear(&_rx_overflow);
	irq_unlock(key);
	return 0;
}

int Uart::TransmitAsync(const void *data, size_t length)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (!IsReady()) {
		return -ENODEV;
	}

	if (data == nullptr || length == 0 || length > sizeof(_dma.tx)) {
		return -EINVAL;
	}

	if (k_mutex_lock(&_tx_lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	if (!atomic_cas(&_tx_busy, 0, 1)) {
		k_mutex_unlock(&_tx_lock);
		return -EBUSY;
	}

	memcpy(_dma.tx, data, length);
	_tx_count = 0;
	_tx_error = 0;
	const int ret = uart_tx(_device, _dma.tx, length, SYS_FOREVER_US);

	if (ret != 0) {
		atomic_clear(&_tx_busy);
	}

	k_mutex_unlock(&_tx_lock);
	return ret;
}

int Uart::AbortTransmit()
{
	return atomic_get(&_tx_busy) ? uart_tx_abort(_device) : 0;
}

int Uart::Transmit(const void *data, size_t length)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (!IsReady()) {
		return -ENODEV;
	}

	if (data == nullptr && length != 0) {
		return -EINVAL;
	}

	if (length > INT_MAX) {
		return -EMSGSIZE;
	}

	if (length == 0) {
		return 0;
	}

	k_mutex_lock(&_tx_lock, K_FOREVER);
	const auto *bytes = static_cast<const uint8_t *>(data);
	size_t sent = 0;
	int result = atomic_get(&_tx_busy) ? -EBUSY : 0;

	while (result == 0 && sent < length) {
		const size_t chunk = ring_buffer_write(&_tx_buffer, bytes + sent, length - sent);

		if (chunk == 0) {
			result = -ENOBUFS;
			break;
		}

		ring_buffer_read(&_tx_buffer, _dma.tx, chunk);
		k_sem_reset(&_tx_done);
		_tx_count = 0;
		_tx_error = 0;
		atomic_set(&_tx_busy, 1);
		result = uart_tx(_device, _dma.tx, chunk, SYS_FOREVER_US);

		if (result != 0) {
			atomic_clear(&_tx_busy);
			break;
		}

		if (k_sem_take(&_tx_done, K_MSEC(1000)) != 0) {
			const unsigned int key = irq_lock();

			if (atomic_get(&_tx_busy)) {
				(void)uart_tx_abort(_device);
				result = -ETIMEDOUT;
			}

			irq_unlock(key);
		}

		sent += _tx_count;

		if (result == 0) {
			result = _tx_error;
		}

		if (result == 0 && _tx_count != chunk) {
			result = -EIO;
		}
	}

	k_mutex_unlock(&_tx_lock);
	return sent != 0 ? static_cast<int>(sent) : result;
}

void bsp_uart_init(const struct device *device)
{
	auto *port = bsp_uart_get(device);

	if (port != nullptr) {
		port->Init();
	}
}

int bsp_uart_receive(void *data, size_t length, const struct device *device)
{
	auto *port = bsp_uart_get(device);
	return port != nullptr ? port->Receive(data, length) : -ENODEV;
}

int bsp_uart_transmit(const void *data, size_t length, const struct device *device)
{
	auto *port = bsp_uart_get(device);
	return port != nullptr ? port->Transmit(data, length) : -ENODEV;
}

extern "C" int printf(const char *format, ...)
{
	if (k_is_in_isr()) {
		errno = EWOULDBLOCK;
		return EOF;
	}

	char message[256];
	va_list args;
	va_start(args, format);
	const int length = vsnprintf(message, sizeof(message), format, args);
	va_end(args);

	if (length < 0) {
		return length;
	}

	if (static_cast<size_t>(length) >= sizeof(message)) {
		errno = EMSGSIZE;
		return EOF;
	}

	const int result = uart3.Transmit(message, length);

	if (result != length) {
		errno = result < 0 ? -result : EIO;
		return EOF;
	}

	return result;
}
