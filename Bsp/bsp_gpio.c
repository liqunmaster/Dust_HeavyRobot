#include "bsp_gpio.h"

static void bsp_gpio_callback(const struct device *port, struct gpio_callback *callback, gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct bsp_gpio_irq *irq = CONTAINER_OF(callback, struct bsp_gpio_irq, callback);
    if (irq->handler != NULL) {
        irq->handler(irq->user_data);
    }
}

int bsp_gpio_input_irq_init(struct bsp_gpio_irq *irq, const struct device *port, gpio_pin_t pin, gpio_flags_t interrupt_flags, bsp_gpio_irq_callback_t handler, void *user_data)
{
    if (irq == NULL || port == NULL || handler == NULL || !device_is_ready(port)) {
        return -EINVAL;
    }
    irq->port = port;
    irq->pin = pin;
    irq->interrupt_flags = interrupt_flags;
    irq->handler = handler;
    irq->user_data = user_data;
    int ret = gpio_pin_configure(port, pin, GPIO_INPUT);
    if (ret != 0) {
        return ret;
    }
    gpio_init_callback(&irq->callback, bsp_gpio_callback, BIT(pin));
    return gpio_add_callback(port, &irq->callback);
}

int bsp_gpio_exti_init(struct bsp_gpio_irq *irq, const struct gpio_dt_spec *spec, gpio_flags_t interrupt_flags, bsp_gpio_irq_callback_t handler, void *user_data)
{
    if (spec == NULL) {
        return -EINVAL;
    }

    const int ret = bsp_gpio_input_irq_init(irq, spec->port, spec->pin, interrupt_flags, handler, user_data);
    if (ret != 0) {
        return ret;
    }
    return bsp_gpio_input_irq_enable(irq);
}

int bsp_gpio_input_irq_enable(struct bsp_gpio_irq *irq)
{
    if (irq == NULL || irq->port == NULL) {
        return -EINVAL;
    }
    return gpio_pin_interrupt_configure(irq->port, irq->pin, irq->interrupt_flags);
}

int bsp_gpio_input_irq_disable(struct bsp_gpio_irq *irq)
{
    if (irq == NULL || irq->port == NULL) {
        return -EINVAL;
    }
    return gpio_pin_interrupt_configure(irq->port, irq->pin, GPIO_INT_DISABLE);
}
