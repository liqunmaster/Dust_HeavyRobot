#pragma once

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void (*bsp_gpio_irq_callback_t)(void *user_data);

    struct bsp_gpio_irq
    {
        const struct device *port;
        gpio_pin_t pin;
        gpio_flags_t interrupt_flags;
        struct gpio_callback callback;
        bsp_gpio_irq_callback_t handler;
        void *user_data;
    };

    int bsp_gpio_input_irq_init(struct bsp_gpio_irq *irq, const struct device *port, gpio_pin_t pin, gpio_flags_t interrupt_flags, bsp_gpio_irq_callback_t handler, void *user_data);

    int bsp_gpio_exti_init(struct bsp_gpio_irq *irq, const struct gpio_dt_spec *spec, gpio_flags_t interrupt_flags, bsp_gpio_irq_callback_t handler, void *user_data);

    int bsp_gpio_input_irq_enable(struct bsp_gpio_irq *irq);

    int bsp_gpio_input_irq_disable(struct bsp_gpio_irq *irq);

#ifdef __cplusplus
}
#endif
