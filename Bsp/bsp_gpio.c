#include "bsp_gpio.h"

/**
 * @brief GPIO 中断回调 触发后调用用户注册的处理函数
 *
 * @param port GPIO 设备指针
 * @param callback 回调描述符
 * @param pins 触发中断的引脚
 */
static void bsp_gpio_callback(const struct device *port, struct gpio_callback *callback, gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct bsp_gpio_irq *irq = CONTAINER_OF(callback, struct bsp_gpio_irq, callback);
    if (irq->handler != NULL) {
        irq->handler(irq->user_data);
    }
}

/**
 * @brief 初始化 GPIO 输入引脚及其中断回调 不使能中断
 *
 * @param irq 外部中断管理结构体
 * @param port GPIO 设备指针
 * @param pin 引脚号
 * @param interrupt_flags 中断触发方式标志
 * @param handler 中断回调函数
 * @param user_data 传给回调的用户上下文
 * @return 成功返回 0 失败返回错误码
 */
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

/**
 * @brief 通过设备树规格(GPIO DT spec)初始化外部中断并直接使能
 *
 * @param irq 外部中断管理结构体
 * @param spec GPIO 设备树规格描述
 * @param interrupt_flags 中断触发方式标志
 * @param handler 中断回调函数
 * @param user_data 传给回调的用户上下文
 * @return 成功返回 0 失败返回错误码
 */
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

/**
 * @brief 使能已配置的 GPIO 外部中断
 *
 * @param irq 外部中断管理结构体
 * @return 成功返回 0 失败返回错误码
 */
int bsp_gpio_input_irq_enable(struct bsp_gpio_irq *irq)
{
    if (irq == NULL || irq->port == NULL) {
        return -EINVAL;
    }
    return gpio_pin_interrupt_configure(irq->port, irq->pin, irq->interrupt_flags);
}

/**
 * @brief 禁用 GPIO 外部中断
 *
 * @param irq 外部中断管理结构体
 * @return 成功返回 0 失败返回错误码
 */
int bsp_gpio_input_irq_disable(struct bsp_gpio_irq *irq)
{
    if (irq == NULL || irq->port == NULL) {
        return -EINVAL;
    }
    return gpio_pin_interrupt_configure(irq->port, irq->pin, GPIO_INT_DISABLE);
}
