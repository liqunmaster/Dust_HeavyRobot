#include "bsp_usb.hpp"

#define USB_NODE DT_NODELABEL(usb0)
#define USB_CONFIG_LENGTH (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static bool ready;
K_SEM_DEFINE(usb_event_sem, 0, 1);

static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCAFE,
    .idProduct = 0x5361,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const tusb_desc_device_qualifier_t qualifier_descriptor = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
};

static const uint8_t full_speed_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, USB_CONFIG_LENGTH, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 16, 0x02, 0x82, 64),
};

static const uint8_t high_speed_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, USB_CONFIG_LENGTH, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 16, 0x02, 0x82, 512),
};

static uint8_t other_speed_configuration[USB_CONFIG_LENGTH];

static uint16_t string_descriptor[32];

static const char *const strings[] = {
    NULL,
    "Dust Robotics",
    "HeavyHero USB CDC",
    "53610001",
    "USB CDC",
};

/**
 * @brief TinyUSB 设备描述符回调 返回设备描述符数据
 *
 * @return 设备描述符数据指针
 */
extern "C" uint8_t const *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&device_descriptor;
}

/**
 * @brief TinyUSB 设备限定符描述符回调
 *
 * @return 限定符描述符数据指针
 */
extern "C" uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (const uint8_t *)&qualifier_descriptor;
}

/**
 * @brief TinyUSB 配置描述符回调 按当前速度返回对应配置
 *
 * @param index 配置索引
 * @return 高速或全速配置描述符数据指针
 */
extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return tud_speed_get() == TUSB_SPEED_HIGH ? high_speed_configuration : full_speed_configuration;
}

/**
 * @brief TinyUSB 其它速度配置描述符回调
 *
 * @param index 配置索引
 * @return 其它速度配置描述符数据指针
 */
extern "C" uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index;
    memcpy(other_speed_configuration, tud_speed_get() == TUSB_SPEED_HIGH ? full_speed_configuration : high_speed_configuration, sizeof(other_speed_configuration));
    other_speed_configuration[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
    return other_speed_configuration;
}

/**
 * @brief TinyUSB 字符串描述符回调 返回指定索引的字符串
 *
 * @param index 字符串索引
 * @param langid 语言 ID
 * @return 字符串描述符数据指针
 */
extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    if (index == 0) {
        string_descriptor[1] = 0x0409;
        string_descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
        return string_descriptor;
    }
    if (index >= sizeof(strings) / sizeof(strings[0])) {
        return NULL;
    }

    const char *text = strings[index];
    size_t count = strlen(text);
    if (count > 31) {
        count = 31;
    }
    for (size_t i = 0; i < count; ++i) {
        string_descriptor[i + 1] = (uint8_t)text[i];
    }
    string_descriptor[0] = (TUSB_DESC_STRING << 8) | (uint16_t)(2 * count + 2);
    return string_descriptor;
}

/**
 * @brief USB 中断处理函数 驱动 TinyUSB 并唤醒事件信号量
 *
 * @param arg 中断参数
 */
static void usb_irq(const void *arg)
{
    (void)arg;
    tusb_int_handler(0, true);
    k_sem_give(&usb_event_sem);
}

/**
 * @brief 初始化 USB 外设与 TinyUSB 栈
 *
 * @return 成功返回 0 失败返回负错误码
 */
int bsp_usb_init(void)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (ready) {
        return 0;
    }
    clock_add_to_group(clock_usb0, 0);
    usb_hcd_set_power_ctrl_polarity(HPM_USB0, true);
    IRQ_CONNECT(DT_IRQN(USB_NODE), DT_IRQ(USB_NODE, priority), usb_irq, NULL, 0);

    const tusb_rhport_init_t init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    if (!tusb_init(0, &init)) {
        return -EIO;
    }
    irq_enable(DT_IRQN(USB_NODE));
    k_sem_give(&usb_event_sem);
    ready = true;
    return 0;
}

/**
 * @brief USB 任务处理 供主循环轮询 TinyUSB
 *
 */
void bsp_usb_task(void)
{
    if (ready) {
        tud_task();
    }
}

/**
 * @brief 等待 USB 中断事件 可通过超时退出
 *
 * @param timeout 等待超时
 */
void bsp_usb_wait_event(k_timeout_t timeout)
{
    if (!ready) {
        return;
    }
    k_sem_take(&usb_event_sem, timeout);
}

/**
 * @brief 从 USB CDC 接收数据
 *
 * @param data 接收缓冲区
 * @param length 期望接收的字节数
 * @return 实际接收字节数 失败返回负错误码
 */
int bsp_usb_receive(void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready) {
        return -ENODEV;
    }
    if (data == NULL && length != 0) {
        return -EINVAL;
    }
    if (length > INT_MAX) {
        return -EMSGSIZE;
    }
    if (length == 0) {
        return 0;
    }
    return (int)tud_cdc_read(data, (uint32_t)length);
}

/**
 * @brief 通过 USB CDC 发送数据
 *
 * @param data 待发送缓冲区
 * @param length 待发送的字节数
 * @return 实际发送字节数 失败返回负错误码
 */
int bsp_usb_transmit(const void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready) {
        return -ENODEV;
    }
    if (data == NULL && length != 0) {
        return -EINVAL;
    }
    if (length > INT_MAX) {
        return -EMSGSIZE;
    }
    if (length == 0) {
        return 0;
    }
    if (!bsp_usb_connected()) {
        return -ENOTCONN;
    }

    const uint32_t count = tud_cdc_write(data, (uint32_t)length);
    if (count > 0) {
        (void)tud_cdc_write_flush();
    }
    return count > 0 ? (int)count : -ENOBUFS;
}

/**
 * @brief 查询 USB CDC 是否已连接就绪
 *
 * @return 已连接返回 true 否则返回 false
 */
bool bsp_usb_connected(void)
{
    return ready && tud_cdc_ready();
}
