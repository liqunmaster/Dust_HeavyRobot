#include "bsp_usb.h"

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

uint8_t const *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&device_descriptor;
}

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (const uint8_t *)&qualifier_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return tud_speed_get() == TUSB_SPEED_HIGH ? high_speed_configuration : full_speed_configuration;
}

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index;
    memcpy(other_speed_configuration, tud_speed_get() == TUSB_SPEED_HIGH ? full_speed_configuration : high_speed_configuration, sizeof(other_speed_configuration));
    other_speed_configuration[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
    return other_speed_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    if (index == 0U) {
        string_descriptor[1] = 0x0409U;
        string_descriptor[0] = (TUSB_DESC_STRING << 8U) | 4U;
        return string_descriptor;
    }
    if (index >= sizeof(strings) / sizeof(strings[0])) {
        return NULL;
    }

    const char *text = strings[index];
    size_t count = strlen(text);
    if (count > 31U) {
        count = 31U;
    }
    for (size_t i = 0; i < count; ++i) {
        string_descriptor[i + 1U] = (uint8_t)text[i];
    }
    string_descriptor[0] = (TUSB_DESC_STRING << 8U) | (uint16_t)(2U * count + 2U);
    return string_descriptor;
}

static void usb_irq(const void *arg)
{
    (void)arg;
    tusb_int_handler(0, true);
    k_sem_give(&usb_event_sem);
}

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

void bsp_usb_task(void)
{
    if (ready) {
        tud_task();
    }
}

int bsp_usb_wait_event(k_timeout_t timeout)
{
    if (!ready) {
        return -ENODEV;
    }
    return k_sem_take(&usb_event_sem, timeout);
}

int bsp_usb_receive(void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready) {
        return -ENODEV;
    }
    if (data == NULL && length != 0U) {
        return -EINVAL;
    }
    if (length > INT_MAX) {
        return -EMSGSIZE;
    }
    if (length == 0U) {
        return 0;
    }
    return (int)tud_cdc_read(data, (uint32_t)length);
}

int bsp_usb_transmit(const void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready) {
        return -ENODEV;
    }
    if (data == NULL && length != 0U) {
        return -EINVAL;
    }
    if (length > INT_MAX) {
        return -EMSGSIZE;
    }
    if (length == 0U) {
        return 0;
    }
    if (!bsp_usb_connected()) {
        return -ENOTCONN;
    }

    const uint32_t count = tud_cdc_write(data, (uint32_t)length);
    if (count > 0U) {
        (void)tud_cdc_write_flush();
    }
    return count > 0U ? (int)count : -ENOBUFS;
}

bool bsp_usb_connected(void)
{
    return ready && tud_cdc_ready();
}
