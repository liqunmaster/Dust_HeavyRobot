#include "dvc_usb.hpp"

#define USB_NODE DT_NODELABEL(usb0)

static UsbState usb_state;

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
	TUD_CONFIG_DESCRIPTOR(1, 2, 0, UsbState::configuration_length, 0, 100),
	TUD_CDC_DESCRIPTOR(0, 4, 0x81, 16, 0x02, 0x82, 64),
};

static const uint8_t high_speed_configuration[] = {
	TUD_CONFIG_DESCRIPTOR(1, 2, 0, UsbState::configuration_length, 0, 100),
	TUD_CDC_DESCRIPTOR(0, 4, 0x81, 16, 0x02, 0x82, 512),
};

static const char *const strings[] = {
	NULL,
	"Dust Robotics",
	"HeavyHero USB CDC",
	"53610001",
	"USB CDC",
};

extern "C" uint8_t const *tud_descriptor_device_cb(void)
{
	return (const uint8_t *)&device_descriptor;
}

extern "C" uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
	return (const uint8_t *)&qualifier_descriptor;
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
	(void)index;
	return tud_speed_get() == TUSB_SPEED_HIGH ? high_speed_configuration : full_speed_configuration;
}

extern "C" uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
	(void)index;
	memcpy(usb_state.other_speed_configuration,
	       tud_speed_get() == TUSB_SPEED_HIGH ? full_speed_configuration : high_speed_configuration,
	       sizeof(usb_state.other_speed_configuration));
	usb_state.other_speed_configuration[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
	return usb_state.other_speed_configuration;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void)langid;

	if (index == 0) {
		usb_state.string_descriptor[1] = 0x0409;
		usb_state.string_descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
		return usb_state.string_descriptor;
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
		usb_state.string_descriptor[i + 1] = (uint8_t)text[i];
	}

	usb_state.string_descriptor[0] = (TUSB_DESC_STRING << 8) | (uint16_t)(2 * count + 2);
	return usb_state.string_descriptor;
}

static void UsbIrq(const void *arg)
{
	(void)arg;
	++usb_state.irq_count;

	if ((HPM_USB0->USBSTS & USB_USBSTS_URI_MASK) != 0U) {
		++usb_state.reset_count;
	}

	if (HPM_USB0->ENDPTSETUPSTAT != 0U) {
		++usb_state.setup_count;
	}

	tusb_int_handler(0, true);
}

int bsp_usb_init(void)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (usb_state.ready) {
		return 0;
	}

	clock_add_to_group(clock_usb0, 0);
	usb_hcd_set_power_ctrl_polarity(HPM_USB0, true);
	IRQ_CONNECT(DT_IRQN(USB_NODE), DT_IRQ(USB_NODE, priority), UsbIrq, NULL, 0);

	const tusb_rhport_init_t init = {
		.role = TUSB_ROLE_DEVICE,
		.speed = TUSB_SPEED_AUTO,
	};

	if (!tusb_init(0, &init)) {
		return -EIO;
	}

	irq_disable(DT_IRQN(USB_NODE));
	HPM_USB0->USBCMD &= ~USB_USBCMD_RS_MASK;
	HPM_USB0->USBMODE |= USB_USBMODE_SLOM_MASK;
	HPM_USB0->PORTSC1 &= ~(USB_PORTSC1_STS_MASK | USB_PORTSC1_PTW_MASK);
	HPM_USB0->USBCMD |= USB_USBCMD_RS_MASK;
	irq_enable(DT_IRQN(USB_NODE));
	usb_state.ready = true;
	return 0;
}

void bsp_usb_task(void)
{
	if (usb_state.ready) {
		tud_task();
	}
}

int bsp_usb_receive(void *data, size_t length)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (!usb_state.ready) {
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

int bsp_usb_transmit(const void *data, size_t length)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (!usb_state.ready) {
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

bool bsp_usb_connected(void)
{
	return usb_state.ready && tud_cdc_ready();
}
