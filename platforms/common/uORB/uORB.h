#pragma once

#include <stddef.h>
#include <stdint.h>

// 工程 Zephyr 后端的 ABI，与 PX4 字符设备 ABI 不兼容。
struct orb_metadata {
	const char *o_name;
	size_t o_size;
	uint16_t o_id;
	uint8_t o_queue;
	uint8_t o_instances;
};

using orb_id_t = const orb_metadata *;
using orb_advert_t = void *;
#define ORB_ID(name) (&__orb_##name)

orb_advert_t orb_advertise(orb_id_t meta, const void *data);
int orb_publish(orb_id_t meta, orb_advert_t handle, const void *data);
