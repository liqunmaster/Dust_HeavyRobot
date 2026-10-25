#pragma once

#include "uORB.h"
#include <uORB/topics/uORBTopics.hpp>
#include <zephyr/spinlock.h>

namespace uORB
{

struct Node {
	const orb_metadata *meta;
	uint8_t *data;
	k_spinlock lock{};
	unsigned generation{0};
	unsigned publishers{0};
	unsigned subscribers{0};
	uint32_t overwritten{0};
	bool published{false};
};

struct TopicStorage {
	const orb_metadata *meta;
	Node *nodes;
};

extern TopicStorage topic_storage[];

// 上游发布/订阅封装使用的静态 Zephyr 后端。
// 所有操作均有明确边界，不分配堆内存。
class Manager
{
public:
	static Manager *get_instance();
	static Node *node(orb_id_t meta, uint8_t instance = 0);
	static orb_advert_t advertise(orb_id_t meta, const void *data, uint8_t instance = 0);
	static int orb_publish(orb_id_t meta, orb_advert_t handle, const void *data);
	static int orb_unadvertise(orb_advert_t handle);
	static unsigned orb_get_queue_size(orb_advert_t handle);
	static void *orb_add_internal_subscriber(ORB_ID id, uint8_t instance, unsigned *generation);
	static void orb_remove_internal_subscriber(void *handle);
	static bool orb_device_node_exists(ORB_ID id, uint8_t instance);
	static bool is_advertised(void *handle);
	static bool updates_available(void *handle, unsigned generation);
	static bool orb_data_copy(void *handle, void *dst, unsigned &generation, bool only_updated);
};

} // uORB 命名空间
