#include "uORBManager.hpp"

#include <errno.h>
#include <string.h>

namespace uORB
{

Manager *Manager::get_instance()
{
	static Manager manager;
	return &manager;
}

Node *Manager::node(orb_id_t meta, uint8_t instance)
{
	if (meta == nullptr || meta->o_id >= static_cast<unsigned>(ORB_ID::COUNT)) {
		return nullptr;
	}

	const auto &storage = topic_storage[meta->o_id];
	return storage.meta == meta && instance < meta->o_instances ? &storage.nodes[instance] : nullptr;
}

orb_advert_t Manager::advertise(orb_id_t meta, const void *data, uint8_t instance)
{
	Node *n = node(meta, instance);

	if (n == nullptr) {
		return nullptr;
	}

	const auto key = k_spin_lock(&n->lock);
	++n->publishers;
	k_spin_unlock(&n->lock, key);

	if (data != nullptr) {
		orb_publish(meta, n, data);
	}

	return n;
}

int Manager::orb_publish(orb_id_t meta, orb_advert_t handle, const void *data)
{
	auto *n = static_cast<Node *>(handle);

	if (n == nullptr || n->meta != meta || data == nullptr) {
		return -EINVAL;
	}

	const auto key = k_spin_lock(&n->lock);

	if (n->publishers == 0) {
		k_spin_unlock(&n->lock, key);
		return -ENODEV;
	}

	const unsigned slot = n->generation % meta->o_queue;
	memcpy(n->data + slot * meta->o_size, data, meta->o_size);
	++n->generation;
	n->published = true;
	k_spin_unlock(&n->lock, key);
	return 0;
}

int Manager::orb_unadvertise(orb_advert_t handle)
{
	auto *n = static_cast<Node *>(handle);

	if (n == nullptr) {
		return -EINVAL;
	}

	const auto key = k_spin_lock(&n->lock);

	if (n->publishers != 0) {
		--n->publishers;
	}

	k_spin_unlock(&n->lock, key);
	return 0;
}

unsigned Manager::orb_get_queue_size(orb_advert_t handle)
{
	const auto *n = static_cast<const Node *>(handle);
	return n != nullptr ? n->meta->o_queue : 0;
}

void *Manager::orb_add_internal_subscriber(ORB_ID id, uint8_t instance, unsigned *generation)
{
	Node *n = node(get_orb_meta(id), instance);

	if (n == nullptr || generation == nullptr) {
		return nullptr;
	}

	const auto key = k_spin_lock(&n->lock);
	++n->subscribers;
	*generation = n->generation - (n->published ? 1U : 0U);
	k_spin_unlock(&n->lock, key);
	return n;
}

void Manager::orb_remove_internal_subscriber(void *handle)
{
	auto *n = static_cast<Node *>(handle);
	const auto key = k_spin_lock(&n->lock);

	if (n->subscribers != 0) {
		--n->subscribers;
	}

	k_spin_unlock(&n->lock, key);
}

bool Manager::orb_device_node_exists(ORB_ID id, uint8_t instance)
{
	return node(get_orb_meta(id), instance) != nullptr;
}

bool Manager::is_advertised(void *handle)
{
	auto *n = static_cast<Node *>(handle);
	const auto key = k_spin_lock(&n->lock);
	const bool result = n->publishers != 0;
	k_spin_unlock(&n->lock, key);
	return result;
}

bool Manager::updates_available(void *handle, unsigned generation)
{
	auto *n = static_cast<Node *>(handle);
	const auto key = k_spin_lock(&n->lock);
	const bool result = n->published && n->generation != generation;
	k_spin_unlock(&n->lock, key);
	return result;
}

bool Manager::orb_data_copy(void *handle, void *dst, unsigned &generation, bool only_updated)
{
	auto *n = static_cast<Node *>(handle);

	if (n == nullptr || dst == nullptr) {
		return false;
	}

	const auto key = k_spin_lock(&n->lock);

	if (!n->published || (only_updated && generation == n->generation)) {
		k_spin_unlock(&n->lock, key);
		return false;
	}

	unsigned next = n->generation;

	if (only_updated) {
		unsigned unread = n->generation - generation;

		if (unread > n->meta->o_queue) {
			n->overwritten += unread - n->meta->o_queue;
			unread = n->meta->o_queue;
		}

		next = n->generation - unread + 1U;
	}

	memcpy(dst, n->data + ((next - 1U) % n->meta->o_queue) * n->meta->o_size, n->meta->o_size);
	generation = next;
	k_spin_unlock(&n->lock, key);
	return true;
}

} // uORB 命名空间

orb_advert_t orb_advertise(orb_id_t meta, const void *data)
{
	return uORB::Manager::advertise(meta, data);
}

int orb_publish(orb_id_t meta, orb_advert_t handle, const void *data)
{
	return uORB::Manager::orb_publish(meta, handle, data);
}
