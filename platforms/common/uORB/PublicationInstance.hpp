#pragma once

#include "uORBManager.hpp"

namespace uORB
{

// 由板级配置显式绑定实例，不采用上游 PublicationMulti 的自动索引。
template<typename T>
class PublicationInstance
{
public:
	PublicationInstance(orb_id_t meta, uint8_t instance) : _meta(meta), _instance(instance) {}
	~PublicationInstance() { if (_handle != nullptr) { Manager::orb_unadvertise(_handle); } }
	PublicationInstance(const PublicationInstance &) = delete;
	PublicationInstance &operator=(const PublicationInstance &) = delete;
	bool publish(const T &data)
	{
		if (_meta == nullptr || _meta->o_size != sizeof(T)) {
			return false;
		}

		if (_handle == nullptr) { _handle = Manager::advertise(_meta, nullptr, _instance); }

		return Manager::orb_publish(_meta, _handle, &data) == 0;
	}

private:
	orb_id_t _meta;
	uint8_t _instance;
	orb_advert_t _handle{nullptr};
};

}
