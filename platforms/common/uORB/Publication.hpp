/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Publication.hpp
 *
 */

#pragma once


#include "uORB.h"
#include "uORBManager.hpp"
#include <uORB/topics/uORBTopics.hpp>

namespace uORB
{

class PublicationBase
{
public:

	bool advertised() const { return _handle != nullptr; }

	bool unadvertise()
	{
		if (_handle == nullptr) { return true; }

		const bool result = Manager::orb_unadvertise(_handle) == 0;

		if (result) { _handle = nullptr; }

		return result;
	}

	orb_id_t get_topic() const { return get_orb_meta(_orb_id); }

protected:

	PublicationBase(ORB_ID id) : _orb_id(id) {}
	PublicationBase(const PublicationBase &) = delete;
	PublicationBase &operator=(const PublicationBase &) = delete;

	~PublicationBase()
	{
		if (_handle != nullptr) {
			// 不自动撤销带队列的发布主题（例如 vehicle_command）
			if (Manager::orb_get_queue_size(_handle) == 1) {
				unadvertise();
			}
		}
	}

	orb_advert_t _handle{nullptr};
	const ORB_ID _orb_id;
};

/**
 * uORB 发布封装类
 */
template<typename T>
class Publication : public PublicationBase
{
public:

	/**
	 * 构造函数
	 *
	 * @param meta 主题的 uORB 元数据，通常由 ORB_ID() 宏获取。
	 */
	Publication(ORB_ID id) : PublicationBase(id) {}
	Publication(const orb_metadata *meta) : PublicationBase(static_cast<ORB_ID>(meta->o_id)) {}

	bool advertise()
	{
		if (!advertised()) {
			_handle = orb_advertise(get_topic(), nullptr);
		}

		return advertised();
	}

	/**
	 * 发布消息结构体
	 * @param data 待更新的 uORB 消息结构体。
	 */
	bool publish(const T &data)
	{
		if (get_topic() == nullptr || get_topic()->o_size != sizeof(T)) {
			return false;
		}

		if (!advertised()) {
			advertise();
		}

		return (Manager::orb_publish(get_topic(), _handle, &data) == 0);
	}
};

/**
 * 内嵌消息数据的发布封装类。
 */
template<typename T>
class PublicationData : public Publication<T>
{
public:
	/**
	 * 构造函数
	 *
	 * @param meta 主题的 uORB 元数据，通常由 ORB_ID() 宏获取。
	 */
	PublicationData(ORB_ID id) : Publication<T>(id) {}
	PublicationData(const orb_metadata *meta) : Publication<T>(meta) {}

	T	 &get() { return _data; }
	void	set(const T &data) { _data = data; }

	// 发布内嵌的消息结构体。
	bool	update() { return Publication<T>::publish(_data); }
	bool	update(const T &data)
	{
		_data = data;
		return Publication<T>::publish(_data);
	}

private:
	T _data{};
};

} // uORB 命名空间
