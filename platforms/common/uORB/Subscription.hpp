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
 * @file Subscription.hpp
 *
 */

#pragma once

#include "uORB.h"
#include <uORB/topics/uORBTopics.hpp>


#include "uORBManager.hpp"

namespace uORB
{

class SubscriptionCallback;

// 订阅封装基类
class Subscription
{
public:

	/**
	 * 构造函数
	 *
	 * @param id 主题对应的 uORB ORB_ID 枚举值。
	 * @param instance 多实例订阅的实例索引。
	 */
	Subscription(ORB_ID id, uint8_t instance = 0) :
		_orb_id(id),
		_instance(instance)
	{
		subscribe();
	}

	/**
	 * 构造函数
	 *
	 * @param meta 主题的 uORB 元数据，通常由 ORB_ID() 宏获取。
	 * @param instance 多实例订阅的实例索引。
	 */
	Subscription(const orb_metadata *meta = nullptr, uint8_t instance = 0) :
		_orb_id((meta == nullptr) ? ORB_ID::INVALID : static_cast<ORB_ID>(meta->o_id)),
		_instance(instance)
	{
		subscribe();
	}

	// 拷贝构造函数
	Subscription(const Subscription &other) : _orb_id(other._orb_id), _instance(other._instance) {}

	// 移动构造函数
	Subscription(const Subscription &&other) noexcept : _orb_id(other._orb_id), _instance(other._instance) {}

	// 拷贝赋值
	Subscription &operator=(const Subscription &other)
	{
		unsubscribe();
		_orb_id = other._orb_id;
		_instance = other._instance;
		return *this;
	}

	// 移动赋值
	Subscription &operator=(Subscription &&other) noexcept
	{
		unsubscribe();
		_orb_id = other._orb_id;
		_instance = other._instance;
		return *this;
	}

	~Subscription()
	{
		unsubscribe();
	}

	bool subscribe();
	void unsubscribe();

	bool valid() const { return _node != nullptr; }
	bool advertised()
	{
		if (subscribe()) {
			return Manager::is_advertised(_node);
		}

		return false;
	}

	/**
	 * 检查是否有新的消息更新。
	 */
	bool updated()
	{
		if (subscribe()) {
			return Manager::updates_available(_node, _last_generation);
		}

		return false;
	}

	/**
	 * 更新消息结构体
	 * @param dst 待更新的 uORB 消息结构体。
	 */
	bool update(void *dst)
	{
		if (subscribe()) {
			return Manager::orb_data_copy(_node, dst, _last_generation, true);
		}

		return false;
	}

	/**
	 * 复制消息结构体
	 * @param dst 待更新的 uORB 消息结构体。
	 */
	bool copy(void *dst)
	{
		if (subscribe()) {
			return Manager::orb_data_copy(_node, dst, _last_generation, false);
		}

		return false;
	}

	/**
	 * 切换订阅实例
	 * @param instance 新的多实例订阅索引
	 */
	bool ChangeInstance(uint8_t instance);

	uint8_t  get_instance() const { return _instance; }
	unsigned get_last_generation() const { return _last_generation; }
	orb_id_t get_topic() const { return get_orb_meta(_orb_id); }

	ORB_ID orb_id() const { return _orb_id; }

protected:

	friend class SubscriptionCallback;
	friend class SubscriptionCallbackWorkItem;

	void *get_node() { return _node; }

	void *_node{nullptr};

	unsigned _last_generation{0}; /**< 订阅者最后读取的发布代次 */

	ORB_ID _orb_id{ORB_ID::INVALID};
	uint8_t _instance{0};
};

// 内嵌消息数据的订阅封装类
template<class T>
class SubscriptionData : public Subscription
{
public:
	/**
	 * 构造函数
	 *
	 * @param id 主题元数据对应的 uORB ORB_ID 枚举值。
	 * @param instance 多实例订阅的实例索引。
	 */
	SubscriptionData(ORB_ID id, uint8_t instance = 0) :
		Subscription(id, instance)
	{
		copy(&_data);
	}

	/**
	 * 构造函数
	 *
	 * @param meta 主题的 uORB 元数据，通常由 ORB_ID() 宏获取。
	 * @param instance 多实例订阅的实例索引。
	 */
	SubscriptionData(const orb_metadata *meta, uint8_t instance = 0) :
		Subscription(meta, instance)
	{
		copy(&_data);
	}

	~SubscriptionData() = default;

	// 禁止拷贝构造、拷贝赋值、移动构造和移动赋值
	SubscriptionData(const SubscriptionData &) = delete;
	SubscriptionData &operator=(const SubscriptionData &) = delete;
	SubscriptionData(SubscriptionData &&) = delete;
	SubscriptionData &operator=(SubscriptionData &&) = delete;

	// 更新内嵌的消息结构体。
	bool update() { return Subscription::update((void *)(&_data)); }

	const T &get() const { return _data; }

private:

	T _data{};
};

} // uORB 命名空间
