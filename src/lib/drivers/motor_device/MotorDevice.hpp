#pragma once

#include <stdint.h>
#include <uORB/topics/motor_feedback.h>

// 物理执行器服务；控制器无需接触 CAN 帧或设备标识。
struct MotorCounters {
	uint32_t tx_completed{0};
	uint32_t tx_errors{0};
	uint32_t rx_received{0};
	uint32_t rx_dropped{0};
};

class MotorDevice
{
public:
	virtual int init() = 0;
	virtual bool refresh(uint32_t max_age_ms) = 0;
	virtual const motor_feedback_s &measurement() const = 0;
	virtual int set_current(float current_a) = 0;
	virtual int send() = 0;
	virtual int send_torque(float torque_nm) = 0;
	virtual int enable() = 0;
	virtual int recover(bool clear_error) = 0;
	virtual MotorCounters counters() const = 0;
protected:
	// 板级持有的静态设备不可通过服务接口删除。
	~MotorDevice() = default;
};
