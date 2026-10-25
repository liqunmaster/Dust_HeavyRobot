#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>

enum class ServoOutput : uint8_t {
	p2 = 2,
	p3 = 3,
};

class Servo final
{
public:
	explicit Servo(ServoOutput output) : _output(output) {}

	int Init();

	int SetPulseUs(uint16_t pulse_us);

	int SetAngle(uint16_t degrees);

private:
	ServoOutput _output;

	bool _initialized = false;
};
