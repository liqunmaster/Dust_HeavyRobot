#include "chassis_control.hpp"
#include "booster_control.hpp"
#include "lift_control.hpp"
#include "dvc_uart.hpp"
#include "encoder.hpp"
#include "health_monitor.hpp"
#include "attitude_estimator.hpp"
#include "control_parameters.hpp"
#include "mcu_bridge_l.hpp"
#include "rs485_bus.hpp"

int main(void)
{
	(void)manual_control_init(board::manual_config());
	bsp_uart_init();
	health_monitor_init();

	imu_port_init(board::attitude_config());

	app_chassis_init(board::chassis_config());
	app_liftup_init(board::lift_config());
	app_booster_init(board::booster_config());

	Encoder encoder;

	if (encoder.Init(Encoder::default_address) == 0) {
		uint8_t request[Encoder::request_size] {};
		encoder.BuildPoll(request);
		PlatformRs485RxHandle feedback_handle{};
		(void)platform_rs485_subscribe(request, sizeof(request), Encoder::response_size, feedback_handle, 10, 20);
	}

	mcu_common_init();

	health_monitor_run();
}
