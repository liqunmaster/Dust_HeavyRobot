#include "pitch_control.hpp"
#include "yaw_control.hpp"
#include "shoot_control.hpp"
#include "dvc_uart.hpp"
#include "health_monitor.hpp"
#include "attitude_estimator.hpp"
#include "control_parameters.hpp"
#include "mcu_bridge_h.hpp"
#include "remote_port.hpp"
#include "usb_port.hpp"

int main(void)
{
	(void)manual_control_init(board::manual_config());
	bsp_uart_init();
	health_monitor_init();

	imu_port_init(board::attitude_config());
	remote_port_init();
	usb_port_init();
	app_yaw_init(board::yaw_config());
	app_pitch_init();
	app_shoot_init();

	mcu_common_init();

	health_monitor_run();
}
