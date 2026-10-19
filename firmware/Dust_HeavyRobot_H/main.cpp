#include "app_pitch.hpp"
#include "app_yaw.hpp"
#include "app_shoot.hpp"
#include "bsp_uart.hpp"
#include "health_monitor.hpp"
#include "imu_port.hpp"
#include "ins.hpp"
#include "mcu_common.hpp"
#include "remote_port.hpp"
#include "usb_port.hpp"

int main(void)
{
    bsp_uart_init();
    health_monitor_init();

    ins_init();
    imu_port_init();
    remote_port_init();
    usb_port_init();
    app_yaw_init();
    app_pitch_init();
    app_shoot_init();

    mcu_common_init();

    health_monitor_run();
}
