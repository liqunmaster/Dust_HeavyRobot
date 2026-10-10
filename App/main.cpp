#include "app_chassis.hpp"
#include "app_booster.hpp"
#include "app_liftup.hpp"
#include "bsp_uart.hpp"
#include "health_monitor.hpp"
#include "imu_port.hpp"
#include "ins.hpp"
#include "remote_port.hpp"
#include "usb_port.hpp"

int main(void)
{
    bsp_uart_init();
    app_chassis_init();
    app_liftup_init();
    // app_booster_init();
    health_monitor_init();
    remote_port_init();
    ins_init();
    imu_port_init();
    usb_port_init();

    while (1) {
        health_monitor_poll();
        k_sleep(K_MSEC(10));
    }
}
