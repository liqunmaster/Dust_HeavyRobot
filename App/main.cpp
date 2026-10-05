#include "bsp_uart.hpp"
#include "cpu_usage.hpp"
#include "fdcan_port.hpp"
#include "imu_port.hpp"
#include "ins.hpp"
#include "input.hpp"
#include "remote_port.hpp"
#include "usb_port.hpp"


int main(void)
{
    (void)bsp_uart_init();
    (void)fdcan_port_init();
    (void)remote_port_init();
    (void)ins_init();
    (void)imu_port_init();
    (void)usb_port_init();

    while (1) {
        cpu_usage_supervisor();
    }
}
