#include "app_chassis.hpp"
#include "app_booster.hpp"
#include "app_liftup.hpp"
#include "bsp_uart.hpp"
#include "encoder.hpp"
#include "health_monitor.hpp"
#include "imu_port.hpp"
#include "ins.hpp"
#include "mcu_common.hpp"
#include "rs485_bus.hpp"

int main(void)
{
    bsp_uart_init();
    health_monitor_init();

    ins_init();
    imu_port_init();

    app_chassis_init();
    app_liftup_init();
    app_booster_init();

    Encoder encoder;
    if (encoder.Init(Encoder::default_address) == 0) {
        uint8_t request[Encoder::request_size]{};
        encoder.BuildPoll(request);
        PlatformRs485RxHandle feedback_handle{};
        (void)platform_rs485_subscribe(request, sizeof(request), Encoder::response_size, feedback_handle, 10, 20);
    }

    mcu_common_init();

    health_monitor_run();
}
