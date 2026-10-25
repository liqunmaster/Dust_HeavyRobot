#pragma once

// PWM0 的输出 7 专用于 IMU 加热器。
int bsp_pwm_heater_init();
void bsp_pwm_heater_set_duty(float duty);
