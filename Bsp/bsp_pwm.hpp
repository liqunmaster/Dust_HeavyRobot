#pragma once

// PWM0 output 7 is reserved exclusively for the IMU heater.
int bsp_pwm_heater_init();
void bsp_pwm_heater_set_duty(float duty);
