#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PIN_BUTTON_UNLOCK  10  // Choose an available GPIO

void hardware_init(void);
void hardware_button_init(void);
void hardware_set_5v(bool enable);
void hardware_set_7v8(bool enable);
void hardware_servo_lock(void);
void hardware_servo_unlock(void);

// Generic PWM for any servo/tool
void hardware_pwm_init(int gpio, int timer, int channel, int freq_hz, int resolution_bits);
void hardware_pwm_set_duty(int channel, uint32_t duty);