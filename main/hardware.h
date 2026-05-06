#pragma once

#include <stdbool.h>

#define PIN_BUTTON_UNLOCK  14  // Choose an available GPIO

void hardware_init(void);
void hardware_set_5v(bool enable);
void hardware_set_7v8(bool enable);
void hardware_servo_lock(void);
void hardware_servo_unlock(void);