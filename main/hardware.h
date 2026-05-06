#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdbool.h>

void hardware_init(void);
void hardware_set_5v(bool enable);
void hardware_set_7v8(bool enable);
void hardware_servo_lock(void);
void hardware_servo_unlock(void);

#endif