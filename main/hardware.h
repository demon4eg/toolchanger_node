#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>

void hardware_init(void);
void hardware_update(void);  // Call this periodically

uint8_t hardware_get_tool_id(void);
float hardware_get_temperature(void);
bool hardware_is_tool_present(void);

#endif