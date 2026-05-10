#pragma once

#include <stdint.h>

void tool_stepper_init(void);
void tool_stepper_process_command(uint8_t command, uint16_t tool_id, uint16_t param);

// Feedback functions
int32_t stepper_get_current_position(void);
int32_t stepper_get_current_speed(void);
uint8_t stepper_get_status(void);