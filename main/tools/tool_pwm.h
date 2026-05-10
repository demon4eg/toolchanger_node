#pragma once

#include <stdint.h>

void tool_pwm_init(void);
void tool_pwm_process_command(uint8_t command, uint16_t tool_id, uint16_t param);

// Feedback functions
int32_t pwm_get_current_velocity(void);
int32_t pwm_get_current_effort(void);
uint8_t pwm_get_status(void);