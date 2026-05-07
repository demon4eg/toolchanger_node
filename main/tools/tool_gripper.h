#pragma once

#include <stdint.h>

void tool_gripper_init(void);
void tool_gripper_process_command(uint8_t command, uint16_t tool_id, uint16_t param);