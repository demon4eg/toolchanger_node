#pragma once

#include <stdint.h>
#include <stdbool.h>

// Tool command callback type
typedef void (*tool_command_handler_t)(uint8_t command, uint16_t tool_id, uint16_t param);

// Tool registration structure
typedef struct {
    uint16_t command_min;
    uint16_t command_max;
    tool_command_handler_t handler;
    const char* name;
} tool_registration_t;

void tool_manager_init(void);
void tool_manager_register_tool(tool_registration_t* tool);
void tool_manager_process_command(uint8_t command, uint16_t tool_id);