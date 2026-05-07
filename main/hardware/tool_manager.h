#pragma once

#include <stdint.h>
#include <stdbool.h>

// Tool types
typedef enum {
    TOOL_TYPE_NONE = 0,
    TOOL_TYPE_GRIPPER = 1,
    TOOL_TYPE_DISPENSER = 2,
    TOOL_TYPE_VACUUM = 3,
    TOOL_TYPE_UART = 4,
    TOOL_TYPE_CUSTOM = 5
} tool_type_t;

// Tool info structure
typedef struct {
    uint16_t tool_id;           // DS18B20 ID (e.g., 2088)
    tool_type_t type;           // Tool type enum
    const char* name;           // Human readable name
    uint16_t cmd_min;           // Minimum command supported
    uint16_t cmd_max;           // Maximum command supported
} tool_info_t;

// Tool command callback
typedef void (*tool_command_handler_t)(uint8_t command, uint16_t tool_id, uint16_t param);

// Registration structure
typedef struct {
    tool_type_t type;
    tool_command_handler_t handler;
    const char* name;
    uint16_t cmd_min;
    uint16_t cmd_max;
} tool_registration_t;

void tool_manager_init(void);
void tool_manager_register_tool(tool_registration_t* tool);
void tool_manager_process_command(uint8_t command, uint16_t tool_id);
tool_type_t tool_manager_get_current_tool_type(void);
uint16_t tool_manager_get_current_tool_id(void);
void tool_manager_set_current_tool(uint16_t tool_id);
tool_type_t tool_manager_get_tool_type_from_id(uint16_t tool_id);