#pragma once

#include <stdint.h>
#include <stdbool.h>

// Tool command ranges (global)
#define TOOL_CMD_GRIPPER_POS_MIN     5
#define TOOL_CMD_GRIPPER_POS_MAX     25
#define TOOL_CMD_GRIPPER_EFFORT_MIN  26
#define TOOL_CMD_GRIPPER_EFFORT_MAX  36

#define TOOL_CMD_DISPENSER_SPEED_MIN 37
#define TOOL_CMD_DISPENSER_SPEED_MAX 42
#define TOOL_CMD_DISPENSER_VOL_MIN   43
#define TOOL_CMD_DISPENSER_VOL_MAX   52

#define TOOL_CMD_VACUUM_ONOFF        60
#define TOOL_CMD_VACUUM_STR_MIN      62
#define TOOL_CMD_VACUUM_STR_MAX      71

// New: Stepper tool
#define TOOL_CMD_STEPPER_POS_MIN     70
#define TOOL_CMD_STEPPER_POS_MAX     100
#define TOOL_CMD_STEPPER_SPEED_MIN   101
#define TOOL_CMD_STEPPER_SPEED_MAX   120
#define TOOL_CMD_STEPPER_HOME        121
#define TOOL_CMD_STEPPER_STOP        122

// New: PWM tool
#define TOOL_CMD_PWM_VEL_MIN         130
#define TOOL_CMD_PWM_VEL_MAX         150
#define TOOL_CMD_PWM_EFFORT_MIN      151
#define TOOL_CMD_PWM_EFFORT_MAX      170
#define TOOL_CMD_PWM_ON              171
#define TOOL_CMD_PWM_OFF             172

#define TOOL_CMD_RESERVED_MIN        80
#define TOOL_CMD_RESERVED_MAX        99


// Tool types
typedef enum {
    TOOL_TYPE_NONE = 0,
    TOOL_TYPE_GRIPPER = 1,
    TOOL_TYPE_STEPPER = 2,
    TOOL_TYPE_PWM = 3,
    TOOL_TYPE_DISPENSER = 4,
    TOOL_TYPE_VACUUM = 5,
    TOOL_TYPE_UART = 6,
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

void tool_manager_init(void);
void tool_manager_process_command(uint8_t command, uint16_t tool_id);
tool_type_t tool_manager_get_current_tool_type(void);
uint16_t tool_manager_get_current_tool_id(void);
void tool_manager_set_current_tool(uint16_t tool_id);
tool_type_t tool_manager_get_tool_type_from_id(uint16_t tool_id);