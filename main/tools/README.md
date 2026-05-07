Tool Changer Controller - Adding Tools & Command Reference
Adding a New Tool
1. Define Tool ID in tool_manager.h
Add your tool's DS18B20 ID to the known tools table (in tool_manager.c):

c
static tool_info_t known_tools[] = {
    { 2088, TOOL_TYPE_GRIPPER, "Gripper", 5, 36 },
    { 1234, TOOL_TYPE_DISPENSER, "Dispenser", 37, 52 },  // Add yours
    // Add more tools here
};
2. Add Tool Type Enum
In tool_manager.h:

c
typedef enum {
    TOOL_TYPE_NONE = 0,
    TOOL_TYPE_GRIPPER = 1,
    TOOL_TYPE_DISPENSER = 2,
    TOOL_TYPE_VACUUM = 3,
    TOOL_TYPE_CUSTOM = 4,
    // Add your tool type here
} tool_type_t;
3. Create Tool Files
Create tools/tool_yourtool.c and tools/tool_yourtool.h:

c
// tool_yourtool.h
#pragma once
#include <stdint.h>

void tool_yourtool_init(void);
void tool_yourtool_process_command(uint8_t command, uint16_t tool_id, uint16_t param);
c
// tool_yourtool.c
#include "tool_yourtool.h"
#include "tool_manager.h"
#include "hardware.h"
#include "esp_log.h"

#define TAG "TOOL_YOURTOOL"

// Define your command ranges (from tool_manager.h)
// Example: 37-42 for speed, 43-52 for volume

void tool_yourtool_process_command(uint8_t command, uint16_t tool_id, uint16_t param)
{
    ESP_LOGI(TAG, "Command: %d", command);
    
    if (command >= 37 && command <= 42) {
        // Speed control logic
        uint8_t speed = (command - 37) * 20;  // 0-100%
        // Your hardware control here
    }
    else if (command >= 43 && command <= 52) {
        // Volume control logic
        uint8_t volume = (command - 43) * 10;  // 0-100%
        // Your hardware control here
    }
}

void tool_yourtool_init(void)
{
    // Initialize hardware (PWM, GPIO, etc.)
    hardware_pwm_init(...);
    
    // Register with tool manager
    static tool_registration_t yourtool = {
        .type = TOOL_TYPE_YOURTOOL,
        .handler = tool_yourtool_process_command,
        .name = "yourtool",
        .cmd_min = 37,
        .cmd_max = 52
    };
    tool_manager_register_tool(&yourtool);
}
4. Initialize Tool on Detection
In state_machine.c, add your tool initialization:

c
tool_type_t type = tool_manager_get_tool_type_from_id(tool_id);

if (type == TOOL_TYPE_GRIPPER) {
    extern void tool_gripper_init(void);
    tool_gripper_init();
}
else if (type == TOOL_TYPE_DISPENSER) {
    extern void tool_dispenser_init(void);
    tool_dispenser_init();
}
// Add your tool here
Complete Command Reference
Global Commands (0-3)
Command	Function	Description
0	UNLOCK	Unlock changer, wait for tool insertion/removal
1	LOCK	Force lock mechanism (with or without tool)
2	7V8_ON	Enable 7.8V rail for tool power
3	7V8_OFF	Disable 7.8V rail
Gripper Commands (5-36)
Command	Function	Value Range
5-25	Position	5=closed(0%), 15=50%, 25=open(100%)
26-36	Effort/Force	26=min(0mA), 31=50%, 36=max(1200mA)
Examples:

bash
# Gripper half open
ros2 topic pub /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 15, tool_id: 0}"

# Gripper max effort
ros2 topic pub /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 36, tool_id: 0}"
Dispenser Commands (37-52)
Command	Function	Value Range
37-42	Speed	37=stop, 39=medium, 42=max
43-52	Volume	43=0%, 47=50%, 52=100%
Examples:

bash
# Dispense at medium speed
ros2 topic pub /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 39, tool_id: 0}"

# Dispense 70% volume
ros2 topic pub /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 50, tool_id: 0}"
Vacuum/Pneumatic Commands (60-71)
Command	Function	Value Range
60	On/Off	60=off, 61=on
62-71	Suction Strength	62=min, 66=50%, 71=max
INA3221 Power Monitor Commands (100-110)
Command	Function	Return Value (in tool_id field)
100	Get CH1 Current (mA)	0-65535
101	Get CH2 Current (mA)	0-65535
102	Get CH3 Current (mA)	0-65535
103	Get CH1 Voltage (mV)	0-65535
104	Get CH2 Voltage (mV)	0-65535
105	Get CH3 Voltage (mV)	0-65535
106	Get CH1 Power (mW)	0-65535
107	Get CH2 Power (mW)	0-65535
108	Get CH3 Power (mW)	0-65535
Examples:

bash
# Get gripper current
ros2 topic pub --once /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 100, tool_id: 0}"
# Response: tool_id = current_in_mA

# Get 7.8V rail voltage
ros2 topic pub --once /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 105, tool_id: 0}"
# Response: tool_id = voltage_in_mV
Reserved Commands (80-99, 111-200)
For future expansion

ROS Message Format
Command Topic: /toolchanger/command
yaml
uint8 command   # Command number (see tables above)
uint16 tool_id  # Tool identifier (0 = ignore, otherwise tool specific)
Status Topic: /toolchanger/status
yaml
uint8 command     # Echo of last received command
uint16 tool_id    # Current attached tool ID (0 if none)
uint8 state       # 0=IDLE, 2=ATTACHED, 3=UNLOCKING, 4=ERROR
uint8 error_code  # 0=ok, 1=timeout, 2=unexpected disconnect, 4=warning
float32 temperature  # Tool temperature (DS18B20) or -273 if none
Typical Usage Sequence
1. Attach Tool
bash
# Send UNLOCK command (wait for tool insertion)
ros2 topic pub --once /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 0, tool_id: 0}"

# Insert tool physically within 5 seconds
# ESP32 auto-locks and powers 7.8V rail
2. Control Tool
bash
# Control gripper
ros2 topic pub /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 25, tool_id: 0}"

# Monitor current
ros2 topic pub --once /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 100, tool_id: 0}"
3. Release Tool
bash
# Send RELEASE command
ros2 topic pub --once /toolchanger/command manipulator_6dof_interfaces/msg/ToolChanger "{command: 0, tool_id: 0}"

# Remove tool within 5 seconds
# ESP32 auto-locks and powers off 7.8V rail
Adding Custom Commands
Step 1: Reserve command range
Add to tool_manager.h:

c
#define TOOL_CMD_MYTOOL_MIN  200
#define TOOL_CMD_MYTOOL_MAX  210
Step 2: Implement handler
c
void tool_mytool_process_command(uint8_t command, uint16_t tool_id, uint16_t param)
{
    switch(command) {
        case 200: /* action 1 */ break;
        case 201: /* action 2 */ break;
        // ...
    }
}
Step 3: Register
c
static tool_registration_t mytool = {
    .type = TOOL_TYPE_CUSTOM,
    .handler = tool_mytool_process_command,
    .name = "mytool",
    .cmd_min = 200,
    .cmd_max = 210
};
Notes
tool_id=0 in commands means "apply to currently attached tool"

Non-zero tool_id can be used for multi-tool systems (future)

All commands from 4-255 are routed to tools

Commands 0-3 are reserved for tool changer control

Status publishes automatically at 10Hz

Timeout for tool insertion/removal is 5 seconds