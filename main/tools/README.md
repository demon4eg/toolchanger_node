Future Tool Example - Adding a Dispenser
Simply create tool_dispenser.c:

c
#include "tool_manager.h"

#define DISPENSER_CMD_START  200
#define DISPENSER_CMD_STOP   201
#define DISPENSER_CMD_AMOUNT 202

void tool_dispenser_process_command(uint8_t command, uint16_t tool_id, uint16_t param)
{
    switch(command) {
        case DISPENSER_CMD_START:
            // Start dispensing
            break;
        case DISPENSER_CMD_STOP:
            // Stop dispensing
            break;
    }
}

void tool_dispenser_init(void)
{
    static tool_registration_t dispenser_tool = {
        .command_min = 200,
        .command_max = 210,
        .handler = tool_dispenser_process_command,
        .name = "dispenser"
    };
    tool_manager_register_tool(&dispenser_tool);
}
Then in ros_manager_init():

c
tool_dispenser_init();  // Just add this line



Summary
This architecture allows:

Each tool is independent in its own file

Tools register their command ranges automatically

Adding a new tool is just creating one file + one init call

Hardware layer stays clean and reusable

No modifications needed to existing tool changer code


Benefits
Simple table - Just add new entries for new tools

Auto-detection - ESP32 knows what tool is attached by its DS18B20 ID

Command routing - Commands automatically go to correct tool handler

Tool ID check - Rejects commands for wrong tool

Easy to add - New tool = add table entry + create handler

Adding a new tool example
c
// In known_tools table:
{ 4321, TOOL_TYPE_DISPENSER, "Dispenser", 400, 500 },

// Create tool_dispenser.c with registration:
static tool_registration_t dispenser_tool = {
    .type = TOOL_TYPE_DISPENSER,
    .handler = tool_dispenser_process_command,
    .name = "dispenser",
    .cmd_min = 400,
    .cmd_max = 500
};