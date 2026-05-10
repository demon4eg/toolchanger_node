#include "tool_manager.h"
#include "esp_log.h"
#include "ds18b20_task.h"
#include <stddef.h>
#include "tool_gripper.h"
#include "tool_stepper.h"
#include "tool_pwm.h"

#define MAX_TOOLS 5
#define MAX_KNOWN_TOOLS 10

#define TAG "TOOL_MANAGER"

// Known tools lookup table (ID -> Tool Type)
// Format: { DS18B20_ID, tool_type, "name", cmd_min, cmd_max }
static tool_info_t known_tools[] = {
    { 65320, TOOL_TYPE_GRIPPER, "Gripper", 0, 300 },      // Your gripper
    // Add more tools as you develop them
    { 63528, TOOL_TYPE_STEPPER, "Stepper", 400, 500 },
    { 2088, TOOL_TYPE_PWM, "PWM Tool", 171, 172 },
    // { 5678, TOOL_TYPE_VACUUM, "Vacuum", 600, 700 },
    // { 9999, TOOL_TYPE_UART, "UART Tool", 800, 900 },
};
static int known_tools_count = sizeof(known_tools) / sizeof(known_tools[0]);

static uint16_t current_tool_id = 0;
static tool_type_t current_tool_type = TOOL_TYPE_NONE;

void tool_manager_init(void)
{
    ESP_LOGI(TAG, "Tool manager initialized with %d known tools", known_tools_count);
    for (int i = 0; i < known_tools_count; i++) {
        ESP_LOGI(TAG, "  Known: ID=%u -> %s", known_tools[i].tool_id, known_tools[i].name);
    }
}

tool_type_t tool_manager_get_tool_type_from_id(uint16_t tool_id)
{
    for (int i = 0; i < known_tools_count; i++) {
        if (known_tools[i].tool_id == tool_id) {
            return known_tools[i].type;
        }
    }
    return TOOL_TYPE_NONE;
}


// Get tool type from known tools table
static tool_type_t get_tool_type_from_id(uint16_t tool_id)
{
    for (int i = 0; i < known_tools_count; i++) {
        if (known_tools[i].tool_id == tool_id) {
            return known_tools[i].type;
        }
    }
    return TOOL_TYPE_NONE;
}


// Call this when tool is attached/detected
void tool_manager_set_current_tool(uint16_t tool_id)
{
    current_tool_id = tool_id;
    current_tool_type = get_tool_type_from_id(tool_id);
    
    if (current_tool_type != TOOL_TYPE_NONE) {
        ESP_LOGI(TAG, "Tool attached: ID=%u, Type=%d", tool_id, current_tool_type);
    } else {
        ESP_LOGW(TAG, "Unknown tool attached: ID=%u", tool_id);
    }
}

tool_type_t tool_manager_get_current_tool_type(void)
{
    return current_tool_type;
}

uint16_t tool_manager_get_current_tool_id(void)
{
    return current_tool_id;
}

void tool_manager_process_command(uint8_t command, uint16_t tool_id)
{
    (void)tool_id;
    
    // Get current tool type from state machine
    tool_type_t type = tool_manager_get_current_tool_type();
    
    if (type == TOOL_TYPE_NONE) {
        ESP_LOGW(TAG, "No tool attached");
        return;
    }
    
    // Direct dispatch based on tool type
    switch (type) {
        case TOOL_TYPE_GRIPPER:
            tool_gripper_process_command(command, 0, 0);
            break;
        case TOOL_TYPE_DISPENSER:
            // tool_dispenser_process_command(command, 0, 0);
            break;
            case TOOL_TYPE_STEPPER:
            tool_stepper_process_command(command, 0, 0);
            break;
        case TOOL_TYPE_PWM:
            tool_pwm_process_command(command, 0, 0);
            break;
        default:
            ESP_LOGW(TAG, "Unknown tool type: %d", type);
            break;
    }
}