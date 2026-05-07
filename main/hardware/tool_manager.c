#include "tool_manager.h"
#include "esp_log.h"
#include "ds18b20_task.h"
#include <stddef.h>

#define MAX_TOOLS 5
#define MAX_KNOWN_TOOLS 10

#define TAG "TOOL_MANAGER"

static tool_registration_t* registered_tools[MAX_TOOLS];
static int tool_count = 0;

// Known tools lookup table (ID -> Tool Type)
// Format: { DS18B20_ID, tool_type, "name", cmd_min, cmd_max }
static tool_info_t known_tools[] = {
    { 2088, TOOL_TYPE_GRIPPER, "Gripper", 0, 300 },      // Your gripper
    // Add more tools as you develop them
    // { 1234, TOOL_TYPE_DISPENSER, "Dispenser", 400, 500 },
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

void tool_manager_register_tool(tool_registration_t* tool)
{
    if (tool_count < MAX_TOOLS) {
        registered_tools[tool_count++] = tool;
        ESP_LOGI(TAG, "Registered tool handler for '%s' (type=%d, cmds=%d-%d)", 
                 tool->name, tool->type, tool->cmd_min, tool->cmd_max);
    } else {
        ESP_LOGE(TAG, "Cannot register tool - max reached");
    }
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

// Get tool registration by type
static tool_registration_t* get_tool_registration(tool_type_t type)
{
    for (int i = 0; i < tool_count; i++) {
        if (registered_tools[i]->type == type) {
            return registered_tools[i];
        }
    }
    return NULL;
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
    (void)tool_id;  // Ignore incoming tool_id - always use current tool
    
    // If no tool attached, reject all commands
    if (current_tool_type == TOOL_TYPE_NONE) {
        ESP_LOGW(TAG, "No tool attached, ignoring command %d", command);
        return;
    }
    
    // Get handler for current tool type
    tool_registration_t* tool = get_tool_registration(current_tool_type);
    if (tool == NULL) {
        ESP_LOGW(TAG, "No handler registered for tool type %d", current_tool_type);
        return;
    }
    
    // Check command range
    if (command >= tool->cmd_min && command <= tool->cmd_max) {
        tool->handler(command, current_tool_id, 0);
    } else {
        ESP_LOGW(TAG, "Command %d out of range for tool '%s' (%d-%d)", 
                 command, tool->name, tool->cmd_min, tool->cmd_max);
    }
}