#include "tool_manager.h"
#include "esp_log.h"

#define MAX_TOOLS 5

#define TAG "TOOL_MANAGER"

static tool_registration_t* registered_tools[MAX_TOOLS];
static int tool_count = 0;

void tool_manager_init(void)
{
    ESP_LOGI(TAG, "Tool manager initialized");
}

void tool_manager_register_tool(tool_registration_t* tool)
{
    if (tool_count < MAX_TOOLS) {
        registered_tools[tool_count++] = tool;
        ESP_LOGI(TAG, "Registered tool '%s' for commands %d-%d", 
                 tool->name, tool->command_min, tool->command_max);
    } else {
        ESP_LOGE(TAG, "Cannot register tool '%s' - max tools reached", tool->name);
    }
}

void tool_manager_process_command(uint8_t command, uint16_t tool_id)
{
    for (int i = 0; i < tool_count; i++) {
        tool_registration_t* tool = registered_tools[i];
        if (command >= tool->command_min && command <= tool->command_max) {
            ESP_LOGD(TAG, "Routing command %d to tool '%s'", command, tool->name);
            tool->handler(command, tool_id, 0);
            return;
        }
    }
    ESP_LOGW(TAG, "No tool registered for command %d", command);
}