#include "hardware.h"
#include "ds18b20.h"
#include "esp_log.h"

#define TAG "HARDWARE"

void hardware_init(void)
{
    ds18b20_init();
    ESP_LOGI(TAG, "Hardware initialized");
}

void hardware_update(void)
{
    ds18b20_update();
}

uint8_t hardware_get_tool_id(void)
{
    return ds18b20_get_tool_id();
}

float hardware_get_temperature(void)
{
    return ds18b20_get_temperature();
}

bool hardware_is_tool_present(void)
{
    return ds18b20_is_present();
}