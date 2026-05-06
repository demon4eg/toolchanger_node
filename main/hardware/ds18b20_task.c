#include "ds18b20_task.h"
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "onewire_bus.h"
#include "ds18b20.h"

#define TAG "DS18B20"

static onewire_bus_handle_t bus = NULL;
static ds18b20_device_handle_t ds18b20s[2];
static int ds18b20_device_num = 0;
static uint64_t tool_address = 0;
static float current_temp = -273.0f;
static bool scan_in_progress = false;
static bool detection_enabled = true;

#if CONFIG_EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP
#define EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP 1
#else
#define EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP 0
#endif

void ds18b20_set_detection_enabled(bool enabled)
{
    detection_enabled = enabled;
    ESP_LOGI(TAG, "Tool detection %s", enabled ? "ENABLED" : "DISABLED");
}

bool ds18b20_is_detection_enabled(void)
{
    return detection_enabled;
}

void ds18b20_task_start(void)
{
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = CONFIG_EXAMPLE_ONEWIRE_BUS_GPIO,
        .flags.en_pull_up = EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP,
    };

    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", CONFIG_EXAMPLE_ONEWIRE_BUS_GPIO);
}

void ds18b20_task_run(void)
{
    if (!bus) {
        ESP_LOGE(TAG, "Bus not initialized!");
        return;
    }

    static bool first_run = true;
    if (first_run) {
        ESP_LOGI(TAG, "First DS18B20 scan starting...");
        first_run = false;
    }
    
    // If device exists, just read temperature (no scan spam)
    if (ds18b20_device_num > 0) {
        esp_err_t err = ds18b20_trigger_temperature_conversion_for_all(bus);
        
        if (err != ESP_OK) {
            // Device disconnected - only clear if detection is enabled
            if (detection_enabled) {
                ESP_LOGW(TAG, "Tool disconnected, clearing...");
                for (int i = 0; i < ds18b20_device_num; i++) {
                    ds18b20_del_device(ds18b20s[i]);
                }
                ds18b20_device_num = 0;
                tool_address = 0;
                current_temp = -273.0f;
            } else {
                ESP_LOGD(TAG, "Device read error but detection disabled - ignoring");
            }
        } else {
            // Wait for conversion (750ms)
            vTaskDelay(pdMS_TO_TICKS(750));
            float temperature;
            if (ds18b20_get_temperature(ds18b20s[0], &temperature) == ESP_OK) {
                current_temp = temperature;
            }
        }
        return;  // Skip scanning if we have a device
    }
    
    // No devices - scan for new tool ONLY if detection is enabled
    if (!detection_enabled) {
        return;
    }
    
    // Skip scan if one is already in progress
    if (scan_in_progress) {
        return;
    }
    
    scan_in_progress = true;
    
    ESP_LOGD(TAG, "Scanning for DS18B20...");
    
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t iter_err = onewire_new_device_iter(bus, &iter);
    
    if (iter_err != ESP_OK) {
        ESP_LOGD(TAG, "No devices found (iter error: %d)", iter_err);
        scan_in_progress = false;
        return;
    }
    
    bool found = false;
    while (onewire_device_iter_get_next(iter, &next_onewire_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        
        if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
            onewire_device_address_t address;
            ds18b20_get_device_address(ds18b20s[ds18b20_device_num], &address);
            tool_address = address;
            ESP_LOGI(TAG, "Tool Found! ID: %016llX", address);
            ds18b20_device_num++;
            found = true;
            if (ds18b20_device_num >= 2) break;
        }
    }
    onewire_del_device_iter(iter);
    
    if (!found) {
        ESP_LOGD(TAG, "No DS18B20 devices found");
    }
    
    scan_in_progress = false;
}

bool ds18b20_is_present(void)
{
    return ds18b20_device_num > 0;
}

uint16_t ds18b20_get_id(void)
{
    // Extract last 2 bytes (bytes 1 and 0) of the 64-bit address
    uint8_t byte0 = tool_address & 0xFF;           // 0x28
    uint8_t byte1 = (tool_address >> 8) & 0xFF;    // 0x08
    return (byte1 << 8) | byte0;  // Returns 0x0828 = 2088
}

float ds18b20_get_temp(void)
{
    return current_temp;
}