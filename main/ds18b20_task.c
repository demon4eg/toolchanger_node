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

#if CONFIG_EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP
#define EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP 1
#else
#define EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP 0
#endif

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
    if (!bus) return;
    
    // If no devices - search (EXACTLY as your example)
    if (ds18b20_device_num == 0) {
        onewire_device_iter_handle_t iter = NULL;
        onewire_device_t next_onewire_device;
        onewire_new_device_iter(bus, &iter);
        
        while (onewire_device_iter_get_next(iter, &next_onewire_device) == ESP_OK) {
            ds18b20_config_t ds_cfg = {};
            
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
                onewire_device_address_t address;
                ds18b20_get_device_address(ds18b20s[ds18b20_device_num], &address);
                tool_address = address;
                ESP_LOGI(TAG, "Tool Found! ID: %016llX", address);
                ds18b20_device_num++;
                if (ds18b20_device_num >= 2) break;
            }
        }
        onewire_del_device_iter(iter);
    }

    // If device found - read temperature (EXACTLY as your example)
    if (ds18b20_device_num > 0) {
        esp_err_t err = ds18b20_trigger_temperature_conversion_for_all(bus);
        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            float temperature;
            for (int i = 0; i < ds18b20_device_num; i++) {
                if (ds18b20_get_temperature(ds18b20s[i], &temperature) == ESP_OK) {
                    current_temp = temperature;
                    ESP_LOGI(TAG, "Tool [%d] Temp: %.2f C", i, temperature);
                }
            }
        } else {
            ESP_LOGW(TAG, "Tool disconnected. Cleaning up...");
            for (int i = 0; i < ds18b20_device_num; i++) {
                ds18b20_del_device(ds18b20s[i]);
            }
            ds18b20_device_num = 0;
            tool_address = 0;
            current_temp = -273.0f;
        }
    } else {
        ESP_LOGI(TAG, "Waiting for tool connection...");
    }
}

bool ds18b20_is_present(void)
{
    return ds18b20_device_num > 0;
}

uint8_t ds18b20_get_id(void)
{
    return (tool_address >> 8) & 0xFF;
}

float ds18b20_get_temp(void)
{
    return current_temp;
}