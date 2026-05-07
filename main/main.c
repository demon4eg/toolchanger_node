#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <uros_network_interfaces.h>
#include "ros_manager.h"
#include "state_machine.h"
#include "hardware.h"
#include "led_status.h"

#define TAG "MAIN"

// Simple flag for ISR (volatile, no logging)
static volatile bool button_pressed = false;

static void IRAM_ATTR button_isr(void *arg)
{
    button_pressed = true;
}

void app_main(void)
{
    led_status_init();
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    esp_log_level_set("1-wire.device", ESP_LOG_ERROR);
    esp_log_level_set("ds18b20", ESP_LOG_NONE);
    esp_log_level_set("STATE_MACHINE", ESP_LOG_INFO);
    esp_log_level_set("HARDWARE", ESP_LOG_INFO);
    esp_log_level_set("LED_STATUS", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting...");
    
    nvs_flash_init();
    uros_network_interface_initialize();
    
    ros_manager_init();
    state_machine_init();
    
    
    
    
    // Configure button
    hardware_button_init();
    
    // Install ISR service and add handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUTTON_UNLOCK, button_isr, NULL);
    
    xTaskCreate(state_machine_task, "state", 4096, NULL, 2, NULL);
    
    while(1) {
        // Process button press from task context (safe for logging)
        if (button_pressed) {
            button_pressed = false;
            ESP_LOGI(TAG, "Button pressed");
            state_machine_button_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}