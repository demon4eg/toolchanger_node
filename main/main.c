#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <uros_network_interfaces.h>
#include "ros_manager.h"
#include "state_machine.h"
#include "hardware.h"

#define TAG "MAIN"
#define PIN_BUTTON_UNLOCK 14

static void IRAM_ATTR button_isr(void *arg)
{
    // Defer processing to task to avoid ISR constraints
    state_machine_button_unlock();
}

void app_main(void)
{

    //esp_log_level_set("1-wire", ESP_LOG_ERROR);      // Only errors from 1-wire
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    esp_log_level_set("1-wire.device", ESP_LOG_ERROR);
    esp_log_level_set("ds18b20", ESP_LOG_WARN);     // Only warnings and errors
    esp_log_level_set("STATE_MACHINE", ESP_LOG_INFO);  // Keep state machine info
    esp_log_level_set("HARDWARE", ESP_LOG_INFO);       // Keep hardware info
    ESP_LOGI(TAG, "Starting...");
    
    nvs_flash_init();
    uros_network_interface_initialize();
    
    state_machine_init();
    ros_manager_init();
    
    // Configure button
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << PIN_BUTTON_UNLOCK),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Trigger on press (falling edge)
    };
    gpio_config(&btn_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUTTON_UNLOCK, button_isr, NULL);
    ESP_LOGI(TAG, "Button configured on GPIO%d", PIN_BUTTON_UNLOCK);
    
    xTaskCreate(state_machine_task, "state", 4096, NULL, 2, NULL);
    
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}