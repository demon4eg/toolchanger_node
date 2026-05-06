#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <uros_network_interfaces.h>
#include "ros_manager.h"
#include "state_machine.h"

#define TAG "MAIN"

void app_main(void)
{
    ESP_LOGI(TAG, "Starting...");
    
    nvs_flash_init();
    uros_network_interface_initialize();
    
    state_machine_init();
    ros_manager_init();
    
    xTaskCreate(state_machine_task, "state", 4096, NULL, 2, NULL);
    
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}