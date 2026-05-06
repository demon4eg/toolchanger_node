#include "state_machine.h"
#include "ros_manager.h"
#include "hardware.h"
#include "ds18b20_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "STATE_MACHINE"
#define TIMEOUT_ATTACH  10000
#define TIMEOUT_RELEASE 5000

// External variables from ros_manager.c
extern uint8_t ros_last_command;
extern uint16_t ros_last_tool_id;
extern bool ros_command_received;

typedef enum {
    IDLE, ATTACHING, LOCKING, ATTACHED, WRONG_ID, TIMEOUT, RELEASING, ERROR
} state_t;

static state_t current_state = IDLE;
static uint32_t state_start = 0;
static uint32_t error_time = 0;

static const char* state_names[] = {"IDLE","ATTACHING","LOCKING","ATTACHED","WRONG_ID","TIMEOUT","RELEASING","ERROR"};

static void enter_state(state_t new_state)
{
    ESP_LOGI(TAG, "%s → %s", state_names[current_state], state_names[new_state]);
    current_state = new_state;
    state_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void publish_current_status(void)
{
    uint16_t tool_id = ds18b20_is_present() ? ds18b20_get_id() : 0;
    float temp = ds18b20_is_present() ? ds18b20_get_temp() : -273.0f;
    uint8_t state_code = 0, error_code = 0;
    
    switch(current_state) {
        case IDLE: state_code = 0; break;
        case ATTACHING: state_code = 1; break;
        case LOCKING: state_code = 2; break;
        case ATTACHED: state_code = 2; break;
        case RELEASING: state_code = 3; break;
        case WRONG_ID: state_code = 4; error_code = 1; break;
        case TIMEOUT: state_code = 4; error_code = 2; break;
        case ERROR: state_code = 4; error_code = 3; break;
    }
    
    ros_publish_status(tool_id, state_code, error_code, temp);
}

void state_machine_init(void)
{
    hardware_init();
    ds18b20_task_start();
    enter_state(IDLE);
}

void state_machine_task(void *arg)
{
    uint32_t now, elapsed;
    
    while(1) {
        ds18b20_task_run();
        now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        elapsed = now - state_start;
        
        // Handle ROS commands
        if (ros_command_received) {
            ros_command_received = false;
            
            if (ros_last_command == 1) {  // ATTACH
                if (current_state == IDLE || current_state == TIMEOUT || 
                    current_state == WRONG_ID || current_state == ERROR) {
                    hardware_set_5v(true);
                    hardware_servo_unlock();
                    enter_state(ATTACHING);
                }
            } 
            else if (ros_last_command == 0) {  // RELEASE
                if (current_state == ATTACHED) {
                    enter_state(RELEASING);
                } else if (current_state == WRONG_ID || current_state == TIMEOUT || current_state == ERROR) {
                    hardware_set_5v(false);
                    hardware_set_7v8(false);
                    hardware_servo_lock();
                    enter_state(IDLE);
                }
            }
        }
        
        // State machine logic
        switch(current_state) {
            case ATTACHING:
                // Wait 500ms for DS18B20 to power up
                if (elapsed < 500) {
                    ESP_LOGI(TAG, "Powering up... %lu ms", elapsed);
                    break;
                }
                
                if (elapsed >= TIMEOUT_ATTACH) {
                    ESP_LOGW(TAG, "Timeout after %lu ms", elapsed);
                    hardware_set_5v(false);
                    enter_state(TIMEOUT);
                } else {
                    bool present = ds18b20_is_present();
                    uint16_t id = ds18b20_get_id();
                    ESP_LOGI(TAG, "Tool present=%d, ID=%d, Expected=%d, elapsed=%lu", 
                            present, id, ros_last_tool_id, elapsed);
                    
                    if (present && id == ros_last_tool_id) {
                        enter_state(LOCKING);
                    } else if (present && id != ros_last_tool_id) {
                        ESP_LOGW(TAG, "Wrong ID: got %d, expected %d", id, ros_last_tool_id);
                        enter_state(WRONG_ID);
                    }
                    // else continue waiting
                }
                break;
                
            case LOCKING:
                hardware_servo_lock();
                hardware_set_5v(false);
                hardware_set_7v8(true);
                enter_state(ATTACHED);
                break;
                
            case ATTACHED:
                if (!ds18b20_is_present()) {
                    hardware_set_5v(false);
                    hardware_set_7v8(false);
                    hardware_servo_lock();
                    enter_state(ERROR);
                    error_time = now;
                }
                break;
                
            case RELEASING:
                hardware_servo_unlock();
                if (!ds18b20_is_present()) {
                    hardware_set_5v(false);
                    hardware_set_7v8(false);
                    hardware_servo_lock();
                    enter_state(IDLE);
                } else if (elapsed >= TIMEOUT_RELEASE) {
                    hardware_set_5v(false);
                    hardware_set_7v8(false);
                    hardware_servo_lock();
                    enter_state(ERROR);
                    error_time = now;
                }
                break;
                
            case WRONG_ID:
                // Keep 5V on, wait for new command
                break;
                
            case ERROR:
                if (now - error_time >= 3000) {
                    enter_state(IDLE);
                }
                break;
                
            default:
                break;
        }
        
        publish_current_status();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}