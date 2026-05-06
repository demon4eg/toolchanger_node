#include "state_machine.h"
#include "ros_manager.h"
#include "hardware.h"
#include "ds18b20_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define TAG "STATE_MACHINE"

// Commands
#define CMD_UNLOCK      0
#define CMD_LOCK        1
#define CMD_7V8_ON      2
#define CMD_7V8_OFF     3

// Timeouts
#define UNLOCK_TIMEOUT_MS  5000U  // 5 seconds (unsigned)

// States
typedef enum {
    IDLE = 0,      // No tool, servo locked
    ATTACHED = 2,  // Tool present, servo locked, 7.8V ON
    UNLOCKING = 3  // Servo unlocked, waiting for tool insertion/removal
} state_t;

static state_t current_state = IDLE;
static uint32_t unlock_start_time = 0;
static bool waiting_for_tool = false;      // Flag for waiting during attach
static bool warning_published = false;

// External variables from ros_manager.c
extern uint8_t ros_last_command;
extern uint16_t ros_last_tool_id;
extern bool ros_command_received;

static const char* state_names[] = {"IDLE", "ATTACHING", "ATTACHED", "RELEASING", "ERROR"};

static void set_servo_locked(bool locked)
{
    // 5V is already ON when entering this function (caller ensures)
    if (locked) {
        hardware_servo_lock();
        ESP_LOGI(TAG, "Servo → LOCKED");
    } else {
        hardware_servo_unlock();
        ESP_LOGI(TAG, "Servo → UNLOCKED");
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Let servo move
    // Note: 5V stays ON after movement (will be turned off by caller if needed)
}

static void enter_state(state_t new_state)
{
    ESP_LOGI(TAG, "%s → %s", state_names[current_state], state_names[new_state]);
    current_state = new_state;
    
    if (new_state == IDLE) {
        warning_published = false;
    }
}

static void publish_warning(const char* warning)
{
    if (!warning_published) {
        ESP_LOGW(TAG, "WARNING: %s", warning);
        warning_published = true;
    }
}

static void publish_current_status(void)
{
    uint16_t tool_id = ds18b20_is_present() ? ds18b20_get_id() : 0;
    float temp = ds18b20_is_present() ? ds18b20_get_temp() : -273.0f;
    uint8_t state_code = 0, error_code = 0;
    
    switch(current_state) {
        case IDLE:       state_code = 0; break;
        case ATTACHED:   state_code = 2; break;
        case UNLOCKING:  state_code = 3; break;
        default:         state_code = 0; break;
    }
    
    // Set error_code for warnings
    if (warning_published) {
        error_code = 4;  // Generic warning
    }
    
    ros_publish_status(tool_id, state_code, error_code, temp);
}

void state_machine_button_unlock(void)
{
    if (current_state == IDLE && !waiting_for_tool) {
        ESP_LOGI(TAG, "Button pressed: Unlocking, waiting for tool");
        hardware_set_7v8(false);
        hardware_set_5v(true);
        set_servo_locked(false);
        waiting_for_tool = true;
        unlock_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        enter_state(UNLOCKING);
    } else {
        ESP_LOGW(TAG, "Button ignored: wrong state (state=%d, waiting=%d)", 
                 current_state, waiting_for_tool);
    }
}

void state_machine_init(void)
{
    hardware_init();
    ds18b20_task_start();
    
    // Start with servo locked, all rails OFF
    hardware_set_5v(true);
    hardware_servo_lock();
    vTaskDelay(pdMS_TO_TICKS(10));
    hardware_set_5v(false);
    hardware_set_7v8(false);
    
    // DISABLE DETECTION FIRST - before any scans
    ds18b20_set_detection_enabled(false);
    
    // Wait for DS18B20 bus to stabilize
    ESP_LOGI(TAG, "Waiting for DS18B20 detection...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Force multiple scans (detection is disabled, so no auto-detection)
    for (int i = 0; i < 3; i++) {
        ds18b20_task_run();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Check if tool is present after scanning
    if (ds18b20_is_present()) {
        uint16_t tool_id = ds18b20_get_id();
        ESP_LOGI(TAG, "Tool already present at boot! ID=%d - powering up", tool_id);
        hardware_set_7v8(true);
        enter_state(ATTACHED);
    } else {
        ESP_LOGI(TAG, "No tool detected at boot");
        enter_state(IDLE);
    }
    
    ESP_LOGI(TAG, "State machine initialized");
}

void state_machine_task(void *arg)
{
    uint32_t now;
    bool tool_present;
    
    ESP_LOGI(TAG, "State machine task running");
    
    while(1) {
        ds18b20_task_run();
        now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        tool_present = ds18b20_is_present();
        
        // Handle ROS commands
        if (ros_command_received) {
            ros_command_received = false;
            
            switch(ros_last_command) {
                case CMD_UNLOCK:  // 0 - ATTACH/UNLOCK command
                    if (current_state == IDLE && !waiting_for_tool) {
                        ESP_LOGI(TAG, "ROS ATTACH command: Unlocking, waiting for tool");
                        hardware_set_7v8(false);
                        hardware_set_5v(true);
                        set_servo_locked(false);
                        ds18b20_set_detection_enabled(true);
                        waiting_for_tool = true;
                        unlock_start_time = now;
                        enter_state(UNLOCKING);
                    } else {
                        ESP_LOGW(TAG, "UNLOCK ignored: not IDLE or already waiting");
                    }
                    break;
                    
                case CMD_LOCK:  // 1 - Manual lock
                    ESP_LOGI(TAG, "ROS LOCK command");
                    hardware_set_5v(true);
                    set_servo_locked(true);
                    if (tool_present) {
                        hardware_set_7v8(true);
                        waiting_for_tool = false;
                        enter_state(ATTACHED);
                    } else {
                        hardware_set_5v(false);
                        hardware_set_7v8(false);
                        waiting_for_tool = false;
                        enter_state(IDLE);
                        publish_warning("LOCK command with no tool present");
                    }
                    break;
                    
                case CMD_7V8_ON:
                    ESP_LOGI(TAG, "ROS 7.8V ON");
                    hardware_set_7v8(true);
                    break;
                    
                case CMD_7V8_OFF:
                    ESP_LOGI(TAG, "ROS 7.8V OFF");
                    hardware_set_7v8(false);
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown ROS command: %d", ros_last_command);
                    break;
            }
        }
        
        // Waiting for tool insertion (UNLOCKING state)
        if (waiting_for_tool) {
            if (tool_present) {
                // Tool inserted! Lock it
                ESP_LOGI(TAG, "Tool detected while unlocked → Locking");
                hardware_set_5v(true);
                set_servo_locked(true);
                ds18b20_set_detection_enabled(false); 
                hardware_set_7v8(true);
                hardware_set_5v(false);  // 5V off (spring holds)
                waiting_for_tool = false;
                enter_state(ATTACHED);
            }
            else if ((now - unlock_start_time) >= UNLOCK_TIMEOUT_MS) {
                // Timeout - no tool inserted, relock
                ESP_LOGW(TAG, "Timeout waiting for tool (%u ms) - relocking", UNLOCK_TIMEOUT_MS);
                hardware_set_5v(true);
                set_servo_locked(true);
                ds18b20_set_detection_enabled(false); 
                hardware_set_5v(false);
                hardware_set_7v8(false);
                waiting_for_tool = false;
                enter_state(IDLE);
                publish_warning("No tool inserted within timeout");
            }
        }
        
        // Check for unexpected tool disappearance in ATTACHED
        if (current_state == ATTACHED && !tool_present) {
            ESP_LOGE(TAG, "Unexpected: Tool disappeared while ATTACHED");
            hardware_set_5v(false);
            hardware_set_7v8(false);
            set_servo_locked(true);
            waiting_for_tool = false;
            enter_state(IDLE);
            publish_warning("Tool unexpectedly disconnected");
        }
        
        publish_current_status();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}