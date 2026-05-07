#include "state_machine.h"
#include "ros_manager.h"
#include "hardware.h"
#include "ds18b20_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"

#define TAG "STATE_MACHINE"

// Commands
#define CMD_UNLOCK      0
#define CMD_LOCK        1
#define CMD_7V8_ON      2
#define CMD_7V8_OFF     3

// Timeouts
#define UNLOCK_TIMEOUT_MS  5000U  // 5 seconds

// States
typedef enum {
    IDLE = 0,      // No tool, servo locked
    ATTACHED = 2,  // Tool present, servo locked, 7.8V ON
    UNLOCKING = 3  // Servo unlocked, waiting for tool action
} state_t;

static state_t current_state = IDLE;
static uint32_t unlock_start_time = 0;
static bool waiting_for_tool = false;      // Flag for waiting during attach/release
static bool waiting_for_insertion = true;  // true=waiting to insert, false=waiting to remove
static bool warning_published = false;
static uint16_t cached_tool_id = 0;

// External variables from ros_manager.c
extern uint8_t ros_last_command;
extern uint16_t ros_last_tool_id;
extern bool ros_command_received;
extern portMUX_TYPE ros_spinlock;

static const char* state_names[] = {"IDLE", "ATTACHING", "ATTACHED", "UNLOCKING", "ERROR"};

static void set_servo_locked(bool locked)
{
    if (locked) {
        hardware_servo_lock();
        ESP_LOGI(TAG, "Servo → LOCKED");
    } else {
        hardware_servo_unlock();
        ESP_LOGI(TAG, "Servo → UNLOCKED");
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Let servo move
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
    uint16_t tool_id = cached_tool_id;
    float temp = (cached_tool_id != 0) ? ds18b20_get_temp() : -273.0f;
    
    uint8_t state_code = 0, error_code = 0;
    
    switch(current_state) {
        case IDLE:       state_code = 0; break;
        case ATTACHED:   state_code = 2; break;
        case UNLOCKING:  state_code = 3; break;
        default:         state_code = 0; break;
    }
    
    if (warning_published) {
        error_code = 4;
    }
    
    // ESP_LOGI(TAG, "PUBLISHING: tool_id=%d, state=%d, error=%d, temp=%.2f", 
    //          tool_id, state_code, error_code, temp);
    
    ros_publish_status(tool_id, state_code, error_code, temp);
}
void state_machine_button_unlock(void)
{
    if (current_state == IDLE && !waiting_for_tool) {
        // ATTACH sequence: unlock to insert tool
        ESP_LOGI(TAG, "Button: Unlocking, waiting for tool insertion");
        hardware_set_7v8(false);
        hardware_set_5v(true);
        set_servo_locked(false);
        ds18b20_set_detection_enabled(true);
        waiting_for_tool = true;
        waiting_for_insertion = true;
        unlock_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        enter_state(UNLOCKING);
    } 
    else if (current_state == ATTACHED && !waiting_for_tool) {
        // RELEASE sequence: unlock to remove tool
        ESP_LOGI(TAG, "Button: Unlocking, waiting for tool removal");
        hardware_set_7v8(false);
        hardware_set_5v(true);
        set_servo_locked(false);
        ds18b20_set_detection_enabled(true);
        waiting_for_tool = true;
        waiting_for_insertion = false;
        unlock_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        enter_state(UNLOCKING);
    } 
    else {
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
    
    // TEMPORARILY ENABLE detection for boot scan
    ds18b20_set_detection_enabled(true);
    
    // Perform initial tool scan
    ESP_LOGI(TAG, "Performing initial tool scan...");
    for (int i = 0; i < 5; i++) {
        ds18b20_task_run();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Check if tool is present
    if (ds18b20_is_present()) {
        uint16_t tool_id = ds18b20_get_id();
        ESP_LOGI(TAG, "Tool already present at boot! ID=%d - powering up", tool_id);
        cached_tool_id = tool_id;
        hardware_set_7v8(true);
        enter_state(ATTACHED);
        // Keep detection disabled after boot (tool is already attached)
        ds18b20_set_detection_enabled(false);
    } else {
        ESP_LOGI(TAG, "No tool detected at boot");
        enter_state(IDLE);
        // Disable detection after boot (no tool, wait for UNLOCK command)
        ds18b20_set_detection_enabled(false);
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
            uint8_t cmd;
            uint16_t tool_id;
            
            portENTER_CRITICAL(&ros_spinlock);
            cmd = ros_last_command;
            tool_id = ros_last_tool_id;
            ros_command_received = false;
            portEXIT_CRITICAL(&ros_spinlock);
            
            switch(cmd) {
                case CMD_UNLOCK:  // 0 - UNLOCK command
                    if (current_state == IDLE && !waiting_for_tool) {
                        // ATTACH sequence: unlock to insert tool
                        ESP_LOGI(TAG, "ROS ATTACH command: Unlocking, waiting for tool insertion");
                        hardware_set_7v8(false);
                        hardware_set_5v(true);
                        set_servo_locked(false);
                        ds18b20_set_detection_enabled(true);
                        waiting_for_tool = true;
                        waiting_for_insertion = true;
                        unlock_start_time = now;
                        enter_state(UNLOCKING);
                    } 
                    else if (current_state == ATTACHED && !waiting_for_tool) {
                        // RELEASE sequence: unlock to remove tool
                        ESP_LOGI(TAG, "ROS RELEASE command: Unlocking, waiting for tool removal");
                        hardware_set_7v8(false);
                        hardware_set_5v(true);
                        set_servo_locked(false);
                        ds18b20_set_detection_enabled(true);
                        waiting_for_tool = true;
                        waiting_for_insertion = false;
                        unlock_start_time = now;
                        enter_state(UNLOCKING);
                    }
                    else {
                        ESP_LOGW(TAG, "UNLOCK ignored: state=%d, waiting=%d", 
                                 current_state, waiting_for_tool);
                    }
                    break;
                    
                case CMD_LOCK:  // 1 - Manual lock
                    ESP_LOGI(TAG, "ROS LOCK command");
                    hardware_set_5v(true);
                    set_servo_locked(true);
                    if (tool_present) {
                        hardware_set_7v8(true);
                        waiting_for_tool = false;
                        ds18b20_set_detection_enabled(false);
                        enter_state(ATTACHED);
                    } else {
                        hardware_set_5v(false);
                        hardware_set_7v8(false);
                        waiting_for_tool = false;
                        ds18b20_set_detection_enabled(false);
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
        
        // Waiting for tool action (UNLOCKING state)
        if (waiting_for_tool) {
            if (waiting_for_insertion && tool_present) {
                // Tool INSERTED - lock it
                ESP_LOGI(TAG, "Tool inserted → Locking");
                cached_tool_id = ds18b20_get_id();
                hardware_set_5v(true);
                set_servo_locked(true);
                hardware_set_7v8(true);
                hardware_set_5v(false);
                waiting_for_tool = false;
                ds18b20_set_detection_enabled(false);
                enter_state(ATTACHED);
            }
            else if (!waiting_for_insertion && !tool_present) {
                // Tool REMOVED - go to IDLE
                ESP_LOGI(TAG, "Tool removed → IDLE");
                cached_tool_id = 0;
                set_servo_locked(true);
                hardware_set_5v(false);
                hardware_set_7v8(false);
                waiting_for_tool = false;
                ds18b20_set_detection_enabled(false);
                enter_state(IDLE);
            }
            else if ((now - unlock_start_time) >= UNLOCK_TIMEOUT_MS) {
                // Timeout
                if (waiting_for_insertion) {
                    ESP_LOGW(TAG, "Timeout - no tool inserted (%u ms) - relocking", UNLOCK_TIMEOUT_MS);
                    publish_warning("No tool inserted within timeout");
                } else {
                    ESP_LOGW(TAG, "Timeout - tool not removed (%u ms) - relocking", UNLOCK_TIMEOUT_MS);
                    publish_warning("Tool not removed within timeout");
                }
                hardware_set_5v(true);
                set_servo_locked(true);
                hardware_set_5v(false);
                if (waiting_for_insertion) {
                    hardware_set_7v8(false);
                    enter_state(IDLE);
                } else {
                    hardware_set_7v8(true);
                    enter_state(ATTACHED);
                }
                waiting_for_tool = false;
                ds18b20_set_detection_enabled(false);
            }
        }
        
        // Check for unexpected tool disappearance in ATTACHED
        if (current_state == ATTACHED && !tool_present && !waiting_for_tool) {
            ESP_LOGE(TAG, "Unexpected: Tool disappeared while ATTACHED");
            cached_tool_id = 0;
            hardware_set_5v(false);
            hardware_set_7v8(false);
            set_servo_locked(true);
            enter_state(IDLE);
            publish_warning("Tool unexpectedly disconnected");
        }
        
        publish_current_status();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}