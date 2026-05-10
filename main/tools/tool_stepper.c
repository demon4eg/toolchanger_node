#include "tool_stepper.h"
#include "tool_manager.h"
#include "hardware.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "TOOL_STEPPER"

// Hardware pins
#define STEPPER_STEP_GPIO   5   // STEP signal
#define STEPPER_DIR_GPIO    8   // DIR signal
#define STEPPER_ENABLE_GPIO -1  // Optional, set to GPIO if needed

// Motion limits
#define STEPPER_POS_MIN     0
#define STEPPER_POS_MAX     100000  // Steps (0-100%)
#define STEPPER_SPEED_MIN   50      // Steps per second
#define STEPPER_SPEED_MAX   2000

// Default values
#define DEFAULT_SPEED       500     // steps/sec

static int32_t current_pos = 0;
static int32_t target_pos = 0;
static int32_t current_speed = DEFAULT_SPEED;
static bool is_moving = false;
static bool homing = false;

// Feedback functions
int32_t stepper_get_current_position(void) { return current_pos; }
int32_t stepper_get_current_speed(void) { return current_speed; }
uint8_t stepper_get_status(void) 
{
    if (homing) return 3;      // homing
    if (is_moving) return 1;   // moving
    return 0;                  // idle
}

// Step pulse generation (simple delay loop)
static void generate_step_pulse(void)
{
    gpio_set_level(STEPPER_STEP_GPIO, 1);
    esp_rom_delay_us(5);  // 5us pulse width
    gpio_set_level(STEPPER_STEP_GPIO, 0);
}

// Move stepper to target position (blocking)
static void stepper_move_to_position(int32_t pos)
{
    int32_t steps = pos - current_pos;
    if (steps == 0) return;
    
    // Set direction
    gpio_set_level(STEPPER_DIR_GPIO, (steps > 0) ? 1 : 0);
    
    // Calculate delay between steps (microseconds)
    int32_t delay_us = 1000000 / current_speed;
    steps = abs(steps);
    
    is_moving = true;
    
    for (int32_t i = 0; i < steps; i++) {
        generate_step_pulse();
        esp_rom_delay_us(delay_us);
        
        // Update current position
        if (gpio_get_level(STEPPER_DIR_GPIO)) {
            current_pos++;
        } else {
            current_pos--;
        }
    }
    
    is_moving = false;
}

// Homing sequence - move to limit switch (simplified)
static void stepper_home(void)
{
    homing = true;
    ESP_LOGI(TAG, "Homing...");
    
    // Move backwards until limit switch (implement with GPIO input)
    // For now, just move to zero position
    stepper_move_to_position(0);
    
    homing = false;
    ESP_LOGI(TAG, "Homing complete, position: %ld", current_pos);
}

// Process commands using defines from tool_manager.h
void tool_stepper_process_command(uint8_t command, uint16_t tool_id, uint16_t param)
{
    ESP_LOGI(TAG, "Stepper command: %d", command);
    
    // Position control (70-100)
    if (command >= TOOL_CMD_STEPPER_POS_MIN && command <= TOOL_CMD_STEPPER_POS_MAX) {
        float percent = (float)(command - TOOL_CMD_STEPPER_POS_MIN) / 
                        (TOOL_CMD_STEPPER_POS_MAX - TOOL_CMD_STEPPER_POS_MIN);
        int32_t pos = percent * STEPPER_POS_MAX;
        target_pos = pos;
        stepper_move_to_position(target_pos);
    }
    // Speed control (101-120)
    else if (command >= TOOL_CMD_STEPPER_SPEED_MIN && command <= TOOL_CMD_STEPPER_SPEED_MAX) {
        float percent = (float)(command - TOOL_CMD_STEPPER_SPEED_MIN) / 
                        (TOOL_CMD_STEPPER_SPEED_MAX - TOOL_CMD_STEPPER_SPEED_MIN);
        current_speed = STEPPER_SPEED_MIN + percent * (STEPPER_SPEED_MAX - STEPPER_SPEED_MIN);
        ESP_LOGI(TAG, "Speed set to: %ld steps/sec", current_speed);
    }
    // Home (121)
    else if (command == TOOL_CMD_STEPPER_HOME) {
        stepper_home();
    }
    // Stop (122)
    else if (command == TOOL_CMD_STEPPER_STOP) {
        is_moving = false;
        ESP_LOGI(TAG, "Stopped");
    }
    else {
        ESP_LOGW(TAG, "Unknown stepper command: %d", command);
    }
}

// Initialize stepper hardware
void tool_stepper_init(void)
{
    ESP_LOGI(TAG, "Initializing stepper tool");
    
    // Configure GPIOs
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEPPER_STEP_GPIO) | (1ULL << STEPPER_DIR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Initial state
    gpio_set_level(STEPPER_STEP_GPIO, 0);
    gpio_set_level(STEPPER_DIR_GPIO, 0);
    
    // Optionally enable driver
    if (STEPPER_ENABLE_GPIO >= 0) {
        gpio_config_t enable_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << STEPPER_ENABLE_GPIO),
        };
        gpio_config(&enable_conf);
        gpio_set_level(STEPPER_ENABLE_GPIO, 1);  // Enable
    }
    
    ESP_LOGI(TAG, "Stepper tool initialized");
}