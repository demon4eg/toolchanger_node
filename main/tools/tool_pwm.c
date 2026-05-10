#include "tool_pwm.h"
#include "tool_manager.h"
#include "hardware.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "ina3221_monitor.h"

#define TAG "TOOL_PWM"

// Hardware pins
#define PWM_OUTPUT_GPIO     5   // PWM signal
#define PWM_DIR_GPIO        8   // Direction (optional)

// PWM configuration
#define PWM_TIMER           LEDC_TIMER_2
#define PWM_CHANNEL         LEDC_CHANNEL_2
#define PWM_FREQ_DEFAULT    25000   // 25kHz for motors
#define PWM_RESOLUTION      10      // 10-bit (0-1023)

static uint8_t current_velocity = 0;    // 0-100%
static uint8_t current_effort = 0;      // 0-100%
static bool enabled = false;
static bool direction = false;

// Feedback functions
int32_t pwm_get_current_velocity(void) { return current_velocity; }
int32_t pwm_get_current_effort(void) { return current_effort; }
uint8_t pwm_get_status(void) 
{
    if (!enabled) return 0;     // idle/off
    if (current_velocity > 0) return 1;  // moving
    return 0;
}

// Set PWM duty cycle (0-100%)
static void pwm_set_duty_percent(uint8_t percent)
{
    if (percent > 100) percent = 100;
    
    uint32_t duty = (percent * (1 << PWM_RESOLUTION)) / 100;
    hardware_pwm_set_duty(PWM_CHANNEL, duty);
    
    ESP_LOGD(TAG, "PWM duty: %d%%", percent);
}

// Set direction
static void pwm_set_direction(bool dir)
{
    direction = dir;
    if (PWM_DIR_GPIO >= 0) {
        gpio_set_level(PWM_DIR_GPIO, dir ? 1 : 0);
        ESP_LOGD(TAG, "Direction: %s", dir ? "FORWARD" : "REVERSE");
    }
}

// Process commands using defines from tool_manager.h
void tool_pwm_process_command(uint8_t command, uint16_t tool_id, uint16_t param)
{
    ESP_LOGI(TAG, "PWM command: %d", command);
    
    // Velocity control (130-150)
    if (command >= TOOL_CMD_PWM_VEL_MIN && command <= TOOL_CMD_PWM_VEL_MAX) {
        float percent = (float)(command - TOOL_CMD_PWM_VEL_MIN) / 
                        (TOOL_CMD_PWM_VEL_MAX - TOOL_CMD_PWM_VEL_MIN);
        current_velocity = percent * 100;
        
        if (enabled) {
            pwm_set_duty_percent(current_velocity);
        }
        ESP_LOGI(TAG, "Velocity set to: %d%%", current_velocity);
    }
    // Effort control (151-170)
    else if (command >= TOOL_CMD_PWM_EFFORT_MIN && command <= TOOL_CMD_PWM_EFFORT_MAX) {
        float percent = (float)(command - TOOL_CMD_PWM_EFFORT_MIN) / 
                        (TOOL_CMD_PWM_EFFORT_MAX - TOOL_CMD_PWM_EFFORT_MIN);
        current_effort = percent * 100;
        
        // For effort mode, adjust PWM based on current feedback (simplified)
        float current_ma = ina3221_get_current_ma(INA3221_CH_7V8);
        if (current_ma < current_effort * 12) {  // 0-100% -> 0-1200mA
            // Increase PWM - implement PID later
        }
        
        ESP_LOGI(TAG, "Effort set to: %d%%", current_effort);
    }
    // On (171)
    else if (command == TOOL_CMD_PWM_ON) {
        enabled = true;
        pwm_set_duty_percent(current_velocity);
        ESP_LOGI(TAG, "Output ENABLED");
    }
    // Off (172)
    else if (command == TOOL_CMD_PWM_OFF) {
        enabled = false;
        pwm_set_duty_percent(0);
        ESP_LOGI(TAG, "Output DISABLED");
    }
    else {
        ESP_LOGW(TAG, "Unknown PWM command: %d", command);
    }
}

// Initialize PWM tool
void tool_pwm_init(void)
{
    ESP_LOGI(TAG, "Initializing PWM tool on GPIO%d", PWM_OUTPUT_GPIO);
    
    // Configure PWM output
    hardware_pwm_init(PWM_OUTPUT_GPIO, PWM_TIMER, PWM_CHANNEL, 
                      PWM_FREQ_DEFAULT, PWM_RESOLUTION);
    
    // Configure direction pin if available
    if (PWM_DIR_GPIO >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << PWM_DIR_GPIO),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(PWM_DIR_GPIO, 0);
    }
    
    // Start with output disabled
    pwm_set_duty_percent(0);
    enabled = false;
    
    ESP_LOGI(TAG, "PWM tool initialized (freq=%dHz)", PWM_FREQ_DEFAULT);
}