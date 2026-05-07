#include "tool_gripper.h"
#include "tool_manager.h"
#include "hardware.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/i2c.h"  // For INA3221
#include "ina3221_monitor.h"

#define TAG "TOOL_GRIPPER"


// Hardware
#define GRIPPER_PWM_GPIO        5
#define GRIPPER_PWM_TIMER       LEDC_TIMER_1
#define GRIPPER_PWM_CHANNEL     LEDC_CHANNEL_1
#define GRIPPER_PWM_FREQ        50
#define GRIPPER_PWM_RESOLUTION  13

// Servo pulse widths (adjust for your servo)
#define GRIPPER_PULSE_MIN       1150   // 0 microns / 0 degrees
#define GRIPPER_PULSE_MAX       2100   // 20000 microns / 180 degrees

// Position range (mirroring STM32)
#define GRIPPER_POS_MIN         0       // 0 microns - closed
#define GRIPPER_POS_MAX         20000   // 20mm - open

// PID constants for effort control (tune these)
#define KP_EFFORT               1.0f
#define IDLE_CURRENT_MA         110     // No-load current threshold
#define MAX_STEP_PER_CYCLE      100     // Max position change per 10ms
#define MIN_STEP_PER_CYCLE      50      // Min step for "squeeze"

#define EFFORT_DEADBAND_MA   20  // 20mA deadband to prevent hunting
#define BACKOFF_STEP            300   // 0.8mm - enough to release stall
#define POSITION_HYSTERESIS     200     // 0.2mm - don't react to small position changes
#define EFFORT_HYSTERESIS       50      // 30mA - prevent hunting

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static int32_t current_pos = GRIPPER_POS_MAX;      // Start open
static int32_t target_pos = GRIPPER_POS_MAX;
static int32_t target_effort_ma = 300;             // Default 300mA
static int32_t current_effort_ma = 0;
static float filtered_effort = 0;
static float alpha = 0.13f;  // EMA filter

// Convert position (microns) to PWM duty
static uint32_t pos_to_duty(int32_t pos)
{
    if (pos < GRIPPER_POS_MIN) pos = GRIPPER_POS_MIN;
    if (pos > GRIPPER_POS_MAX) pos = GRIPPER_POS_MAX;
    
    float slope = (float)(GRIPPER_PULSE_MAX - GRIPPER_PULSE_MIN) / (GRIPPER_POS_MAX - GRIPPER_POS_MIN);
    int32_t pulse_us = GRIPPER_PULSE_MIN + (int32_t)(slope * (pos - GRIPPER_POS_MIN));
    
    // Convert pulse width (us) to duty cycle for 50Hz (20ms period)
    // Duty = (pulse_us / 20000) * 2^resolution
    uint32_t duty = (pulse_us * (1 << GRIPPER_PWM_RESOLUTION)) / 20000;
    return duty;
}

// Update servo position
static void gripper_set_position(int32_t pos)
{
    if (pos < GRIPPER_POS_MIN) pos = GRIPPER_POS_MIN;
    if (pos > GRIPPER_POS_MAX) pos = GRIPPER_POS_MAX;
    current_pos = pos;
    
    uint32_t duty = pos_to_duty(pos);
    hardware_pwm_set_duty(GRIPPER_PWM_CHANNEL, duty);
    ESP_LOGD(TAG, "Position: %d microns, duty: %lu", (int)pos, duty);
}

// Read filtered current from INA3221 (matches STM32 EMA filter)
static void read_current(void)
{
    // Get raw current from INA3221 (mA)
    float raw_ma = ina3221_get_current_ma(INA3221_CH_7V8);
    
    // Apply EMA filter (same as STM32 - alpha 0.13)
    filtered_effort = (alpha * raw_ma) + ((1.0f - alpha) * filtered_effort);
    current_effort_ma = (int32_t)filtered_effort;
    
    // Optional: periodic log (every 5 seconds)
    static uint32_t last_log = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - last_log) > 2000) {
        ESP_LOGI(TAG, "Current: %ld mA (raw: %.0f mA)", current_effort_ma, raw_ma);
        last_log = now;
    }
}

// PID regulation 
static uint32_t last_backoff_time = 0;
static int32_t stable_position = -1;

static void gripper_regulate(void)
{
    read_current();
    
    int32_t posDiff = target_pos - current_pos;
    int32_t effortGap = target_effort_ma - current_effort_ma;
    
    // Apply position hysteresis - ignore tiny movements
    if (abs(posDiff) < POSITION_HYSTERESIS && posDiff != 0) {
        // Position is close enough, consider it reached
        current_pos = target_pos;
        gripper_set_position(current_pos);
        return;
    }
    
    // OVERLOAD: Current exceeds target - back off
    if (effortGap < -EFFORT_HYSTERESIS && current_pos > GRIPPER_POS_MIN) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if ((now - last_backoff_time) > 200) {
            current_pos += BACKOFF_STEP;
            if (current_pos > GRIPPER_POS_MAX) current_pos = GRIPPER_POS_MAX;
            gripper_set_position(current_pos);
            last_backoff_time = now;
        }
        return;
    }
    
    // OPENING - full speed
    if (posDiff > 0) {
        current_pos = target_pos;
        gripper_set_position(current_pos);
    }
    // CLOSING - dynamic effort control
    else if (posDiff < 0 && effortGap > EFFORT_HYSTERESIS) {
        int32_t dynamicStep = (int32_t)(effortGap * KP_EFFORT);
        dynamicStep = constrain(dynamicStep, MIN_STEP_PER_CYCLE, MAX_STEP_PER_CYCLE);
        
        if (abs(posDiff) > dynamicStep) {
            current_pos -= dynamicStep;
        } else {
            current_pos = target_pos;
        }
        current_pos = constrain(current_pos, GRIPPER_POS_MIN, GRIPPER_POS_MAX);
        gripper_set_position(current_pos);
    }
}

// Public: Set target position (microns)
static void gripper_set_target_position(int32_t pos)
{
    target_pos = constrain(pos, GRIPPER_POS_MIN, GRIPPER_POS_MAX);
    ESP_LOGI(TAG, "Target position: %d microns", (int)target_pos);
}

// Public: Set target effort (mA)
static void gripper_set_target_effort(int32_t ma)
{
    target_effort_ma = constrain(ma, 0, 1200);
    ESP_LOGI(TAG, "Target effort: %d mA", (int)target_effort_ma);
}

// Convert angle (0-180) to position (0-20000 microns)
static int32_t angle_to_position(uint8_t angle)
{
    return (angle * GRIPPER_POS_MAX) / 180;
}

void tool_gripper_process_command(uint8_t command, uint16_t tool_id, uint16_t param)
{
    ESP_LOGI(TAG, "Gripper command: %d, tool_id: %d", command, tool_id);
    
    // Position control - use defines from tool_manager.h
    if (command >= TOOL_CMD_GRIPPER_POS_MIN && command <= TOOL_CMD_GRIPPER_POS_MAX) {
        // Map to 0-20000 microns
        float percent = (float)(command - TOOL_CMD_GRIPPER_POS_MIN) / 
                        (TOOL_CMD_GRIPPER_POS_MAX - TOOL_CMD_GRIPPER_POS_MIN);
        int32_t pos = percent * GRIPPER_POS_MAX;
        gripper_set_target_position(pos);
    }
    
    // Effort control - use defines from tool_manager.h
    else if (command >= TOOL_CMD_GRIPPER_EFFORT_MIN && command <= TOOL_CMD_GRIPPER_EFFORT_MAX) {
        float percent = (float)(command - TOOL_CMD_GRIPPER_EFFORT_MIN) / 
                        (TOOL_CMD_GRIPPER_EFFORT_MAX - TOOL_CMD_GRIPPER_EFFORT_MIN);
        int32_t effort_ma = percent * 1200;
        gripper_set_target_effort(effort_ma);
    }
    
    else {
        ESP_LOGW(TAG, "Unknown gripper command: %d", command);
    }
}

// Periodic regulation task (runs at ~100Hz)
static void gripper_regulation_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 10ms = 100Hz
    
    while (1) {
        gripper_regulate();
        vTaskDelayUntil(&last_wake, period);
    }
}

void tool_gripper_init(void)
{
    // Initialize PWM
    hardware_pwm_init(GRIPPER_PWM_GPIO, GRIPPER_PWM_TIMER, GRIPPER_PWM_CHANNEL, 
                      GRIPPER_PWM_FREQ, GRIPPER_PWM_RESOLUTION);
    
    // Initialize INA3221 and set EMA filter coefficient (same as STM32: 0.13)
    ina3221_monitor_init();
    //ina3221_set_alpha(alpha);  // Use same alpha (0.13f)
    
    // Set default position (open)
    gripper_set_position(GRIPPER_POS_MAX);
    
    // Start regulation task
    xTaskCreate(gripper_regulation_task, "gripper_reg", 4096, NULL, 10, NULL);
    
    // Register with tool manager
    static tool_registration_t gripper_tool = {
        .type = TOOL_TYPE_GRIPPER, 
        .handler = tool_gripper_process_command,
        .name = "gripper",
        .cmd_min = TOOL_CMD_GRIPPER_POS_MIN,
        .cmd_max = TOOL_CMD_GRIPPER_EFFORT_MAX
    };
    
    tool_manager_register_tool(&gripper_tool);
    ESP_LOGI(TAG, "Gripper tool initialized with INA3221 effort control");
}