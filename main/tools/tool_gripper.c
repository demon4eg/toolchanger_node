#include "tool_gripper.h"
#include "tool_manager.h"
#include "hardware.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/i2c.h"  // For INA3221

#define TAG "TOOL_GRIPPER"

// Command ranges
#define GRIPPER_CMD_ANGLE_MIN   0
#define GRIPPER_CMD_ANGLE_MAX   180
#define GRIPPER_CMD_OPEN        4
#define GRIPPER_CMD_CLOSE       5
#define GRIPPER_CMD_EFFORT_MIN  200
#define GRIPPER_CMD_EFFORT_MAX  300

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
#define KP_EFFORT               1.5f
#define IDLE_CURRENT_MA         110     // No-load current threshold
#define MAX_STEP_PER_CYCLE      400     // Max position change per 10ms
#define MIN_STEP_PER_CYCLE      10      // Min step for "squeeze"

// INA3221 configuration
#define INA3221_ADDR            0x40
#define I2C_MASTER_SCL_IO       1
#define I2C_MASTER_SDA_IO       2
#define I2C_MASTER_FREQ_HZ      100000

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

// Read current from INA3221
static void read_current(void)
{
    // TODO: Implement INA3221 read
    // For now, simulate or return 0 if not yet implemented
    // current_effort_ma = ina3221_read_current_ma(1);
    
    // Temporary: just use filtered value or 0
    if (filtered_effort == 0) {
        filtered_effort = IDLE_CURRENT_MA;
    }
    filtered_effort = (alpha * current_effort_ma) + ((1.0 - alpha) * filtered_effort);
    current_effort_ma = (int32_t)filtered_effort;
}

// PID regulation (mirroring STM32 logic)
static void gripper_regulate(void)
{
    read_current();
    
    int32_t pos_diff = target_pos - current_pos;
    
    // OPENING (moving toward MAX_POS) - full speed
    if (pos_diff > 0) {
        current_pos = target_pos;  // Instant move when opening
        gripper_set_position(current_pos);
    }
    // CLOSING (moving toward MIN_POS) - effort controlled
    else if (pos_diff < 0) {
        int32_t effort_gap = target_effort_ma - current_effort_ma;
        
        if (effort_gap > 0) {
            // Below target effort - can move closer
            int32_t dynamic_step = (int32_t)(effort_gap * KP_EFFORT);
            dynamic_step = constrain(dynamic_step, MIN_STEP_PER_CYCLE, MAX_STEP_PER_CYCLE);
            
            if (abs(pos_diff) > dynamic_step) {
                current_pos -= dynamic_step;
            } else {
                current_pos = target_pos;
            }
        } else {
            // OVERLOAD: Current exceeds target, back off slightly
            current_pos += 50;
        }
        
        gripper_set_position(current_pos);
    }
    
    ESP_LOGD(TAG, "Regulate: pos=%d, target=%d, effort=%d/%d, diff=%d",
             (int)current_pos, (int)target_pos, (int)current_effort_ma, (int)target_effort_ma, (int)pos_diff);
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
    
    // Direct angle commands (0-180) - convert to position
    if (command >= GRIPPER_CMD_ANGLE_MIN && command <= GRIPPER_CMD_ANGLE_MAX) {
        int32_t pos = angle_to_position(command);
        gripper_set_target_position(pos);
    }
    
    // Shortcut commands
    else if (command == GRIPPER_CMD_OPEN) {
        gripper_set_target_position(GRIPPER_POS_MAX);  // Fully open
    }
    else if (command == GRIPPER_CMD_CLOSE) {
        gripper_set_target_position(GRIPPER_POS_MIN);  // Fully closed
    }
    
    // Effort control (200-300 maps to 0-1200 mA)
    else if (command >= GRIPPER_CMD_EFFORT_MIN && command <= GRIPPER_CMD_EFFORT_MAX) {
        uint8_t percent = command - GRIPPER_CMD_EFFORT_MIN + 1;
        int32_t effort_ma = (percent * 1200) / 100;  // 1-100% → 0-1200mA
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
    
    // Initialize INA3221 on I2C (GPIO1=SCL, GPIO2=SDA)
    // TODO: Add INA3221 initialization
    
    // Set default position (open)
    gripper_set_position(GRIPPER_POS_MAX);
    
    // Start regulation task
    xTaskCreate(gripper_regulation_task, "gripper_reg", 2048, NULL, 10, NULL);
    
    // Register with tool manager
    static tool_registration_t gripper_tool = {
        .command_min = 0,
        .command_max = 300,
        .handler = tool_gripper_process_command,
        .name = "gripper"
    };
    
    tool_manager_register_tool(&gripper_tool);
    ESP_LOGI(TAG, "Gripper tool initialized");
}