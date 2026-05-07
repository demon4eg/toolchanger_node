#include "hardware.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"



#define TAG "HARDWARE"
#define PIN_5V_EN   11
#define PIN_7V8_EN  12
#define PIN_SERVO   7
#define SERVO_LOCK  2000
#define SERVO_UNLOCK 1000

// Store PWM channel configurations
typedef struct {
    ledc_channel_t channel;
    ledc_mode_t speed_mode;
} pwm_channel_t;

static pwm_channel_t pwm_channels[LEDC_CHANNEL_MAX];

void hardware_pwm_init(int gpio, int timer_num, int channel_num, int freq_hz, int resolution_bits)
{
    ledc_timer_t timer = (ledc_timer_t)timer_num;
    ledc_channel_t channel = (ledc_channel_t)channel_num;
    ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
    
    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = mode,
        .timer_num = timer,
        .duty_resolution = resolution_bits,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // Configure channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = gpio,
        .speed_mode = mode,
        .channel = channel,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
    
    // Store for later use
    pwm_channels[channel_num].channel = channel;
    pwm_channels[channel_num].speed_mode = mode;
    
    ESP_LOGI(TAG, "PWM initialized on GPIO%d (timer=%d, channel=%d, freq=%dHz)", 
             gpio, timer_num, channel_num, freq_hz);
}

void hardware_pwm_set_duty(int channel_num, uint32_t duty)
{
    ledc_channel_t channel = (ledc_channel_t)channel_num;
    ledc_mode_t mode = pwm_channels[channel_num].speed_mode;
    
    ledc_set_duty(mode, channel, duty);
    ledc_update_duty(mode, channel);
}

void hardware_init(void)
{
    gpio_config_t en = {
        .pin_bit_mask = (1ULL << PIN_5V_EN) | (1ULL << PIN_7V8_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&en);
    gpio_set_level(PIN_5V_EN, 0);
    gpio_set_level(PIN_7V8_EN, 0);
    
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    
    ledc_channel_config_t c = {
        .gpio_num = PIN_SERVO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&c);
    
    hardware_servo_lock();
    ESP_LOGI(TAG, "Hardware initialized");
}

void hardware_button_init(void)
{
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << PIN_BUTTON_UNLOCK),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_config);
    ESP_LOGI(TAG, "Button configured on GPIO%d", PIN_BUTTON_UNLOCK);
}

void hardware_set_5v(bool enable)
{
    gpio_set_level(PIN_5V_EN, enable ? 1 : 0);
    ESP_LOGI(TAG, "5V rail: %s", enable ? "ON" : "OFF");
}

void hardware_set_7v8(bool enable)
{
    gpio_set_level(PIN_7V8_EN, enable ? 1 : 0);
    ESP_LOGI(TAG, "7.8V rail: %s", enable ? "ON" : "OFF");
}

static void set_servo(uint16_t pulse_us)
{
    uint32_t duty = (pulse_us * 8191) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void hardware_servo_lock(void)
{
    set_servo(SERVO_LOCK);
    ESP_LOGI(TAG, "Servo LOCKED");
}

void hardware_servo_unlock(void)
{
    set_servo(SERVO_UNLOCK);
    ESP_LOGI(TAG, "Servo UNLOCKED");
}