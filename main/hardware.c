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