// Usage Examples
// c
// // Set low brightness (dim)
// led_status_set_brightness(LED_BRIGHTNESS_LOW);

// // Set medium brightness
// led_status_set_brightness(LED_BRIGHTNESS_MEDIUM);

// // Set custom brightness (0-255)
// led_status_set_brightness(50);

// // Set color with specific brightness
// led_status_set(0, 255, 0, LED_PATTERN_SOLID, 30);
// Test Code
// c
// void test_led_brightness(void)
// {
//     led_status_init();
    
//     // Test different brightness levels
//     led_status_set_brightness(20);  // Very dim
//     led_status_green(LED_PATTERN_SOLID);
//     vTaskDelay(pdMS_TO_TICKS(2000));
    
//     led_status_set_brightness(80);  // Medium
//     vTaskDelay(pdMS_TO_TICKS(2000));
    
//     led_status_set_brightness(150); // Bright
//     vTaskDelay(pdMS_TO_TICKS(2000));
    
//     led_status_set_brightness(255); // Max
//     vTaskDelay(pdMS_TO_TICKS(2000));
    
//     led_status_off();
// }



#include "led_status.h"
#include "led_strip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "LED_STATUS"
#define RGB_LED_GPIO 21

static led_strip_handle_t led_strip = NULL;
static led_pattern_t current_pattern = LED_PATTERN_SOLID;
static uint8_t current_r = 0, current_g = 0, current_b = 0;
static uint8_t current_brightness = LED_BRIGHTNESS_MEDIUM;  // Default medium
static SemaphoreHandle_t led_mutex = NULL;
static TaskHandle_t led_task_handle = NULL;
static bool led_task_running = false;

// Apply brightness to RGB values
static void apply_brightness(uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = (*r * current_brightness) / 255;
    *g = (*g * current_brightness) / 255;
    *b = (*b * current_brightness) / 255;
}

// LED control task
static void led_task(void *arg)
{
    bool led_state = false;
    TickType_t last_toggle = 0;
    uint32_t blink_interval_ms = 0;
    
    while (led_task_running) {
        uint8_t r, g, b;
        led_pattern_t pattern;
        
        xSemaphoreTake(led_mutex, portMAX_DELAY);
        r = current_r;
        g = current_g;
        b = current_b;
        pattern = current_pattern;
        xSemaphoreGive(led_mutex);
        
        // Apply brightness
        apply_brightness(&r, &g, &b);
        
        switch (pattern) {
            case LED_PATTERN_SLOW_BLINK:
                blink_interval_ms = 500;
                break;
            case LED_PATTERN_FAST_BLINK:
                blink_interval_ms = 100;
                break;
            case LED_PATTERN_VERY_FAST:
                blink_interval_ms = 50;
                break;
            case LED_PATTERN_SOLID:
            default:
                if (led_strip) {
                    led_strip_set_pixel(led_strip, 0, r, g, b);
                    led_strip_refresh(led_strip);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
        }
        
        TickType_t now = xTaskGetTickCount();
        if ((now - last_toggle) >= pdMS_TO_TICKS(blink_interval_ms)) {
            led_state = !led_state;
            last_toggle = now;
            
            if (led_strip) {
                if (led_state) {
                    led_strip_set_pixel(led_strip, 0, r, g, b);
                } else {
                    led_strip_set_pixel(led_strip, 0, 0, 0, 0);
                }
                led_strip_refresh(led_strip);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    vTaskDelete(NULL);
}

void led_status_init(void)
{
    ESP_LOGI(TAG, "Initializing LED on GPIO%d with brightness %d", RGB_LED_GPIO, current_brightness);
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };
    
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags = {
            .with_dma = false,
        },
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    
    led_mutex = xSemaphoreCreateMutex();
    led_task_running = true;
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);
    
    led_status_set(0, 0, 255, LED_PATTERN_SOLID, current_brightness);
    
    ESP_LOGI(TAG, "LED initialized");
}

void led_status_set(uint8_t red, uint8_t green, uint8_t blue, led_pattern_t pattern, uint8_t brightness)
{
    if (led_mutex == NULL) return;
    
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    current_r = red;
    current_g = green;
    current_b = blue;
    current_pattern = pattern;
    current_brightness = brightness;
    xSemaphoreGive(led_mutex);
    
    ESP_LOGD(TAG, "LED set: R=%d G=%d B=%d, pattern=%d, brightness=%d", red, green, blue, pattern, brightness);
}

void led_status_set_brightness(uint8_t brightness)
{
    if (brightness > 255) brightness = 255;
    current_brightness = brightness;
    ESP_LOGI(TAG, "LED brightness set to %d", brightness);
}

void led_status_off(void)
{
    led_status_set(0, 0, 0, LED_PATTERN_SOLID, current_brightness);
}

void led_status_red(led_pattern_t pattern)
{
    led_status_set(255, 0, 0, pattern, current_brightness);
}

void led_status_green(led_pattern_t pattern)
{
    led_status_set(0, 255, 0, pattern, current_brightness);
}

void led_status_blue(led_pattern_t pattern)
{
    led_status_set(0, 0, 255, pattern, current_brightness);
}

void led_status_yellow(led_pattern_t pattern)
{
    led_status_set(255, 255, 0, pattern, current_brightness);
}

void led_status_update_from_state(uint8_t state, bool ros_connected, bool overcurrent)
{
    if (overcurrent) {
        led_status_red(LED_PATTERN_VERY_FAST);
        return;
    }
    
    if (!ros_connected) {
        led_status_blue(LED_PATTERN_SLOW_BLINK);
        return;
    }
    
    switch (state) {
        case 0:  // IDLE
            led_status_green(LED_PATTERN_SOLID);
            break;
        case 2:  // ATTACHED
            led_status_green(LED_PATTERN_FAST_BLINK);
            break;
        case 3:  // UNLOCKING
            led_status_yellow(LED_PATTERN_SOLID);
            break;
        default:
            led_status_red(LED_PATTERN_SOLID);
            break;
    }
}