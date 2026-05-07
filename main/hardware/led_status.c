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
static SemaphoreHandle_t led_mutex = NULL;
static TaskHandle_t led_task_handle = NULL;
static bool led_task_running = false;

// LED control task (handles blinking)
static void led_task(void *arg)
{
    bool led_state = false;
    TickType_t last_toggle = 0;
    uint32_t blink_interval_ms = 0;
    
    while (led_task_running) {
        // Get current settings
        uint8_t r, g, b;
        led_pattern_t pattern;
        
        xSemaphoreTake(led_mutex, portMAX_DELAY);
        r = current_r;
        g = current_g;
        b = current_b;
        pattern = current_pattern;
        xSemaphoreGive(led_mutex);
        
        // Determine blink interval based on pattern
        switch (pattern) {
            case LED_PATTERN_SLOW_BLINK:
                blink_interval_ms = 500;  // 1Hz
                break;
            case LED_PATTERN_FAST_BLINK:
                blink_interval_ms = 100;  // 5Hz
                break;
            case LED_PATTERN_VERY_FAST:
                blink_interval_ms = 50;   // 10Hz
                break;
            case LED_PATTERN_SOLID:
            default:
                // Solid: always on
                if (led_strip) {
                    led_strip_set_pixel(led_strip, 0, r, g, b);
                    led_strip_refresh(led_strip);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
        }
        
        // Handle blinking
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
    ESP_LOGI(TAG, "Initializing LED on GPIO%d", RGB_LED_GPIO);
    
    // Configure LED strip
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
    
    // Create mutex
    led_mutex = xSemaphoreCreateMutex();
    
    // Create LED task
    led_task_running = true;
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);
    
    // Set default color (blue solid during boot)
    led_status_set(0, 0, 255, LED_PATTERN_SOLID);
    
    ESP_LOGI(TAG, "LED initialized");
}

void led_status_set(uint8_t red, uint8_t green, uint8_t blue, led_pattern_t pattern)
{
    if (led_mutex == NULL) return;
    
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    current_r = red;
    current_g = green;
    current_b = blue;
    current_pattern = pattern;
    xSemaphoreGive(led_mutex);
    
    ESP_LOGD(TAG, "LED set: R=%d G=%d B=%d, pattern=%d", red, green, blue, pattern);
}

void led_status_off(void)
{
    led_status_set(0, 0, 0, LED_PATTERN_SOLID);
}

void led_status_red(led_pattern_t pattern)
{
    led_status_set(255, 0, 0, pattern);
}

void led_status_green(led_pattern_t pattern)
{
    led_status_set(0, 255, 0, pattern);
}

void led_status_blue(led_pattern_t pattern)
{
    led_status_set(0, 0, 255, pattern);
}

void led_status_yellow(led_pattern_t pattern)
{
    led_status_set(255, 255, 0, pattern);
}

// Status mapping
void led_status_update_from_state(uint8_t state, bool ros_connected, bool overcurrent)
{
    // Overcurrent has highest priority
    if (overcurrent) {
        led_status_red(LED_PATTERN_VERY_FAST);
        return;
    }
    
    // ROS disconnected
    if (!ros_connected) {
        led_status_blue(LED_PATTERN_SLOW_BLINK);
        return;
    }
    
    // State machine based
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
        default:  // ERROR
            led_status_red(LED_PATTERN_SOLID);
            break;
    }
}