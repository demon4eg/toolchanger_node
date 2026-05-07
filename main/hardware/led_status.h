#pragma once

#include <stdint.h>
#include <stdbool.h>

// LED Patterns
typedef enum {
    LED_PATTERN_SOLID = 0,
    LED_PATTERN_SLOW_BLINK,
    LED_PATTERN_FAST_BLINK,
    LED_PATTERN_VERY_FAST,
} led_pattern_t;

// Brightness levels (0-255, where 0=off, 255=max)
typedef enum {
    LED_BRIGHTNESS_LOW = 20,
    LED_BRIGHTNESS_MEDIUM = 80,
    LED_BRIGHTNESS_HIGH = 150,
    LED_BRIGHTNESS_MAX = 255,
} led_brightness_t;

// Initialize LED
void led_status_init(void);

// Set LED color, pattern, and brightness
void led_status_set(uint8_t red, uint8_t green, uint8_t blue, led_pattern_t pattern, uint8_t brightness);

// Set global brightness for all future LEDs
void led_status_set_brightness(uint8_t brightness);

// Quick helper functions (use current brightness)
void led_status_off(void);
void led_status_red(led_pattern_t pattern);
void led_status_green(led_pattern_t pattern);
void led_status_blue(led_pattern_t pattern);
void led_status_yellow(led_pattern_t pattern);

// Status LED update
void led_status_update_from_state(uint8_t state, bool ros_connected, bool overcurrent);