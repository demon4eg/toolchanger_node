#pragma once

#include <stdint.h>
#include <stdbool.h>

// LED Patterns
typedef enum {
    LED_PATTERN_SOLID = 0,
    LED_PATTERN_SLOW_BLINK,    // 1Hz (500ms on, 500ms off)
    LED_PATTERN_FAST_BLINK,    // 5Hz (100ms on, 100ms off)
    LED_PATTERN_VERY_FAST,     // 10Hz (50ms on, 50ms off)
} led_pattern_t;

// Initialize LED (GPIO21)
void led_status_init(void);

// Set LED color and pattern
void led_status_set(uint8_t red, uint8_t green, uint8_t blue, led_pattern_t pattern);

// Quick helper functions
void led_status_off(void);
void led_status_red(led_pattern_t pattern);
void led_status_green(led_pattern_t pattern);
void led_status_blue(led_pattern_t pattern);
void led_status_yellow(led_pattern_t pattern);

// Status LED update (call from state machine)
void led_status_update_from_state(uint8_t state, bool ros_connected, bool overcurrent);