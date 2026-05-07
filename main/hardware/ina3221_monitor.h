#pragma once

#include <stdint.h>
#include <stdbool.h>

// Channel mapping (matching your example)
// CH1 = 7.8V rail (tools)
// CH2 = 5V rail (logic)
// CH3 = unused
#define INA3221_CH_7V8   0   // CH3 in example
#define INA3221_CH_5V    1   // CH2 in example

// Overcurrent thresholds (mA)
#define OVERCURRENT_5V_MA     500   // 0.5A
#define OVERCURRENT_7V8_MA    1200  // 1.2A

void ina3221_monitor_init(void);
float ina3221_get_current_ma(uint8_t channel);
float ina3221_get_voltage_mv(uint8_t channel);
bool ina3221_is_overcurrent(uint8_t channel);
void ina3221_update_filtered_current(void);  // Call in regulation task
float ina3221_get_filtered_current_ma(uint8_t channel);
void ina3221_set_alpha(float alpha);  // EMA filter coefficient (0.13 = 13%)