#pragma once

#include <stdbool.h>
#include <stdint.h>

void ds18b20_task_start(void);
void ds18b20_task_run(void);
bool ds18b20_is_present(void);
uint16_t ds18b20_get_id(void);
float ds18b20_get_temp(void);
void ds18b20_set_detection_enabled(bool enabled);
bool ds18b20_is_detection_enabled(void);