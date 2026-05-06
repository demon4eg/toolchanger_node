#ifndef DS18B20_TASK_H
#define DS18B20_TASK_H

#include <stdint.h>
#include <stdbool.h>

void ds18b20_task_start(void);
void ds18b20_task_run(void);
bool ds18b20_is_present(void);
uint8_t ds18b20_get_id(void);
float ds18b20_get_temp(void);

#endif