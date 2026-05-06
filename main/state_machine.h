#pragma once

#include <stdint.h>
#include <stdbool.h>

void state_machine_init(void);
void state_machine_task(void *arg);
void state_machine_button_unlock(void);  // For physical button interrupt