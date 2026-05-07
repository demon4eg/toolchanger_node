#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// External variables for state machine to use
extern uint8_t ros_last_command;
extern uint16_t ros_last_tool_id;
extern bool ros_command_received;

// Publish tool status (called by state machine)
void ros_publish_status(uint8_t command, uint16_t tool_id, uint8_t state, uint8_t error_code, float temperature);

// Initialize ROS (creates tasks)
void ros_manager_init(void);
bool ros_manager_is_connected(void);

#endif