#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <manipulator_6dof_interfaces/msg/tool_changer.h>
#include "ds18b20_task.h"
#include "ros_manager.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define TAG "ROS_MANAGER"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed on line %d: %ld", __LINE__, (long)temp_rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft fail on line %d: %ld", __LINE__, (long)temp_rc); }}

// External variables for state machine
uint8_t ros_last_command = 0;
uint16_t ros_last_tool_id = 0;
bool ros_command_received = false;

static rcl_publisher_t heartbeat_pub;
static rcl_publisher_t status_pub;
static rcl_subscription_t cmd_sub;
static manipulator_6dof_interfaces__msg__ToolChanger cmd_msg;
static rclc_executor_t executor;
static rcl_node_t node;

// Heartbeat counter
static int heartbeat_counter = 0;

// Heartbeat publisher (1Hz)
static void heartbeat_task(void *arg)
{
    std_msgs__msg__Int32 msg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);
    
    while(1){
        msg.data = heartbeat_counter++;
        rcl_publish(&heartbeat_pub, &msg, NULL);
        ESP_LOGD(TAG, "Heartbeat: %ld", msg.data);
        vTaskDelayUntil(&last_wake, period);
    }
}

// Tool command subscriber callback
static void command_callback(const void *msgin)
{
    const manipulator_6dof_interfaces__msg__ToolChanger *msg = msgin;
    ros_last_command = msg->command;
    ros_last_tool_id = msg->tool_id;
    ros_command_received = true;
    ESP_LOGI(TAG, "Raw cmd: %d, Raw ID: %d (0x%X)", 
             ros_last_command, ros_last_tool_id, ros_last_tool_id);
}

void ros_publish_status(uint16_t tool_id, uint8_t state, uint8_t error_code, float temperature)
{
    manipulator_6dof_interfaces__msg__ToolChanger msg;
    manipulator_6dof_interfaces__msg__ToolChanger__init(&msg);
    msg.command = 2;
    msg.tool_id = tool_id;
    msg.state = state;
    msg.error_code = error_code;
    msg.temperature = temperature;
    rcl_publish(&status_pub, &msg, NULL);
}

static void status_task(void *arg)
{
    while(1) {
        // Status published by state machine via ros_publish_status
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "toolchanger_node", "", &support));
    ESP_LOGI(TAG, "Node created: toolchanger_node");

    // Initialize heartbeat publisher
    RCCHECK(rclc_publisher_init_default(&heartbeat_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/toolchanger/heartbeat"));

    // Initialize tool status publisher
    RCCHECK(rclc_publisher_init_default(&status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, ToolChanger),
        "/toolchanger/status"));

    // Initialize tool command subscriber
    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, ToolChanger),
        "/toolchanger/command"));

    // Initialize executor with 1 subscription
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                           command_callback, ON_NEW_DATA));

    // Create tasks
    xTaskCreate(heartbeat_task, "heartbeat", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(status_task, "status", 2048, NULL, 1, NULL);

    // Main loop - spin executor
    while(1){
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ros_manager_init(void)
{
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}