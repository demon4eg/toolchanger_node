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

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define TAG "ROS_MANAGER"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed on line %d: %ld", __LINE__, (long)temp_rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft fail on line %d: %ld", __LINE__, (long)temp_rc); }}

static rcl_publisher_t publisher_1;
static rcl_publisher_t tool_status_pub;
static rcl_subscription_t tool_cmd_sub;
static rclc_executor_t executor;
static rcl_node_t node;
static manipulator_6dof_interfaces__msg__ToolChanger tool_cmd_msg;

// Heartbeat publisher (1Hz)
static void heartbeat_task(void *arg)
{
    std_msgs__msg__Int32 msg;
    msg.data = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);
    
    while(1){
        RCSOFTCHECK(rcl_publish(&publisher_1, &msg, NULL));
        ESP_LOGI(TAG, "Heartbeat: %ld", msg.data);
        msg.data++;
        vTaskDelayUntil(&last_wake, period);
    }
}

// Tool status publisher (5Hz)
static void tool_status_task(void *arg)
{
    manipulator_6dof_interfaces__msg__ToolChanger msg;
    manipulator_6dof_interfaces__msg__ToolChanger__init(&msg);
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(200); // 5Hz
    
    while(1){
        // Fill custom message with DS18B20 data
        msg.command = 2;  // status_request
        msg.tool_id = ds18b20_get_id();
        msg.state = ds18b20_is_present() ? 2 : 0;  // 2=attached, 0=idle
        msg.error_code = 0;
        msg.temperature = ds18b20_get_temp();
        
        RCSOFTCHECK(rcl_publish(&tool_status_pub, &msg, NULL));
        ESP_LOGD(TAG, "Tool status: ID=%d, State=%d, Temp=%.1f", 
                 msg.tool_id, msg.state, msg.temperature);
        
        vTaskDelayUntil(&last_wake, period);
    }
}

// Tool command subscriber callback
static void tool_command_callback(const void *msgin)
{
    const manipulator_6dof_interfaces__msg__ToolChanger *msg = 
        (const manipulator_6dof_interfaces__msg__ToolChanger *)msgin;
    
    ESP_LOGI(TAG, "Received command: %d, Tool ID: %d", msg->command, msg->tool_id);
    
    if (msg->command == 1) {
        ESP_LOGI(TAG, "Attach tool %d", msg->tool_id);
        // TODO: Start attach sequence
    } else if (msg->command == 0) {
        ESP_LOGI(TAG, "Release tool");
        // TODO: Start release sequence
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
    RCCHECK(rclc_publisher_init_default(
        &publisher_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/toolchanger/heartbeat"));

    // Initialize tool status publisher (custom message)
    RCCHECK(rclc_publisher_init_default(
        &tool_status_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, ToolChanger),
        "/toolchanger/status"));

    // Initialize tool command subscriber (custom message)
    RCCHECK(rclc_subscription_init_default(
        &tool_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, ToolChanger),
        "/toolchanger/command"));

    // Initialize executor with 1 subscription
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &tool_cmd_sub, &tool_cmd_msg,
                                       &tool_command_callback, ON_NEW_DATA));

    // Create tasks
    xTaskCreate(heartbeat_task, "heartbeat", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(tool_status_task, "tool_status", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                CONFIG_MICRO_ROS_APP_TASK_PRIO + 1, NULL);

    // Main loop - spin executor
    while(1){
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&publisher_1, &node));
    RCCHECK(rcl_publisher_fini(&tool_status_pub, &node));
    RCCHECK(rcl_subscription_fini(&tool_cmd_sub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void ros_manager_init(void)
{
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}