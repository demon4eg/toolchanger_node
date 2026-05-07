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
#include "freertos/portmacro.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define TAG "ROS_MANAGER"

// Mutex for shared variables accessed from callback and main task
portMUX_TYPE ros_spinlock = portMUX_INITIALIZER_UNLOCKED;

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
    
    portENTER_CRITICAL(&ros_spinlock);
    ros_last_command = msg->command;
    ros_last_tool_id = msg->tool_id;
    ros_command_received = true;
    portEXIT_CRITICAL(&ros_spinlock);
    
    ESP_LOGI(TAG, "Raw cmd: %d, Raw ID: %d (0x%X)", 
             ros_last_command, ros_last_tool_id, ros_last_tool_id);
}

void ros_publish_status(uint8_t command, uint16_t tool_id, uint8_t state, uint8_t error_code, float temperature)
{
    manipulator_6dof_interfaces__msg__ToolChanger msg;
    manipulator_6dof_interfaces__msg__ToolChanger__init(&msg);
    msg.command = command;
    msg.tool_id = tool_id;
    msg.state = state;
    msg.error_code = error_code;
    msg.temperature = temperature;
    
    rcl_ret_t ret = rcl_publish(&status_pub, &msg, NULL);
    // if (ret != RCL_RET_OK) {
    //     ESP_LOGE(TAG, "rcl_publish failed: %ld", (long)ret);
    // } else {
    //     ESP_LOGI(TAG, "rcl_publish SUCCESS");
    // }
}

static bool init_micro_ros(rclc_support_t *support, rcl_node_t *node, 
                           rcl_publisher_t *heartbeat_pub, rcl_publisher_t *status_pub,
                           rcl_subscription_t *cmd_sub, rclc_executor_t *executor,
                           rcl_allocator_t *allocator)
{
    rcl_ret_t rc;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    
    rc = rcl_init_options_init(&init_options, *allocator);
    if (rc != RCL_RET_OK) return false;

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rc = rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, 
                                           CONFIG_MICRO_ROS_AGENT_PORT, 
                                           rmw_options);
    if (rc != RMW_RET_OK) {
        rcl_init_options_fini(&init_options);
        return false;
    }
    #endif

    // Initialize support
    rc = rclc_support_init_with_options(support, 0, NULL, &init_options, allocator);
    if (rc != RCL_RET_OK) {
        rcl_init_options_fini(&init_options);
        return false;
    }

    // Create node
    rc = rclc_node_init_default(node, "toolchanger_node", "", support);
    if (rc != RCL_RET_OK) {
        rclc_support_fini(support);
        rcl_init_options_fini(&init_options);
        return false;
    }

    // Create publishers
    rc = rclc_publisher_init_default(heartbeat_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/toolchanger/heartbeat");
    if (rc != RCL_RET_OK) {
        rcl_node_fini(node);
        rclc_support_fini(support);
        rcl_init_options_fini(&init_options);
        return false;
    }

    rc = rclc_publisher_init_default(status_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, ToolChanger),
        "/toolchanger/status");
    if (rc != RCL_RET_OK) {
        rcl_publisher_fini(heartbeat_pub, node);
        rcl_node_fini(node);
        rclc_support_fini(support);
        rcl_init_options_fini(&init_options);
        return false;
    }

    // Create subscriber
    rc = rclc_subscription_init_default(cmd_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, ToolChanger),
        "/toolchanger/command");
    if (rc != RCL_RET_OK) {
        rcl_publisher_fini(status_pub, node);
        rcl_publisher_fini(heartbeat_pub, node);
        rcl_node_fini(node);
        rclc_support_fini(support);
        rcl_init_options_fini(&init_options);
        return false;
    }

    // Initialize executor
    extern manipulator_6dof_interfaces__msg__ToolChanger cmd_msg;
    rc = rclc_executor_init(executor, &support->context, 1, allocator);
    if (rc != RCL_RET_OK) {
        rcl_subscription_fini(cmd_sub, node);
        rcl_publisher_fini(status_pub, node);
        rcl_publisher_fini(heartbeat_pub, node);
        rcl_node_fini(node);
        rclc_support_fini(support);
        rcl_init_options_fini(&init_options);
        return false;
    }

    rc = rclc_executor_add_subscription(executor, cmd_sub, &cmd_msg, command_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        rclc_executor_fini(executor);
        rcl_subscription_fini(cmd_sub, node);
        rcl_publisher_fini(status_pub, node);
        rcl_publisher_fini(heartbeat_pub, node);
        rcl_node_fini(node);
        rclc_support_fini(support);
        rcl_init_options_fini(&init_options);
        return false;
    }

    rcl_init_options_fini(&init_options);
    return true;
}

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_subscription_t cmd_sub;
    rclc_executor_t executor;
    
    while (1) {
        ESP_LOGI(TAG, "Attempting micro-ROS connection...");
        
        if (init_micro_ros(&support, &node, &heartbeat_pub, &status_pub, 
                           &cmd_sub, &executor, &allocator)) {
            ESP_LOGI(TAG, "micro-ROS initialized successfully");
            
            // Create heartbeat task (only once per successful connection)
            static bool heartbeat_task_created = false;
            if (!heartbeat_task_created) {
                xTaskCreate(heartbeat_task, "heartbeat", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                            CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
                heartbeat_task_created = true;
            }
            
            // Main spin loop
            while (1) {
                rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                if (rc != RCL_RET_OK) {
                    ESP_LOGW(TAG, "Executor spin failed: %ld", (long)rc);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            // Cleanup on failure
            ESP_LOGW(TAG, "Cleaning up micro-ROS...");
            rclc_executor_fini(&executor);
            rcl_subscription_fini(&cmd_sub, &node);
            rcl_publisher_fini(&status_pub, &node);
            rcl_publisher_fini(&heartbeat_pub, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
        }
        
        ESP_LOGW(TAG, "micro-ROS init failed, retrying in 2 seconds...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void ros_manager_init(void)
{
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}