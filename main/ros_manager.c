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

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define TAG "ROS_MANAGER"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed on line %d: %ld", __LINE__, (long)temp_rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft fail on line %d: %ld", __LINE__, (long)temp_rc); }}

static rcl_publisher_t publisher_1;
static rcl_publisher_t publisher_2;

static void thread_1(void *arg)
{
    std_msgs__msg__Int32 msg;
    msg.data = 0;
    while(1){
        RCSOFTCHECK(rcl_publish(&publisher_1, &msg, NULL));
        ESP_LOGI(TAG, "Publisher 1: %ld", msg.data);
        msg.data++;
        usleep(1000000);
    }
}

static void thread_2(void *arg)
{
    std_msgs__msg__Int32 msg;
    msg.data = 0;
    while(1){
        RCSOFTCHECK(rcl_publish(&publisher_2, &msg, NULL));
        ESP_LOGI(TAG, "Publisher 2: %ld", msg.data);
        msg.data--;
        usleep(500000);
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

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "multithread_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "multithread_publisher_1"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "multithread_publisher_2"));

    xTaskCreate(thread_1, "thread_1", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(thread_2, "thread_2", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO + 1, NULL);

    while(1){
        sleep(100);
    }

    RCCHECK(rcl_publisher_fini(&publisher_1, &node));
    RCCHECK(rcl_publisher_fini(&publisher_2, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void ros_manager_init(void)
{
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}