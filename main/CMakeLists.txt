idf_component_register(
    SRCS 
        main.c
        ros_manager.c
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash
        esp_wifi
        esp_event
        micro_ros_espidf_component
)