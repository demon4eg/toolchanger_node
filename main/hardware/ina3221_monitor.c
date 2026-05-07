#include "ina3221_monitor.h"
#include "ina3221.h"
#include "i2cdev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#define TAG "INA3221"

// I2C configuration
#define I2C_PORT            0
#define INA3221_I2C_ADDR    0x40
#define I2C_SDA_PIN         2
#define I2C_SCL_PIN         1

// Shunt resistors (milliohms) - from your example
#define SHUNT_7V8_MILLI_OHM     100    
#define SHUNT_5V_MILLI_OHM      100 
#define SHUNT_UNUSED_MILLI_OHM  100

// Warning thresholds (Amps) - from your example
#define WARNING_CURRENT_7V8_A   1.5f   // 2.5A
#define WARNING_CURRENT_5V_A    0.8f   // 1.0A

static ina3221_t dev;
static bool initialized = false;
static SemaphoreHandle_t i2c_mutex = NULL;
static int error_count = 0;

void ina3221_monitor_init(void)
{
    ESP_LOGI(TAG, "Initializing INA3221...");
    
    // Configure INA3221 with shunt resistors (milliohms)
    dev = (ina3221_t){
        .shunt = {
            SHUNT_7V8_MILLI_OHM,      // CH1 - 7.8V rail (index 0)
            SHUNT_5V_MILLI_OHM,       // CH2 - 5V rail (index 1)
            SHUNT_UNUSED_MILLI_OHM    // CH3 - unused (index 2)
        },
        .config.config_register = INA3221_DEFAULT_CONFIG,
        .mask.mask_register = INA3221_DEFAULT_MASK
    };
    
    memset(&dev.i2c_dev, 0, sizeof(i2c_dev_t));
    
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina3221_init_desc(&dev, INA3221_I2C_ADDR, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN));
    
    // Configure measurement settings (from your example)
    ESP_ERROR_CHECK(ina3221_set_options(&dev, true, true, true));  // Continuous mode
    ESP_ERROR_CHECK(ina3221_enable_channel(&dev, true, true, true)); // Enable all channels
    ESP_ERROR_CHECK(ina3221_set_average(&dev, INA3221_AVG_64));
    ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&dev, INA3221_CT_2116));
    ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&dev, INA3221_CT_2116));
    
    // Set warning thresholds (Amps)
    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, INA3221_CHANNEL_1, WARNING_CURRENT_7V8_A));
    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, INA3221_CHANNEL_2, WARNING_CURRENT_5V_A));
    
    i2c_mutex = xSemaphoreCreateMutex();
    initialized = true;
    
    ESP_LOGI(TAG, "INA3221 initialized successfully");
}

float ina3221_get_current_ma(uint8_t channel)
{
    if (!initialized) return 0.0f;
    
    float current_amps = 0.0f;
    int ch_index;
    
    // Convert channel to INA3221 channel index
    if (channel == INA3221_CH_5V) {
        ch_index = INA3221_CHANNEL_2;  // CH2 for 5V rail
    } else if (channel == INA3221_CH_7V8) {
        ch_index = INA3221_CHANNEL_1;  // CH1 for 7.8V rail
    } else {
        return 0.0f;
    }
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        float shunt_voltage_mv, current_amps;
        esp_err_t ret = ina3221_get_shunt_value(&dev, ch_index, &shunt_voltage_mv, &current_amps);
        xSemaphoreGive(i2c_mutex);
        
        if (ret != ESP_OK) {
            error_count++;
            return 0.0f;
        }
        
        return current_amps;
    }
    
    return 0.0f;
}

float ina3221_get_voltage_mv(uint8_t channel)
{
    if (!initialized) return 0.0f;
    
    float voltage_volts = 0.0f;
    int ch_index;
    
    if (channel == INA3221_CH_5V) {
        ch_index = INA3221_CHANNEL_2;
    } else if (channel == INA3221_CH_7V8) {
        ch_index = INA3221_CHANNEL_1;
    } else {
        return 0.0f;
    }
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = ina3221_get_bus_voltage(&dev, ch_index, &voltage_volts);
        xSemaphoreGive(i2c_mutex);
        
        if (ret != ESP_OK) {
            return 0.0f;
        }
        
        return voltage_volts * 1000.0f;  // Convert to mV
    }
    
    return 0.0f;
}

bool ina3221_is_overcurrent(uint8_t channel)
{
    float current = ina3221_get_current_ma(channel);
    
    if (channel == INA3221_CH_5V) {
        return current > OVERCURRENT_5V_MA;
    } else {
        return current > OVERCURRENT_7V8_MA;
    }
}