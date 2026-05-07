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

static ina3221_t dev;
static bool initialized = false;
static SemaphoreHandle_t i2c_mutex = NULL;
static int error_count = 0;
static float filtered_current_7v8 = 0;
static float filtered_current_5v = 0;
static float alpha = 0.13f;  // EMA coefficient (matches your STM32)

void ina3221_set_alpha(float new_alpha)
{
    alpha = new_alpha;
}

void ina3221_update_filtered_current(void)
{
    // This now returns mA, not Amps
    float raw_current_ma = ina3221_get_current_ma(INA3221_CH_7V8);
    
    // Apply EMA filter
    filtered_current_7v8 = (alpha * raw_current_ma) + ((1.0f - alpha) * filtered_current_7v8);
    
    ESP_LOGD(TAG, "Filtered current: %.0f mA (raw: %.0f mA)", filtered_current_7v8, raw_current_ma);
}

float ina3221_get_filtered_current_ma(uint8_t channel)
{
    if (channel == INA3221_CH_7V8) {
        return filtered_current_7v8;  // Already in mA
    } else {
        return ina3221_get_current_ma(channel);  // Now returns mA
    }
}

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
    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, INA3221_CHANNEL_1, OVERCURRENT_7V8_MA * 1000));  // Convert A to mA
    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, INA3221_CHANNEL_2, OVERCURRENT_5V_MA * 1000));   // Convert A to mA
    
    i2c_mutex = xSemaphoreCreateMutex();
    initialized = true;
    
    ESP_LOGI(TAG, "INA3221 initialized successfully");
}

float ina3221_get_current_ma(uint8_t channel)
{
    if (!initialized) return 0.0f;
    
    float current_amps = 0.0f;
    int ch_index;
    
    if (channel == INA3221_CH_5V) {
        ch_index = INA3221_CHANNEL_2;
    } else if (channel == INA3221_CH_7V8) {
        ch_index = INA3221_CHANNEL_1;
    } else {
        return 0.0f;
    }
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
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