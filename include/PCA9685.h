#pragma once

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

class PCA9685 {
private:
    static constexpr const char* TAG = "PCA9685";
    i2c_port_t i2c_port;
    uint8_t address;

    void writeRegister(uint8_t reg, uint8_t data) {
        uint8_t buf[2] = {reg, data};
        i2c_master_write_to_device(i2c_port, address, buf, sizeof(buf), pdMS_TO_TICKS(10));
    }

    uint8_t readRegister(uint8_t reg) {
        uint8_t data = 0;
        i2c_master_write_read_device(i2c_port, address, &reg, 1, &data, 1, pdMS_TO_TICKS(10));
        return data;
    }

public:
    PCA9685(i2c_port_t port = I2C_NUM_0, uint8_t addr = 0x40) : i2c_port(port), address(addr) {}

    bool begin(int sda_pin = 21, int scl_pin = 22, uint32_t clk_speed = 400000, uint32_t osc_freq = 25000000) {
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = sda_pin;
        conf.scl_io_num = scl_pin;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = clk_speed;
        
        i2c_param_config(i2c_port, &conf);
        esp_err_t err = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // Ignore if already installed
            ESP_LOGE(TAG, "I2C install failed: %s", esp_err_to_name(err));
            return false;
        }
        
        // Reset PCA9685
        writeRegister(0x00, 0x00); // Mode 1, Normal mode
        vTaskDelay(pdMS_TO_TICKS(10));
        
        return setPWMFreq(100.0f, osc_freq); // Default to 100Hz as agreed
    }

    bool setPWMFreq(float freqHz, uint32_t osc_freq = 25000000) {
        // prescale = round(osc_value / (4096 * update_rate)) - 1
        float prescaleval = (float)osc_freq;
        prescaleval /= 4096.0f;
        prescaleval /= freqHz;
        prescaleval -= 1.0f;
        
        uint8_t prescale = std::floor(prescaleval + 0.5f);
        
        uint8_t oldmode = readRegister(0x00);
        uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
        writeRegister(0x00, newmode); // go to sleep
        writeRegister(0xFE, prescale); // set the prescaler
        writeRegister(0x00, oldmode);
        vTaskDelay(pdMS_TO_TICKS(5));
        writeRegister(0x00, oldmode | 0xa0); // This sets the MODE1 register to turn on auto increment.
        
        ESP_LOGI(TAG, "Set PWM Frequency to %.1f Hz (Prescale 0x%02X)", freqHz, prescale);
        return true;
    }

    void setPWM(uint8_t num, uint16_t on, uint16_t off) {
        // Send sequential writes using auto increment (faster)
        uint8_t buf[5];
        buf[0] = 0x06 + 4 * num;  // Register address
        buf[1] = on & 0xFF;       // ON LSB
        buf[2] = on >> 8;         // ON MSB
        buf[3] = off & 0xFF;      // OFF LSB
        buf[4] = off >> 8;        // OFF MSB
        i2c_master_write_to_device(i2c_port, address, buf, sizeof(buf), pdMS_TO_TICKS(10));
    }
};
