#pragma once

#include "PCA9685.h"
#include "quadruped_types.h"
#include "servo_config.h"
#include <algorithm>
#include <cmath>

class HardwareOutput {
private:
    PCA9685 driver;
    
    // Limits
    const float min_angle_deg = 0.0f;
    const float max_angle_deg = 180.0f;
    
    // PWM Pulse bounds (20kg standard servo: 500us ~ 2500us for 0 ~ 180 degrees)
    const float pwm_min_us = 500.0f;
    const float pwm_max_us = 2500.0f;
    const float angle_range_deg = 180.0f;

    // Rate Limiting (4.65 rad/s)
    const float max_rate_rad_per_s = 4.65f;
    const float max_delta_rad = max_rate_rad_per_s * CONTROL_DT_S; // Maximum change per frame (e.g. 0.0465 rad at 100Hz)

    // Deadband (0.5 degrees)
    const float deadband_rad = 0.5f * (M_PI / 180.0f); 
    

    
    // Direction Reversal
    // 1.0 = normal (CCW increases angle), -1.0 = flipped (CW increases angle)
    // Left/Right symmetry means opposite sides operate reversed.
    // Must be verified and adjusted during physical calibration!
    float motor_dir[4][3] = {
        { 1.0f,  1.0f,  1.0f}, // LF
        {-1.0f, -1.0f, -1.0f}, // RF
        {-1.0f,  1.0f,  1.0f}, // LH (HAA: YZ 평면 대칭 장착으로 반전)
        { 1.0f, -1.0f, -1.0f}  // RH (HAA: YZ 평면 대칭 장착으로 반전)
    };
    
    // Mount Offset (rad) - Physical servo angle when math angle = 0.
    // HAA : 90°  — symmetric ±90° global range
    // HFE : 120° — 30° backward bias; trot standing (θ=-60°) → servo 60°, bowing range secured
    // KFE : 180° — 90° perpendicular assembly; singularity (full extension) mapped to servo 180° → clamped to 175°
    float mount_offset_rad[4][3] = {
        {M_PI/2, 2.0f*M_PI/3.0f, M_PI},  // LF [HAA=90°, HFE=120°, KFE=180°]
        {M_PI/2, 2.0f*M_PI/3.0f, M_PI},  // RF
        {M_PI/2, 2.0f*M_PI/3.0f, M_PI},  // LH
        {M_PI/2, 2.0f*M_PI/3.0f, M_PI}   // RH
    };

    float prev_angles_rad[4][3] = {{0}};
    bool first_update = true;

public:
    HardwareOutput() {}

    bool init() {
        bool ok = driver.begin(21, 22, 400000, PCA9685_OSC_FREQ);
        if (ok) {
            ESP_LOGI("HardwareOutput", "PCA9685 initialized. OSC_FREQ: %u Hz", PCA9685_OSC_FREQ);
        }
        return ok;
    }

    void setBoardFreq(uint32_t freq) {
        driver.setPWMFreq(100.0f, freq);
        ESP_LOGI("HardwareOutput", "Live PCA9685 Board Freq Update: %u Hz", freq);
    }

    uint16_t usToTicks(float us) {
        // 100Hz = 1 period takes 10,000 us (10ms).
        // PCA9685 handles 0-4095 ticks per period.
        float ticks = (us / 10000.0f) * 4096.0f;
        return (uint16_t)std::round(ticks);
    }

    // Apply scaling, limits, deadband, and send to motors
    void writeJoints(const float target_math_angles[4][3]) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                float target = target_math_angles[i][j];
                
                if (first_update) {
                    prev_angles_rad[i][j] = target;
                }

                // 1. Rate Limiting
                float delta = target - prev_angles_rad[i][j];
                if (delta > max_delta_rad) target = prev_angles_rad[i][j] + max_delta_rad;
                if (delta < -max_delta_rad) target = prev_angles_rad[i][j] - max_delta_rad;

                // 2. Deadband
                if (std::abs(target - prev_angles_rad[i][j]) < deadband_rad) {
                    target = prev_angles_rad[i][j];
                }
                prev_angles_rad[i][j] = target;

                // 3. Math to Physical Conversion
                // ZERO_OFFSET is in degrees, so we convert it to rad
                float offset_rad = ZERO_OFFSET[i][j] * (M_PI / 180.0f);
                float phys_rad = mount_offset_rad[i][j] + (target * motor_dir[i][j]) + offset_rad;
                
                // 4. Safety Clamping
                float phys_deg = phys_rad * (180.0f / M_PI);
                if (phys_deg < min_angle_deg) phys_deg = min_angle_deg;
                if (phys_deg > max_angle_deg) phys_deg = max_angle_deg;

                // 5. Convert angle to microsecond pulse
                float pulse_us = pwm_min_us + (phys_deg / angle_range_deg) * (pwm_max_us - pwm_min_us);
                
                // 6. Write to I2C via driver
                uint16_t ticks = usToTicks(pulse_us);
                
                // Map legs to 16 channels sequentially:
                // channel 0=LF0, 1=LF1, 2=LF2 / 3=RF0, 4=RF1...
                uint8_t channel = i * 3 + j; 
                driver.setPWM(channel, 0, ticks);
            }
        }
        first_update = false;
    }
    
    // calibration/test mode
    void writeCalibrationDeg(uint8_t channel, float angle_deg) {
        if (angle_deg < min_angle_deg) angle_deg = min_angle_deg;
        if (angle_deg > max_angle_deg) angle_deg = max_angle_deg;
        
        float pulse_us = pwm_min_us + (angle_deg / angle_range_deg) * (pwm_max_us - pwm_min_us);
        uint16_t ticks = usToTicks(pulse_us);
        driver.setPWM(channel, 0, ticks);
        
        ESP_LOGI("HardwareOutput", "Calibration CH %d -> %.1f deg (%.0f us, %d ticks)", channel, angle_deg, pulse_us, ticks);
    }
};
