#pragma once

#include <stdint.h>

namespace Config {
    constexpr float DT = 0.002f;
    constexpr uint32_t LOOP_RATE_HZ = 500;
    constexpr uint32_t LOOP_INTERVAL_US = 2000;
    
    namespace Pins {
        constexpr uint8_t LED = 13;
        constexpr uint8_t MOTOR1 = 2;
        constexpr uint8_t MOTOR2 = 3;
        constexpr uint8_t MOTOR3 = 4;
        constexpr uint8_t MOTOR4 = 5;
        constexpr uint8_t RC_PPM = 6;
        constexpr uint8_t I2C_SDA = 18;
        constexpr uint8_t I2C_SCL = 19;
    }
    
    namespace PID {
        constexpr float ROLL_KP = 4.5f;
        constexpr float ROLL_KI = 0.03f;
        constexpr float ROLL_KD = 0.3f;
        constexpr float PITCH_KP = 4.5f;
        constexpr float PITCH_KI = 0.03f;
        constexpr float PITCH_KD = 0.3f;
        constexpr float YAW_KP = 6.0f;
        constexpr float YAW_KI = 0.05f;
        constexpr float YAW_KD = 0.0f;
        constexpr float I_MAX = 100.0f;
    }
    
    namespace Limits {
        constexpr float MAX_ROLL = 45.0f;
        constexpr float MAX_PITCH = 45.0f;
        constexpr float MAX_YAW_RATE = 180.0f;
        constexpr uint16_t MOTOR_MIN = 1000;
        constexpr uint16_t MOTOR_MAX = 2000;
        constexpr uint16_t MOTOR_IDLE = 1100;
    }
}
