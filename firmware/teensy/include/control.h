#pragma once

#include "config.h"

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float imax);
    float compute(float error, float dt);
    void reset();
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    
private:
    float kp, ki, kd;
    float imax;
    float integral = 0;
    float prevError = 0;
    float outMin = -1000;
    float outMax = 1000;
};

class AttitudeController {
public:
    void begin();
    void update(const float roll, const float pitch, const float yaw,
                const float rollRate, const float pitchRate, const float yawRate,
                float dt);
    void setTargetAttitude(float roll, float pitch, float yaw);
    void setTargetRates(float rollRate, float pitchRate, float yawRate);
    void getMotorOutputs(float& m1, float& m2, float& m3, float& m4) const;
    void setThrottle(float throttle);
    
private:
    PIDController rollPID{Config::PID::ROLL_KP, Config::PID::ROLL_KI, 
                          Config::PID::ROLL_KD, Config::PID::I_MAX};
    PIDController pitchPID{Config::PID::PITCH_KP, Config::PID::PITCH_KI,
                           Config::PID::PITCH_KD, Config::PID::I_MAX};
    PIDController yawPID{Config::PID::YAW_KP, Config::PID::YAW_KI,
                         Config::PID::YAW_KD, Config::PID::I_MAX};
    
    PIDController rollRatePID{0.15f, 0.002f, 0.003f, 50.0f};
    PIDController pitchRatePID{0.15f, 0.002f, 0.003f, 50.0f};
    PIDController yawRatePID{0.25f, 0.005f, 0.0f, 50.0f};
    
    float targetRoll = 0, targetPitch = 0, targetYaw = 0;
    float throttle = 0;
    float motor[4] = {0, 0, 0, 0};
    
    void mixMotors(float roll, float pitch, float yaw, float throttle);
};
