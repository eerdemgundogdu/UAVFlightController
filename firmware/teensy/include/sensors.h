#pragma once

#include "types.h"

class IMU {
public:
    bool begin();
    bool update();
    IMUData getData() const { return data; }
    void calibrate();
    bool isCalibrated() const { return calibrated; }
    
private:
    IMUData data;
    Vec3 gyroBias;
    Vec3 accelBias;
    bool calibrated = false;
    uint32_t lastUpdate = 0;
};

class GPS {
public:
    bool begin();
    bool update();
    GPSData getData() const { return data; }
    bool hasFix() const { return data.fix >= 2; }
    
private:
    GPSData data;
    void parseNMEA(const char* sentence);
};

class Barometer {
public:
    bool begin();
    bool update();
    BaroData getData() const { return data; }
    void setSeaLevelPressure(float hPa);
    
private:
    BaroData data;
    float seaLevelPressure = 1013.25f;
};
