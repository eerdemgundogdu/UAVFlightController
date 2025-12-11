#pragma once

#include "types.h"
#include "config.h"

class AHRS {
public:
    void begin();
    void update(const IMUData& imu, float dt);
    void updateWithMag(const IMUData& imu, float dt);
    AttitudeState getAttitude() const { return attitude; }
    Quaternion getQuaternion() const { return q; }
    
private:
    Quaternion q;
    AttitudeState attitude;
    float beta = 0.1f;
    
    void madgwickUpdate(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt);
    void madgwickUpdateMag(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz, float dt);
};

class EKF {
public:
    void begin();
    void predict(const IMUData& imu, float dt);
    void updateGPS(const GPSData& gps);
    void updateBaro(const BaroData& baro);
    PositionState getPosition() const { return position; }
    
private:
    PositionState position;
    float P[15][15];
    float Q[15][15];
    float R_gps[6];
    float R_baro;
    
    void initCovariance();
};
