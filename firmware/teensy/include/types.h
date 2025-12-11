#pragma once

#include <stdint.h>

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
};

struct Quaternion {
    float w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    void normalize();
    Vec3 toEuler() const;
};

struct IMUData {
    Vec3 accel;
    Vec3 gyro;
    Vec3 mag;
    float temp;
    uint32_t timestamp;
    bool valid;
};

struct GPSData {
    double lat, lon;
    float alt;
    float speed;
    float course;
    uint8_t satellites;
    uint8_t fix;
    uint32_t timestamp;
    bool valid;
};

struct BaroData {
    float pressure;
    float temperature;
    float altitude;
    uint32_t timestamp;
    bool valid;
};

struct AttitudeState {
    float roll, pitch, yaw;
    Vec3 rates;
    Quaternion orientation;
};

struct PositionState {
    double lat, lon;
    float altMSL;
    float altAGL;
    Vec3 velocity;
    Vec3 position;
};

struct FlightState {
    AttitudeState attitude;
    PositionState position;
    uint8_t flightMode;
    bool armed;
    uint32_t timestamp;
};
