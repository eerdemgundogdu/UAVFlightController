/*
 * Sensor Fusion Algorithms
 * Madgwick AHRS and Complementary Filter
 */

#include "sensor_fusion.h"
#include <math.h>
#include <string.h>

#define ARM_DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)

static inline float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Quaternion::normalize() {
    float n = invSqrt(w*w + x*x + y*y + z*z);
    w *= n;
    x *= n;
    y *= n;
    z *= n;
}

Vec3 Quaternion::toEuler() const {
    Vec3 euler;
    
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler.x = atan2f(sinr_cosp, cosr_cosp);
    
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        euler.y = copysignf(1.5707963f, sinp);
    } else {
        euler.y = asinf(sinp);
    }
    
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler.z = atan2f(siny_cosp, cosy_cosp);
    
    return euler;
}

void AHRS::begin() {
    q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    memset(&attitude, 0, sizeof(attitude));
    beta = 0.04f;
}

void AHRS::update(const IMUData& imu, float dt) {
    float gx = imu.gyro.x;
    float gy = imu.gyro.y;
    float gz = imu.gyro.z;
    float ax = imu.accel.x;
    float ay = imu.accel.y;
    float az = imu.accel.z;
    
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    float _2qw = 2.0f * qw;
    float _2qx = 2.0f * qx;
    float _2qy = 2.0f * qy;
    float _2qz = 2.0f * qz;
    float _4qw = 4.0f * qw;
    float _4qx = 4.0f * qx;
    float _4qy = 4.0f * qy;
    float _8qx = 8.0f * qx;
    float _8qy = 8.0f * qy;
    float qwqw = qw * qw;
    float qxqx = qx * qx;
    float qyqy = qy * qy;
    float qzqz = qz * qz;
    
    float s0 = _4qw * qyqy + _2qy * ax + _4qw * qxqx - _2qx * ay;
    float s1 = _4qx * qzqz - _2qz * ax + 4.0f * qwqw * qx - _2qw * ay - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * az;
    float s2 = 4.0f * qwqw * qy + _2qw * ax + _4qy * qzqz - _2qz * ay - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * az;
    float s3 = 4.0f * qxqx * qz - _2qx * ax + 4.0f * qyqy * qz - _2qy * ay;
    
    norm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    
    float qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz) - beta * s0;
    float qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy) - beta * s1;
    float qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx) - beta * s2;
    float qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx) - beta * s3;
    
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;
    
    q.normalize();
    
    Vec3 euler = q.toEuler();
    attitude.roll = euler.x * (180.0f / 3.14159265f);
    attitude.pitch = euler.y * (180.0f / 3.14159265f);
    attitude.yaw = euler.z * (180.0f / 3.14159265f);
    attitude.rates.x = gx * (180.0f / 3.14159265f);
    attitude.rates.y = gy * (180.0f / 3.14159265f);
    attitude.rates.z = gz * (180.0f / 3.14159265f);
    attitude.orientation = q;
}

void AHRS::updateWithMag(const IMUData& imu, float dt) {
    update(imu, dt);
    
    if (imu.mag.x == 0 && imu.mag.y == 0 && imu.mag.z == 0) return;
    
    float mx = imu.mag.x, my = imu.mag.y, mz = imu.mag.z;
    float norm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= norm;
    my *= norm;
    mz *= norm;
    
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    
    float hx = 2.0f * (mx * (0.5f - q2*q2 - q3*q3) + my * (q1*q2 - q0*q3) + mz * (q1*q3 + q0*q2));
    float hy = 2.0f * (mx * (q1*q2 + q0*q3) + my * (0.5f - q1*q1 - q3*q3) + mz * (q2*q3 - q0*q1));
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f * (mx * (q1*q3 - q0*q2) + my * (q2*q3 + q0*q1) + mz * (0.5f - q1*q1 - q2*q2));
    
    float magYaw = atan2f(hy, hx);
    attitude.yaw = magYaw * (180.0f / 3.14159265f);
}

void EKF::begin() {
    memset(&position, 0, sizeof(position));
    initCovariance();
}

void EKF::initCovariance() {
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] = (i == j) ? 10.0f : 0.0f;
            Q[i][j] = 0.0f;
        }
    }
    
    Q[0][0] = Q[1][1] = Q[2][2] = 0.1f;
    Q[3][3] = Q[4][4] = Q[5][5] = 0.01f;
    Q[6][6] = Q[7][7] = Q[8][8] = 0.001f;
    
    R_gps[0] = R_gps[1] = 5.0f;
    R_gps[2] = 10.0f;
    R_gps[3] = R_gps[4] = R_gps[5] = 0.5f;
    R_baro = 2.0f;
}

void EKF::predict(const IMUData& imu, float dt) {
    float ax = imu.accel.x;
    float ay = imu.accel.y;
    float az = imu.accel.z - 9.81f;
    
    position.velocity.x += ax * dt;
    position.velocity.y += ay * dt;
    position.velocity.z += az * dt;
    
    position.position.x += position.velocity.x * dt + 0.5f * ax * dt * dt;
    position.position.y += position.velocity.y * dt + 0.5f * ay * dt * dt;
    position.position.z += position.velocity.z * dt + 0.5f * az * dt * dt;
    
    for (int i = 0; i < 6; i++) {
        P[i][i] += Q[i][i] * dt;
    }
}

void EKF::updateGPS(const GPSData& gps) {
    if (!gps.valid || gps.fix < 2) return;
    
    const double R_EARTH = 6378137.0;
    
    float dx = (gps.lat - position.lat) * R_EARTH * (3.14159265 / 180.0);
    float dy = (gps.lon - position.lon) * R_EARTH * (3.14159265 / 180.0) * 
               cos(gps.lat * 3.14159265 / 180.0);
    
    float K = P[0][0] / (P[0][0] + R_gps[0]);
    position.position.x += K * dx;
    P[0][0] *= (1.0f - K);
    
    K = P[1][1] / (P[1][1] + R_gps[1]);
    position.position.y += K * dy;
    P[1][1] *= (1.0f - K);
    
    K = P[2][2] / (P[2][2] + R_gps[2]);
    position.altMSL += K * (gps.alt - position.altMSL);
    P[2][2] *= (1.0f - K);
    
    position.lat = gps.lat;
    position.lon = gps.lon;
}

void EKF::updateBaro(const BaroData& baro) {
    if (!baro.valid) return;
    
    float K = P[2][2] / (P[2][2] + R_baro);
    position.altAGL += K * (baro.altitude - position.altAGL);
    P[2][2] *= (1.0f - K);
}
