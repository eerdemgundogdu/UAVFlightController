/*
 * UAV Flight Controller - Main Application
 * Bare-Metal 500Hz Control Loop for Teensy 4.1
 */

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "sensors.h"
#include "sensor_fusion.h"
#include "control.h"
#include "mavlink.h"

#define IMXRT_GPT1_BASE     0x401EC000UL
#define IMXRT_GPT2_BASE     0x401F0000UL
#define ARM_DWT_CYCCNT      (*(volatile uint32_t*)0xE0001004)
#define ARM_DWT_CTRL        (*(volatile uint32_t*)0xE0001000)
#define ARM_DEMCR           (*(volatile uint32_t*)0xE000EDFC)

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PR;
    volatile uint32_t SR;
    volatile uint32_t IR;
    volatile uint32_t OCR[3];
    volatile uint32_t ICR[2];
    volatile uint32_t CNT;
} GPT_TypeDef;

#define GPT1    ((GPT_TypeDef*)IMXRT_GPT1_BASE)
#define GPT2    ((GPT_TypeDef*)IMXRT_GPT2_BASE)

IMU imu;
Barometer baro;
GPS gps;
AHRS ahrs;
EKF ekf;
AttitudeController controller;
MAVLinkHandler mavlink;

static volatile uint32_t loop_count = 0;
static volatile uint32_t last_loop_us = 0;
static volatile uint32_t max_loop_us = 0;
static volatile uint32_t imu_errors = 0;
static volatile bool armed = false;
static volatile uint8_t flight_mode = 0;
static volatile uint32_t arm_time = 0;

static float target_roll = 0, target_pitch = 0, target_yaw = 0;
static float throttle_cmd = 0;

static struct {
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float throttle;
    bool failsafe;
    uint32_t last_update;
} rc_input;

static struct {
    float target_roll;
    float target_pitch;
    float target_yaw;
    bool override;
    uint32_t last_update;
} jetson_cmd;

static inline uint32_t micros_fast(void) {
    return ARM_DWT_CYCCNT / 600;
}

static inline void delay_us(uint32_t us) {
    uint32_t start = ARM_DWT_CYCCNT;
    uint32_t cycles = us * 600;
    while ((ARM_DWT_CYCCNT - start) < cycles);
}

static void gpt_init(void) {
    GPT1->CR = 0;
    GPT1->PR = 149;
    GPT1->OCR[0] = 2000;
    GPT1->CR = (1 << 0) | (1 << 6) | (1 << 9);
}

static void process_rc_input(void) {
    rc_input.last_update = micros_fast();
    rc_input.failsafe = false;
}

static void process_jetson_commands(void) {
    while (Serial2.available()) {
        static char buf[128];
        static uint8_t idx = 0;
        
        char c = Serial2.read();
        if (c == '\n') {
            buf[idx] = 0;
            
            if (buf[0] == 'T' && buf[1] == 'G' && buf[2] == 'T') {
                float r, p, y;
                if (sscanf(buf + 4, "%f,%f,%f", &r, &p, &y) == 3) {
                    jetson_cmd.target_roll = r;
                    jetson_cmd.target_pitch = p;
                    jetson_cmd.target_yaw = y;
                    jetson_cmd.override = true;
                    jetson_cmd.last_update = micros_fast();
                }
            } else if (buf[0] == 'A' && buf[1] == 'R' && buf[2] == 'M') {
                armed = true;
                arm_time = millis();
            } else if (buf[0] == 'D' && buf[1] == 'I' && buf[2] == 'S') {
                armed = false;
            } else if (buf[0] == 'M' && buf[1] == 'O' && buf[2] == 'D') {
                flight_mode = buf[4] - '0';
            }
            
            idx = 0;
        } else if (idx < 127) {
            buf[idx++] = c;
        }
    }
    
    if ((micros_fast() - jetson_cmd.last_update) > 500000) {
        jetson_cmd.override = false;
    }
}

static void send_telemetry(void) {
    AttitudeState att = ahrs.getAttitude();
    PositionState pos = ekf.getPosition();
    
    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "TEL:%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.1f,%d,%d\n",
        loop_count,
        att.roll, att.pitch, att.yaw,
        att.rates.x, att.rates.y, att.rates.z,
        pos.lat, pos.lon, pos.altMSL,
        armed ? 1 : 0, flight_mode);
    
    Serial2.write((uint8_t*)buf, len);
}

static void check_safety(void) {
    AttitudeState att = ahrs.getAttitude();
    
    if (fabsf(att.roll) > 60.0f || fabsf(att.pitch) > 60.0f) {
        armed = false;
    }
    
    if (rc_input.failsafe || (micros_fast() - rc_input.last_update) > 1000000) {
        if (armed && flight_mode != 3) {
            flight_mode = 3;
        }
    }
    
    BaroData bd = baro.getData();
    if (bd.valid && bd.altitude < -10.0f) {
        armed = false;
    }
}

static void compute_control(float dt) {
    float roll_target, pitch_target, yaw_target;
    
    if (jetson_cmd.override) {
        roll_target = jetson_cmd.target_roll;
        pitch_target = jetson_cmd.target_pitch;
        yaw_target = jetson_cmd.target_yaw;
    } else {
        roll_target = rc_input.roll_cmd * Config::Limits::MAX_ROLL;
        pitch_target = rc_input.pitch_cmd * Config::Limits::MAX_PITCH;
        yaw_target = rc_input.yaw_cmd * Config::Limits::MAX_YAW_RATE;
    }
    
    controller.setTargetAttitude(roll_target, pitch_target, yaw_target);
    controller.setThrottle(rc_input.throttle);
    
    AttitudeState att = ahrs.getAttitude();
    controller.update(att.roll, att.pitch, att.yaw,
                     att.rates.x, att.rates.y, att.rates.z, dt);
}

void setup() {
    ARM_DEMCR |= (1 << 24);
    ARM_DWT_CTRL |= 1;
    
    Serial.begin(115200);
    Serial1.begin(57600);
    Serial2.begin(115200);
    
    delay(100);
    
    Serial.println("UAV Flight Controller Initializing...");
    
    if (!imu.begin()) {
        Serial.println("ERROR: IMU init failed");
        while (1) {
            digitalToggle(Config::Pins::LED);
            delay(100);
        }
    }
    Serial.println("IMU: OK");
    
    if (!baro.begin()) {
        Serial.println("WARNING: Barometer init failed");
    } else {
        Serial.println("Barometer: OK");
    }
    
    gps.begin();
    Serial.println("GPS: Initialized");
    
    Serial.println("Calibrating IMU (keep level)...");
    imu.calibrate();
    Serial.println("Calibration complete");
    
    ahrs.begin();
    ekf.begin();
    controller.begin();
    mavlink.begin(&Serial1);
    
    gpt_init();
    
    pinMode(Config::Pins::LED, OUTPUT);
    
    rc_input.failsafe = true;
    jetson_cmd.override = false;
    
    Serial.println("System ready - waiting for arm command");
    last_loop_us = micros_fast();
}

void loop() {
    uint32_t now_us = micros_fast();
    uint32_t elapsed = now_us - last_loop_us;
    
    if (elapsed < Config::LOOP_INTERVAL_US) {
        return;
    }
    
    float dt = elapsed / 1000000.0f;
    last_loop_us = now_us;
    
    if (elapsed > max_loop_us) max_loop_us = elapsed;
    
    if (!imu.update()) {
        imu_errors++;
        return;
    }
    
    if ((loop_count % 5) == 0) {
        baro.update();
    }
    
    if ((loop_count % 10) == 0) {
        gps.update();
    }
    
    IMUData imu_data = imu.getData();
    ahrs.update(imu_data, dt);
    ekf.predict(imu_data, dt);
    
    if ((loop_count % 5) == 0) {
        ekf.updateBaro(baro.getData());
    }
    if ((loop_count % 10) == 0) {
        ekf.updateGPS(gps.getData());
    }
    
    process_rc_input();
    process_jetson_commands();
    
    check_safety();
    
    if (armed) {
        compute_control(dt);
        
        float m1, m2, m3, m4;
        controller.getMotorOutputs(m1, m2, m3, m4);
    }
    
    if ((loop_count % 25) == 0) {
        AttitudeState att = ahrs.getAttitude();
        mavlink.sendAttitude(att.roll, att.pitch, att.yaw,
                            att.rates.x, att.rates.y, att.rates.z);
    }
    
    if ((loop_count % 50) == 0) {
        send_telemetry();
    }
    
    if ((loop_count % 500) == 0) {
        mavlink.sendHeartbeat(flight_mode, armed ? 4 : 3);
        digitalToggle(Config::Pins::LED);
        
        if (loop_count % 5000 == 0) {
            Serial.printf("Loop: %lu, MaxDt: %lu us, IMU Err: %lu\n",
                         loop_count, max_loop_us, imu_errors);
            max_loop_us = 0;
        }
    }
    
    mavlink.update();
    
    loop_count++;
}
