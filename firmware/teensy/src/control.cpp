/*
 * Flight Control PID and Motor Mixing
 * Optimized for 500Hz control loop
 */

#include "control.h"
#include <math.h>
#include <string.h>

#define FLEXPWM1_BASE   0x403DC000UL
#define IOMUXC_BASE     0x401F8000UL
#define CCM_BASE        0x400FC000UL

typedef struct {
    volatile uint16_t CNT;
    volatile uint16_t INIT;
    volatile uint16_t CTRL2;
    volatile uint16_t CTRL;
    uint16_t reserved0;
    volatile uint16_t VAL0;
    volatile uint16_t FRACVAL1;
    volatile uint16_t VAL1;
    volatile uint16_t FRACVAL2;
    volatile uint16_t VAL2;
    volatile uint16_t FRACVAL3;
    volatile uint16_t VAL3;
    volatile uint16_t FRACVAL4;
    volatile uint16_t VAL4;
    volatile uint16_t FRACVAL5;
    volatile uint16_t VAL5;
    volatile uint16_t FRCTRL;
    volatile uint16_t OCTRL;
    volatile uint16_t STS;
    volatile uint16_t INTEN;
    volatile uint16_t DMAEN;
    volatile uint16_t TCTRL;
    volatile uint16_t DISMAP0;
    volatile uint16_t DISMAP1;
    volatile uint16_t DTCNT0;
    volatile uint16_t DTCNT1;
    volatile uint16_t CAPTCTRLA;
    volatile uint16_t CAPTCOMPA;
    volatile uint16_t CAPTCTRLB;
    volatile uint16_t CAPTCOMPB;
    volatile uint16_t CAPTCTRLX;
    volatile uint16_t CAPTCOMPX;
    volatile uint16_t CVAL0;
    volatile uint16_t CVAL0CYC;
    volatile uint16_t CVAL1;
    volatile uint16_t CVAL1CYC;
    volatile uint16_t CVAL2;
    volatile uint16_t CVAL2CYC;
    volatile uint16_t CVAL3;
    volatile uint16_t CVAL3CYC;
    volatile uint16_t CVAL4;
    volatile uint16_t CVAL4CYC;
    volatile uint16_t CVAL5;
    volatile uint16_t CVAL5CYC;
    uint16_t reserved1[4];
} FLEXPWM_SM;

typedef struct {
    FLEXPWM_SM SM[4];
    volatile uint16_t OUTEN;
    volatile uint16_t MASK;
    volatile uint16_t SWCOUT;
    volatile uint16_t DTSRCSEL;
    volatile uint16_t MCTRL;
    volatile uint16_t MCTRL2;
    volatile uint16_t FCTRL0;
    volatile uint16_t FSTS0;
    volatile uint16_t FFILT0;
    volatile uint16_t FTST0;
    volatile uint16_t FCTRL20;
} FLEXPWM_TypeDef;

#define FLEXPWM1    ((FLEXPWM_TypeDef*)FLEXPWM1_BASE)
#define CCM_CCGR4   (*(volatile uint32_t*)(CCM_BASE + 0x78))

static uint16_t motor_values[4] = {1000, 1000, 1000, 1000};
static bool pwm_initialized = false;

static void pwm_init(void) {
    if (pwm_initialized) return;
    
    CCM_CCGR4 |= (3UL << 16);
    
    volatile uint32_t *iomux = (volatile uint32_t*)IOMUXC_BASE;
    
    iomux[0x154/4] = 1;
    iomux[0x158/4] = 1;
    iomux[0x15C/4] = 1;
    iomux[0x160/4] = 1;
    
    for (int sm = 0; sm < 4; sm++) {
        FLEXPWM1->SM[sm].CTRL2 = (1 << 6);
        FLEXPWM1->SM[sm].CTRL = (6 << 4);
        FLEXPWM1->SM[sm].INIT = 0;
        FLEXPWM1->SM[sm].VAL0 = 0;
        FLEXPWM1->SM[sm].VAL1 = 15000;
        FLEXPWM1->SM[sm].VAL2 = 0;
        FLEXPWM1->SM[sm].VAL3 = 1500;
        FLEXPWM1->SM[sm].OCTRL = 0;
        FLEXPWM1->SM[sm].DTCNT0 = 0;
        FLEXPWM1->SM[sm].DTCNT1 = 0;
    }
    
    FLEXPWM1->OUTEN = 0x0F0F;
    FLEXPWM1->MCTRL = (0x0F << 8) | 0x0F;
    
    pwm_initialized = true;
}

static void pwm_write(uint8_t channel, uint16_t us) {
    if (channel > 3) return;
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    
    uint16_t val = us * 15;
    FLEXPWM1->SM[channel].VAL3 = val;
    FLEXPWM1->MCTRL |= (1 << (8 + channel));
    motor_values[channel] = us;
}

PIDController::PIDController(float kp, float ki, float kd, float imax)
    : kp(kp), ki(ki), kd(kd), imax(imax), integral(0), prevError(0), 
      prevDerivative(0), outMin(-1000), outMax(1000) {}

float PIDController::compute(float error, float dt) {
    float p = kp * error;
    
    integral += error * dt;
    if (integral > imax) integral = imax;
    else if (integral < -imax) integral = -imax;
    float i = ki * integral;
    
    float derivative = (error - prevError) / dt;
    derivative = 0.7f * prevDerivative + 0.3f * derivative;
    float d = kd * derivative;
    prevError = error;
    prevDerivative = derivative;
    
    float output = p + i + d;
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    
    return output;
}

void PIDController::reset() {
    integral = 0;
    prevError = 0;
    prevDerivative = 0;
}

void PIDController::setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PIDController::setOutputLimits(float min, float max) {
    outMin = min;
    outMax = max;
}

void AttitudeController::begin() {
    pwm_init();
    
    rollPID.setOutputLimits(-250, 250);
    pitchPID.setOutputLimits(-250, 250);
    yawPID.setOutputLimits(-250, 250);
    rollRatePID.setOutputLimits(-500, 500);
    pitchRatePID.setOutputLimits(-500, 500);
    yawRatePID.setOutputLimits(-300, 300);
    
    targetRoll = targetPitch = targetYaw = 0;
    throttle = 0;
    memset(motor, 0, sizeof(motor));
}

void AttitudeController::update(const float roll, const float pitch, const float yaw,
                                const float rollRate, const float pitchRate, const float yawRate,
                                float dt) {
    float rollError = targetRoll - roll;
    float pitchError = targetPitch - pitch;
    
    float yawError = targetYaw - yaw;
    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;
    
    float rollRateTarget = rollPID.compute(rollError, dt);
    float pitchRateTarget = pitchPID.compute(pitchError, dt);
    float yawRateTarget = yawPID.compute(yawError, dt);
    
    float rollCmd = rollRatePID.compute(rollRateTarget - rollRate, dt);
    float pitchCmd = pitchRatePID.compute(pitchRateTarget - pitchRate, dt);
    float yawCmd = yawRatePID.compute(yawRateTarget - yawRate, dt);
    
    mixMotors(rollCmd, pitchCmd, yawCmd, throttle);
}

void AttitudeController::setTargetAttitude(float r, float p, float y) {
    if (r > Config::Limits::MAX_ROLL) r = Config::Limits::MAX_ROLL;
    else if (r < -Config::Limits::MAX_ROLL) r = -Config::Limits::MAX_ROLL;
    targetRoll = r;
    
    if (p > Config::Limits::MAX_PITCH) p = Config::Limits::MAX_PITCH;
    else if (p < -Config::Limits::MAX_PITCH) p = -Config::Limits::MAX_PITCH;
    targetPitch = p;
    
    targetYaw = y;
}

void AttitudeController::setTargetRates(float rollRate, float pitchRate, float yawRate) {
    targetRoll = rollRate;
    targetPitch = pitchRate;
    targetYaw = yawRate;
}

void AttitudeController::setThrottle(float t) {
    if (t < 0) t = 0;
    else if (t > 1) t = 1;
    throttle = t;
}

void AttitudeController::getMotorOutputs(float& m1, float& m2, float& m3, float& m4) const {
    m1 = motor[0];
    m2 = motor[1];
    m3 = motor[2];
    m4 = motor[3];
}

void AttitudeController::mixMotors(float roll, float pitch, float yaw, float thr) {
    float base = Config::Limits::MOTOR_IDLE + 
                 thr * (Config::Limits::MOTOR_MAX - Config::Limits::MOTOR_IDLE);
    
    /*
     * X configuration:
     *   Front
     *  1     2
     *    \ /
     *    / \
     *  4     3
     *   Rear
     * 
     * Motor 1 (FL): CCW, +roll, +pitch, -yaw
     * Motor 2 (FR): CW,  -roll, +pitch, +yaw
     * Motor 3 (RR): CCW, -roll, -pitch, -yaw
     * Motor 4 (RL): CW,  +roll, -pitch, +yaw
     */
    motor[0] = base + roll * 0.5f + pitch * 0.5f - yaw * 0.25f;
    motor[1] = base - roll * 0.5f + pitch * 0.5f + yaw * 0.25f;
    motor[2] = base - roll * 0.5f - pitch * 0.5f - yaw * 0.25f;
    motor[3] = base + roll * 0.5f - pitch * 0.5f + yaw * 0.25f;
    
    for (int i = 0; i < 4; i++) {
        if (motor[i] < Config::Limits::MOTOR_MIN) motor[i] = Config::Limits::MOTOR_MIN;
        else if (motor[i] > Config::Limits::MOTOR_MAX) motor[i] = Config::Limits::MOTOR_MAX;
        
        pwm_write(i, (uint16_t)motor[i]);
    }
}
