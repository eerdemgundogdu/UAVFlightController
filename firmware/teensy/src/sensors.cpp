/*
 * IMU Bare-Metal Driver - MPU9250
 * Direct I2C register access on Teensy 4.1
 */

#include "sensors.h"
#include <stdint.h>
#include <math.h>

#define LPI2C1_BASE     0x403F0000UL
#define IOMUXC_BASE     0x401F8000UL
#define CCM_BASE        0x400FC000UL

typedef struct {
    volatile uint32_t VERID;
    volatile uint32_t PARAM;
    uint32_t reserved0[2];
    volatile uint32_t MCR;
    volatile uint32_t MSR;
    volatile uint32_t MIER;
    volatile uint32_t MDER;
    volatile uint32_t MCFGR0;
    volatile uint32_t MCFGR1;
    volatile uint32_t MCFGR2;
    volatile uint32_t MCFGR3;
    uint32_t reserved1[4];
    volatile uint32_t MDMR;
    uint32_t reserved2;
    volatile uint32_t MCCR0;
    uint32_t reserved3;
    volatile uint32_t MCCR1;
    uint32_t reserved4;
    volatile uint32_t MFCR;
    volatile uint32_t MFSR;
    volatile uint32_t MTDR;
    uint32_t reserved5[3];
    volatile uint32_t MRDR;
} LPI2C_TypeDef;

#define LPI2C1      ((LPI2C_TypeDef*)LPI2C1_BASE)

#define CCM_CCGR2   (*(volatile uint32_t*)(CCM_BASE + 0x70))

#define LPI2C_MCR_MEN       (1UL << 0)
#define LPI2C_MCR_RST       (1UL << 1)
#define LPI2C_MSR_TDF       (1UL << 0)
#define LPI2C_MSR_RDF       (1UL << 1)
#define LPI2C_MSR_EPF       (1UL << 8)
#define LPI2C_MSR_NDF       (1UL << 10)

#define MPU9250_ADDR        0x68
#define AK8963_ADDR         0x0C
#define BMP280_ADDR         0x76

#define MPU_WHO_AM_I        0x75
#define MPU_PWR_MGMT_1      0x6B
#define MPU_PWR_MGMT_2      0x6C
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_ACCEL_CONFIG2   0x1D
#define MPU_INT_PIN_CFG     0x37
#define MPU_ACCEL_XOUT_H    0x3B
#define MPU_GYRO_XOUT_H     0x43

static void i2c_configure_pins(void) {
    volatile uint32_t *iomux = (volatile uint32_t*)IOMUXC_BASE;
    
    iomux[0x14C/4] = 3 | (1UL << 4);
    iomux[0x150/4] = 3 | (1UL << 4);
    
    volatile uint32_t *pad = (volatile uint32_t*)(IOMUXC_BASE + 0x200);
    pad[0x33C/4] = 0xF8B0;
    pad[0x340/4] = 0xF8B0;
}

static void i2c_init(void) {
    CCM_CCGR2 |= (3UL << 6);
    
    i2c_configure_pins();
    
    LPI2C1->MCR = LPI2C_MCR_RST;
    LPI2C1->MCR = 0;
    
    LPI2C1->MCFGR1 = (0UL << 0);
    LPI2C1->MCCR0 = (25UL << 0) | (12UL << 8) | (25UL << 16) | (5UL << 24);
    
    LPI2C1->MCR = LPI2C_MCR_MEN;
}

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (4UL << 8) | (addr << 1);
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (0UL << 8) | reg;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (0UL << 8) | val;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (2UL << 8);
    
    while (!(LPI2C1->MSR & LPI2C_MSR_EPF));
    LPI2C1->MSR = LPI2C_MSR_EPF;
    
    return !(LPI2C1->MSR & LPI2C_MSR_NDF);
}

static uint8_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (4UL << 8) | (addr << 1);
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (0UL << 8) | reg;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (4UL << 8) | (addr << 1) | 1;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (1UL << 8) | 0;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_RDF));
    uint8_t val = LPI2C1->MRDR & 0xFF;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_EPF));
    LPI2C1->MSR = LPI2C_MSR_EPF;
    
    return val;
}

static bool i2c_read_burst(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (4UL << 8) | (addr << 1);
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (0UL << 8) | reg;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (4UL << 8) | (addr << 1) | 1;
    
    while (!(LPI2C1->MSR & LPI2C_MSR_TDF));
    LPI2C1->MTDR = (1UL << 8) | (len - 1);
    
    for (uint8_t i = 0; i < len; i++) {
        while (!(LPI2C1->MSR & LPI2C_MSR_RDF));
        buf[i] = LPI2C1->MRDR & 0xFF;
    }
    
    while (!(LPI2C1->MSR & LPI2C_MSR_EPF));
    LPI2C1->MSR = LPI2C_MSR_EPF;
    
    return true;
}

bool IMU::begin() {
    i2c_init();
    
    for (volatile int i = 0; i < 100000; i++);
    
    uint8_t whoami = i2c_read_reg(MPU9250_ADDR, MPU_WHO_AM_I);
    if (whoami != 0x71 && whoami != 0x73) {
        return false;
    }
    
    i2c_write_reg(MPU9250_ADDR, MPU_PWR_MGMT_1, 0x80);
    for (volatile int i = 0; i < 100000; i++);
    
    i2c_write_reg(MPU9250_ADDR, MPU_PWR_MGMT_1, 0x01);
    i2c_write_reg(MPU9250_ADDR, MPU_PWR_MGMT_2, 0x00);
    
    i2c_write_reg(MPU9250_ADDR, MPU_CONFIG, 0x03);
    i2c_write_reg(MPU9250_ADDR, MPU_GYRO_CONFIG, 0x10);
    i2c_write_reg(MPU9250_ADDR, MPU_ACCEL_CONFIG, 0x08);
    i2c_write_reg(MPU9250_ADDR, MPU_ACCEL_CONFIG2, 0x03);
    
    i2c_write_reg(MPU9250_ADDR, MPU_INT_PIN_CFG, 0x02);
    
    return true;
}

bool IMU::update() {
    uint8_t buf[14];
    if (!i2c_read_burst(MPU9250_ADDR, MPU_ACCEL_XOUT_H, buf, 14)) {
        return false;
    }
    
    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];
    
    data.accel.x = (ax / 4096.0f - accelBias.x) * 9.81f;
    data.accel.y = (ay / 4096.0f - accelBias.y) * 9.81f;
    data.accel.z = (az / 4096.0f - accelBias.z) * 9.81f;
    
    const float gyro_scale = (1000.0f / 32768.0f) * (3.14159265f / 180.0f);
    data.gyro.x = gx * gyro_scale - gyroBias.x;
    data.gyro.y = gy * gyro_scale - gyroBias.y;
    data.gyro.z = gz * gyro_scale - gyroBias.z;
    
    data.valid = true;
    lastUpdate = ARM_DWT_CYCCNT;
    
    return true;
}

void IMU::calibrate() {
    Vec3 gyroSum, accelSum;
    const int samples = 2000;
    
    for (int i = 0; i < samples; i++) {
        update();
        gyroSum = gyroSum + data.gyro;
        accelSum = accelSum + data.accel;
        for (volatile int j = 0; j < 3000; j++);
    }
    
    gyroBias = gyroSum * (1.0f / samples);
    accelBias.x = accelSum.x / samples / 9.81f;
    accelBias.y = accelSum.y / samples / 9.81f;
    accelBias.z = (accelSum.z / samples / 9.81f) - 1.0f;
    
    calibrated = true;
}

bool Barometer::begin() {
    for (volatile int i = 0; i < 10000; i++);
    
    uint8_t id = i2c_read_reg(BMP280_ADDR, 0xD0);
    if (id != 0x58 && id != 0x60) {
        return false;
    }
    
    i2c_write_reg(BMP280_ADDR, 0xF4, 0x57);
    i2c_write_reg(BMP280_ADDR, 0xF5, 0x10);
    
    return true;
}

bool Barometer::update() {
    uint8_t buf[6];
    i2c_read_burst(BMP280_ADDR, 0xF7, buf, 6);
    
    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    
    data.temperature = adc_T / 5120.0f;
    data.pressure = adc_P / 256.0f;
    data.altitude = 44330.0f * (1.0f - powf(data.pressure / seaLevelPressure / 100.0f, 0.1903f));
    data.valid = true;
    
    return true;
}

void Barometer::setSeaLevelPressure(float hPa) {
    seaLevelPressure = hPa * 100.0f;
}

bool GPS::begin() {
    return true;
}

bool GPS::update() {
    return false;
}

void GPS::parseNMEA(const char* sentence) {
    (void)sentence;
}
