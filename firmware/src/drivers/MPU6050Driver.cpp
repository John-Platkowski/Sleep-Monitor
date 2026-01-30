#include "MPU6050Driver.h"

bool MPU6050Driver::init()
{
    Wire.begin();
    Wire.setClock(100000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    return true;
}

void MPU6050Driver::configureMotionInterrupt(uint8_t threshold, uint8_t duration)
{
    // Set motion detection threshold (LSB = 2mg in default ±2g range)
    writeRegister(0x1F, threshold);  // MOT_THR
    
    // Set motion detection duration (LSB = 1ms)
    writeRegister(0x20, duration);   // MOT_DUR
    
    // Enable motion detection interrupt
    writeRegister(0x38, 0x40);       // INT_ENABLE: MOT_EN
    
    // Configure interrupt pin: active low, push-pull, latched until cleared
    writeRegister(0x37, 0x30);       // INT_PIN_CFG: LATCH_INT_EN, INT_RD_CLEAR
}

void MPU6050Driver::clearInterrupt()
{
    // Reading INT_STATUS clears the interrupt (when INT_RD_CLEAR is set)
    readRegister(0x3A);
}

void MPU6050Driver::writeRegister(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

uint8_t MPU6050Driver::readRegister(uint8_t reg)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
    return Wire.read();
}

MPU6050Driver::Data MPU6050Driver::read()
{
    Data data;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true); // 6 accel + 2 temp + 6 gyro = 14 bytes
    data.ax = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x3B-0x3C (acceleration x)
    data.ay = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x3D-0x3E (acceleration y)
    data.az = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x3F-0x40 (acceleration z)
    Wire.read(); Wire.read();                            // 0x41-0x42 (temperature) - discard
    data.gx = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x43-0x44 (gyro x)
    data.gy = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x45-0x46 (gyro y)
    data.gz = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x47-0x48 (gyro z)
    return data;
}

float MPU6050Driver::getAccelerationMagnitude(Data& data)
{
    float total_force = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    return abs(total_force - 1.0f);
}

void MPU6050Driver::sleep()
{
    writeRegister(0x6B, 0b01000000); // PWR_MGMT_1 register: Sleep mode
}

void MPU6050Driver::wake()
{
    writeRegister(0x6B, 0b00000000); // PWR_MGMT_1 register: Wake mode
}