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
    // Ensure accelerometer is enabled, clear standby bits for accel
    writeRegister(0x6C, 0x00); // PWR_MGMT_2 enable all axes
    
    // Configure high-pass filter for motion detection
    writeRegister(0x1C, 0x01); // ACCEL_CONFIG, HPF = 5Hz
    
    // Set motion detection threshold
    writeRegister(0x1F, threshold); // MOT_THR
    
    // Set motion detection duration
    writeRegister(0x20, duration); // MOT_DUR
    
    // Configure motion detection control
    writeRegister(0x69, 0x15); // MOT_DETECT_CTRL, delay counter, all axes
    
    // Enable motion detection interrupt
    writeRegister(0x38, 0x40); // INT_ENABLE, MOT_EN
    
    // Configure interrupt pin: ACTIVE LOW, push-pull, latched until cleared
    writeRegister(0x37, 0xB0); // INT_PIN_CFG, INT_LEVEL=1, LATCH_INT_EN, INT_RD_CLEAR
}

void MPU6050Driver::clearInterrupt()
{
    // Reading INT_STATUS clears the interrupt when INT_RD_CLEAR is set
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
    Wire.write(0x3B); // starting register ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true); // 6 accel + 2 temp + 6 gyro = 14 bytes
    data.ax = (int16_t)(Wire.read() << 8 | Wire.read()); // acceleration x
    data.ay = (int16_t)(Wire.read() << 8 | Wire.read()); // acceleration y
    data.az = (int16_t)(Wire.read() << 8 | Wire.read()); // acceleration z
    Wire.read(); Wire.read();                            // temperature, discarded
    data.gx = (int16_t)(Wire.read() << 8 | Wire.read()); // gyro x
    data.gy = (int16_t)(Wire.read() << 8 | Wire.read()); // gyro y
    data.gz = (int16_t)(Wire.read() << 8 | Wire.read()); // gyro z
    return data;
}

float MPU6050Driver::getAccelerationMagnitude(Data& data)
{
    float total_force = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    return abs(total_force - 1.0f);
}

void MPU6050Driver::sleep()
{
    writeRegister(0x6B, 0b01000000); // PWR_MGMT_1 register, Sleep mode
}

void MPU6050Driver::wake()
{
    writeRegister(0x6B, 0b00000000); // PWR_MGMT_1 register, Wake mode
}