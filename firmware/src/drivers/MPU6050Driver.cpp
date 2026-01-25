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

void MPU6050Driver::writeRegister(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

MPU6050Driver::Data MPU6050Driver::read()
{
    Data data;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
    data.ax = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    data.ay = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    data.az = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    data.gx = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    data.gy = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    data.gz = (int16_t)(Wire.read() << 8 | Wire.read()); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    return data;
}

void MPU6050Driver::sleep()
{
    writeRegister(0x6B, 0b01000000); // PWR_MGMT_1 register: Sleep mode
}

void MPU6050Driver::wake()
{
    writeRegister(0x6B, 0b00000000); // PWR_MGMT_1 register: Wake mode
}