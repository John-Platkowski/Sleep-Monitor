#pragma once
#include <Wire.h>
#include <Arduino.h>

class MPU6050Driver
{
public:
    bool init();
    void configureMotionInterrupt(uint8_t threshold, uint8_t duration);
    void sleep();
    void wake();

    struct Data { float ax, ay, az, gx, gy, gz;};
    Data read();
    float getAccelerationMagnitude(Data& data);
    void clearInterrupt();

private:
    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
    const uint8_t MPU_ADDR = 0x68;
};