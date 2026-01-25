#pragma once
#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>

class MPU6050Driver
{
public:
    bool init();
    void sleep();
    void wake();

    struct Data { float ax, ay, az, gx, gy, gz;};
    Data read();

private:
    void writeRegister(uint8_t reg, uint8_t val);
    const uint8_t MPU_ADDR = 0x68;
};