// Wrapper for the SparkFun MAX30102 Driver
#pragma once
#include <Wire.h>
#include <MAX30105.h>
#include <Arduino.h>

class MAX30102Driver
{
public:
    bool init();
    void sleep();
    void wake();

    // Returns raw IR value from sensor
    uint32_t readIR();
    
private:
    MAX30105 sensor;
};