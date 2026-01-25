// Wrapper for the SparkFun MAX30102 Driver
#pragma once
#include <Wire.h>
#include <MAX30105.h>
#include <Arduino.h>

class MAX30102Driver
{
public:
    bool init()
    {
        return internalSensor.begin(Wire, I2C_SPEED_FAST);
    }
    void sleep()
    {
        internalSensor.shutDown();
    }
    void wake()
    {
        internalSensor.wakeUp();
    }

    float read()
    {
        return internalSensor.getIR();
    }
    
private:
    MAX30105 internalSensor;
};