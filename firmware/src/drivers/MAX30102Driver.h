// Wrapper for the SparkFun MAX30102 Driver
#pragma once
#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class MAX30102Driver
{
public:
    bool init();
    void sleep();
    void wake();

    // Returns raw IR value from sensor
    uint32_t readIR();
    
    // Returns instantaneous BPM if beat detected, -1.0f otherwise
    float processSample();
    
    // Check if finger is present on sensor
    bool isFingerPresent();
    
private:
    MAX30105 sensor;
    TickType_t lastBeatTick = 0;
    bool firstBeat = true;
    
    static constexpr uint32_t FINGER_THRESHOLD = 50000;  // IR threshold for finger detection
    static constexpr float MIN_BPM = 30.0f;
    static constexpr float MAX_BPM = 220.0f;
};