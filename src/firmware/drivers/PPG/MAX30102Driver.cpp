#include "MAX30102Driver.h"

bool MAX30102Driver::init()
{
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) 
    {
        return false;
    }
    
    sensor.setup();
    sensor.setPulseAmplitudeRed(0x1F);
    sensor.setPulseAmplitudeGreen(0);
    
    // Initialize timing
    lastBeatTick = xTaskGetTickCount();
    firstBeat = true;
    
    return true;
}

void MAX30102Driver::sleep()
{
    sensor.shutDown();
}

void MAX30102Driver::wake()
{
    sensor.wakeUp();
    // Reset beat detection state after wake
    lastBeatTick = xTaskGetTickCount();
    firstBeat = true;
}

uint32_t MAX30102Driver::readIR()
{
    return sensor.getIR();
}

float MAX30102Driver::processSample()
{
    uint32_t irValue = sensor.getIR();
    
    // No finger detected, reset state and return invalid
    if (irValue < FINGER_THRESHOLD)
    {
        firstBeat = true;
        return -1.0f;
    }
    
    // Check for beat using the PBA algorithm from heartRate library
    if (checkForBeat(irValue))
    {
        TickType_t currentTick = xTaskGetTickCount();
        
        // First beat after init/wake/finger placement, just record time
        if (firstBeat)
        {
            lastBeatTick = currentTick;
            firstBeat = false;
            return -1.0f;
        }
        
        TickType_t deltaTicks = currentTick - lastBeatTick;
        lastBeatTick = currentTick;
        
        // Convert ticks to milliseconds, then to BPM
        float deltaMs = (float)deltaTicks * (1000.0f / configTICK_RATE_HZ);
        
        if (deltaMs < 1.0f)
        {
            return -1.0f;
        }
        
        float bpm = 60000.0f / deltaMs;
        
        if (bpm < MIN_BPM || bpm > MAX_BPM)
        {
            return -1.0f;
        }
        
        return bpm;
    }
    
    // No beat detected this sample
    return -1.0f;
}