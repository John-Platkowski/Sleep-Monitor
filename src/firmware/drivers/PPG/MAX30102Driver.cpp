#include "MAX30102Driver.h"

bool MAX30102Driver::init()
{
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) 
    {
        return false;
    }
    
    // ledMode 2 is red + infrared, the only two the MAX30102 carries. 200sps with 4x averaging emits
    // 50Hz, which lands exactly on the sample loop.
    // (powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange)
    sensor.setup(0x1F, 4, 2, 200, 411, 4096);

    // setup() applies powerLevel to every channel including green, which a MAX30102 has neither a die
    // nor a 2-LED FIFO slot for.
    sensor.setPulseAmplitudeGreen(0);

    lastBeatTick = xTaskGetTickCount();
    firstBeat = true;
    lastFingerPresent = false;

    return true;
}

void MAX30102Driver::sleep()
{
    sensor.shutDown();
    lastFingerPresent = false;
}

void MAX30102Driver::wake()
{
    sensor.wakeUp();

    // Shutting down leaves the FIFO and its pointers alone; resetting them leaves the next read
    // waiting on data taken after the wake.
    sensor.clearFIFO();

    // Reset beat detection state after wake
    lastBeatTick = xTaskGetTickCount();
    firstBeat = true;
    lastFingerPresent = false;
}

uint32_t MAX30102Driver::readIR()
{
    return sensor.getIR();
}

float MAX30102Driver::processSample()
{
    uint32_t irValue = sensor.getIR();
    lastFingerPresent = (irValue >= FINGER_THRESHOLD);

    // Arm firstBeat, so the next beat after a finger returns only records a reference point.
    if (!lastFingerPresent)
    {
        firstBeat = true;
        return -1.0f;
    }

    if (checkForBeat(irValue))
    {
        TickType_t currentTick = xTaskGetTickCount();

        // Nothing to measure against yet, so record the reference and wait for the next beat to close
        // an interval.
        if (firstBeat)
        {
            lastBeatTick = currentTick;
            firstBeat = false;
            return -1.0f;
        }

        // Unsigned subtraction, so this stays correct across a tick counter wrap.
        TickType_t deltaTicks = currentTick - lastBeatTick;
        lastBeatTick = currentTick;

        float deltaMs = (float)deltaTicks * (1000.0f / configTICK_RATE_HZ);

        // Guard the division below.
        if (deltaMs < 1.0f)
        {
            return -1.0f;
        }

        float bpm = 60000.0f / deltaMs;

        // An interval outside human range means the detector miscounted, either splitting one beat in
        // two or missing one entirely.
        if (bpm < MIN_BPM || bpm > MAX_BPM)
        {
            return -1.0f;
        }

        return bpm;
    }
    
    // No beat detected this sample
    return -1.0f;
}