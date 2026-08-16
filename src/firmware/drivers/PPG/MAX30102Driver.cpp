#include "MAX30102Driver.h"

bool MAX30102Driver::init()
{
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) 
    {
        return false;
    }
    
    // Library defaults are wrong for this build in two ways: ledMode 3 reserves a green FIFO slot the
    // MAX30102 has no LED for, and 400sps with 4x averaging emits 100Hz into a task that consumes at
    // 50Hz, so half of every read is fetched over I2C and dropped. 200sps / 4 lands exactly on the
    // 50Hz sample loop.
    // (powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange)
    sensor.setup(0x1F, 4, 2, 200, 411, 4096);

    // setup() applies powerLevel to every channel including green. Harmless on a MAX30102, which has
    // no green die and no green slot in 2-LED mode, but kept explicit in case the part is a MAX30105.
    sensor.setPulseAmplitudeGreen(0);

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

    // Arming firstBeat here means removing and replacing a finger cannot produce a bogus interval
    // spanning the gap.
    if (irValue < FINGER_THRESHOLD)
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

        // Guard the division below; two beats in the same tick would divide by roughly zero.
        if (deltaMs < 1.0f)
        {
            return -1.0f;
        }

        float bpm = 60000.0f / deltaMs;

        // An interval outside human range means the detector miscounted, either splitting one beat in
        // two or missing one entirely. Dropping it beats feeding the filter a value it would partly
        // believe.
        if (bpm < MIN_BPM || bpm > MAX_BPM)
        {
            return -1.0f;
        }

        return bpm;
    }
    
    // No beat detected this sample
    return -1.0f;
}