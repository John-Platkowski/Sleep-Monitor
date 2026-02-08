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

    // Temperature: epoch-aggregate with velocity clamp; sample at fixed interval to avoid pure polling
    void addTemperatureSample(uint32_t nowMs);
    float getEpochTemperatureC(uint32_t nowMs, uint32_t epochDurationMs);

private:
    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
    int16_t readTempRaw();
    const uint8_t MPU_ADDR = 0x68;

    // Epoch aggregation and simple velocity filter for temperature
    static constexpr uint32_t TEMP_SAMPLE_INTERVAL_MS = 100;  // min time between samples
    static constexpr float TEMP_OFFSET_C = 36.53f;
    static constexpr float TEMP_SCALE = 340.0f;
    static constexpr float TEMP_MAX_DELTA_C = 0.3f;  // max change between epochs (deg C / epoch)
    float tempEpochSum = 0.0f;
    uint32_t tempEpochCount = 0;
    uint32_t tempEpochStartMs = 0;
    uint32_t tempLastSampleMs = 0;
    float tempLastReportedC = 0.0f;
    bool tempHasReported = false;
};