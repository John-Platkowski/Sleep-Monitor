// I2C driver for the InvenSense six-axis IMU, covering acceleration, wake-on-motion interrupts, and
// die temperature.
//
// Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#pragma once
#include <Wire.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// WHO_AM_I (register 0x75) value identifying an MPU6500.
#define MPU_WHOAMI_6500 0x70

// Driver for the MPU6500 accelerometer/gyroscope.
//
// The class name says 6050 for historical reasons. The breakout in this build is marked MPU-6050, but
// it answers WHO_AM_I with 0x70 and follows the MPU6500 register map, which uses a different
// temperature formula and has no motion duration register. init() rejects any other part rather than
// reading a 6050 with 6500 constants and silently reporting wrong degrees.
//
// Thread safety: addTemperatureSample() and getEpochTemperatureC() may be called concurrently from
// different tasks and are used that way, with the sampling task accumulating and the BLE timer task
// draining. Every other method is unsynchronized and must be confined to one task, since they share
// the I2C bus without holding a lock.
//
// Call order: init() first, then configureMotionInterrupt() if wake-on-motion is wanted. All other
// methods assume init() returned true.
class MPU6050Driver
{
public:
    // Starts the I2C bus, verifies the part, and wakes it from reset.
    //
    // Returns false if WHO_AM_I is not an MPU6500, in which case no other method should be called.
    bool init();

    // Enables the wake-on-motion interrupt on the INT pin, active low and latched until cleared by
    // clearInterrupt().
    //
    // threshold: acceleration that trips the interrupt, in WOM_THR counts of 4mg each. A threshold of
    //     10 is roughly 40mg.
    void configureMotionInterrupt(uint8_t threshold);

    // Puts the part into low-power sleep, and brings it back out.
    //
    // Neither is currently called anywhere; power management is still a TODO in BioMonitor::runLoop().
    void sleep();
    void wake();

    // One six-axis sample as raw signed sensor counts, not physical units. At the configured +/-2g
    // range, acceleration is 16384 counts per g. The gyroscope fields are populated but unread.
    struct Data { float ax, ay, az, gx, gy, gz;};

    // Reads all six axes in a single burst.
    //
    // Returns a zero-filled Data on any I2C error, which is indistinguishable from a genuine reading
    // of zero. Errors are logged to Serial.
    Data read();

    // Returns total acceleration in g with gravity removed, so a stationary sensor reads near 0.0 in
    // any orientation.
    float getAccelerationMagnitude(Data& data);

    // Clears the latched motion interrupt by reading INT_STATUS. Must be called after each interrupt
    // or no further one will fire.
    void clearInterrupt();

    // Adds one temperature reading to the current epoch, at most once per TEMP_SAMPLE_INTERVAL_MS.
    // Earlier calls return without touching the bus, so this is cheap to call from the sample loop.
    //
    // nowMs: current time from millis().
    void addTemperatureSample(uint32_t nowMs);

    // Returns the mean temperature in degrees Celsius over the epoch just ended, then starts a new one.
    //
    // Before epochDurationMs has elapsed, returns the previous epoch's value rather than a partial
    // mean, so callers see a stable reading. Successive results are clamped to TEMP_MAX_DELTA_C to
    // reject spikes. Returns 0.0 if no sample has ever been taken.
    //
    // nowMs: current time from millis().
    // epochDurationMs: how long each averaging window lasts.
    float getEpochTemperatureC(uint32_t nowMs, uint32_t epochDurationMs);

private:
    // Single-register bus access. Both block for the duration of the transfer.
    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);

    // Reads TEMP_OUT as a raw signed count, before scaling to Celsius.
    int16_t readTempRaw();

    // I2C address with pin AD0 tied low.
    const uint8_t MPU_ADDR = 0x68;

    // Minimum spacing between temperature samples. The sample loop runs at 50Hz, far faster than die
    // temperature can change, so this throttles bus traffic without losing information.
    static constexpr uint32_t TEMP_SAMPLE_INTERVAL_MS = 100;

    // MPU6500 conversion: degC = (raw - RoomTemp_Offset)/333.87 + 21.0, where RoomTemp_Offset is 0.
    // These differ from the MPU6050's 340 and 36.53, which is why init() refuses to run on a 6050.
    static constexpr float TEMP_SCALE = 333.87f;
    static constexpr float TEMP_OFFSET_C = 21.0f;

    // Largest change accepted between consecutive epochs, in degrees Celsius. Skin temperature cannot
    // move this fast, so a larger jump is taken as sensor noise and clamped.
    static constexpr float TEMP_MAX_DELTA_C = 0.3f;

    // Accumulator for the epoch in progress. Guarded by tempMux.
    float tempEpochSum = 0.0f;
    uint32_t tempEpochCount = 0;
    uint32_t tempEpochStartMs = 0;
    uint32_t tempLastSampleMs = 0;

    // Last value handed out, used as the clamp reference and as the reading returned mid-epoch.
    // Guarded by tempMux.
    float tempLastReportedC = 0.0f;
    bool tempHasReported = false;

    // Guards every field from tempEpochSum down. The sampling task writes them while the BLE timer
    // task drains and resets them, so the drain must be atomic against the accumulate. Never held
    // across an I2C transfer.
    portMUX_TYPE tempMux = portMUX_INITIALIZER_UNLOCKED;
};