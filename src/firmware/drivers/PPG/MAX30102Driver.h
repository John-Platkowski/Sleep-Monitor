// Heart rate acquisition from the MAX30102 pulse oximeter, wrapping SparkFun's MAX30105 library.

#pragma once
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Turns the raw infrared photoplethysmogram into beat-to-beat heart rate.
//
// Beats are found by Maxim's peripheral beat amplitude algorithm in heartRate.h, and rate comes from
// the interval between consecutive beats rather than any frequency analysis, so each result derives
// from exactly two beats and carries no smoothing. Callers are expected to filter the output;
// BioMonitor runs it through a Kalman filter.
//
// Thread safety: none. All methods share the I2C bus and the beat timing state, so one task must own
// the instance.
//
// Call order: init() first, then processSample() at the rate the sensor was configured for.
class MAX30102Driver
{
public:
    // Configures the sensor for two-LED operation at 50Hz output. Assumes the I2C bus is already open,
    // which MPU6050Driver::init() handles.
    //
    // Returns false if the sensor does not respond.
    bool init();

    // Shuts the LEDs and ADC down, and brings them back. wake() resets beat timing and empties the
    // FIFO, so the first sample read afterwards is one taken after the wake. processSample() must not
    // be called between the two: a shut down part still answers on I2C, and each call costs the
    // library's 250ms FIFO timeout.
    void sleep();
    void wake();

    // Returns the most recent raw infrared count, unprocessed. Useful for diagnosing sensor placement,
    // where the absolute level is what matters.
    uint32_t readIR();

    // Reads one sample and returns the instantaneous heart rate in BPM if that sample completed a beat.
    //
    // Returns a negative value when no rate is available, which is the usual case: at 50Hz only about
    // one call in fifty lands on a beat, so callers must test the sign before using the result. The
    // four cases are no finger on the sensor, no beat in this sample, the first beat after placement
    // or wake, and an interval implying a rate outside MIN_BPM to MAX_BPM.
    //
    // Must be called at a steady rate; the beat detector's internal filters assume evenly spaced
    // samples.
    float processSample();

    // Whether the last processSample() saw enough reflected infrared to count as skin contact, which
    // separates the one of its four negative returns that means the device is not being worn from the
    // three that mean it is worn with no rate yet. False until processSample() has run since a wake().
    bool fingerPresent() const { return lastFingerPresent; }

private:
    MAX30105 sensor;

    // Set by processSample() from the same sample the reported rate came from.
    bool lastFingerPresent = false;

    // Tick count at the last accepted beat, for measuring the next interval.
    TickType_t lastBeatTick = 0;

    // True until a beat establishes a reference point. The first beat after placement or wake only
    // records a timestamp, since there is no earlier beat to measure an interval against.
    bool firstBeat = true;

    // Infrared count below which no finger is considered present. Skin returns far more reflected
    // light than open air, so this separates the two cleanly.
    static constexpr uint32_t FINGER_THRESHOLD = 50000;

    // Plausible human heart rate bounds in BPM. Intervals outside this range are rejected as
    // artifacts, usually a double-counted peak or a missed beat.
    static constexpr float MIN_BPM = 30.0f;
    static constexpr float MAX_BPM = 220.0f;
};