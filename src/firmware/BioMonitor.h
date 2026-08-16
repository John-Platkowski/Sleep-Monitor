// Top-level coordinator for the sleep monitor: owns the sensors, the heart rate
// filter, and the BLE link.

#ifndef BIOMONITOR_H
#define BIOMONITOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Matrix.h"
#include <Wire.h>
#include "MAX30102Driver.h"
#include "MPU6050Driver.h"
#include "BLEDriver.h"

// How often the BLE characteristic is notified, in milliseconds. Also the averaging window handed to
// MPU6050Driver::getEpochTemperatureC(), so one notification carries one epoch of temperature.
#define BLE_NOTIFY_PERIOD_MS 1000

// Drives the whole acquisition and reporting pipeline.
//
// Estimates heart rate from a PPG, which is badly corrupted by movement.
// An optical pulse sensor cannot distinguish a heartbeat from the
// sensor shifting against skin. The IMU is what resolves that. Rather than
// filtering motion out of the signal, motion is fed into the filter as a measure
// of how much the current reading can be trusted, so the estimate leans on its
// own model while the wearer moves and back on the sensor once they settle.
//
// Three execution contexts touch this object:
//
//   1. "BioTask", created by begin(), runs runLoop() at 50Hz. It reads both sensors and advances the
//      Kalman filter. Owns the I2C bus by convention: no other context may touch a sensor.
//   2. The FreeRTOS timer service task, which fires bleNotifyCallback() every BLE_NOTIFY_PERIOD_MS
//      and calls the three public accessors below.
//   3. The GPIO interrupt from the IMU motion pin, which only sets a flag.
//
// Contexts 1 and 2 run concurrently, so the accessors and the state they read are synchronized; see
// the individual declarations.
class BioMonitor
{
public:
    BioMonitor();

    // Brings up both sensors, arms the motion interrupt, starts the sampling task, and begins BLE
    // advertising and periodic notification.
    //
    // Sensor failures are logged to Serial but not fatal: a monitor with a dead PPG still reports
    // motion and temperature. Must be called exactly once.
    void begin();

    // Accessors used by the BLE notification callback. All three are safe to call from a task other
    // than the sampling task, which is how they are used.

    // Returns the current filtered heart rate in BPM. This is the filter's estimate, so it is defined
    // even when no beat has been detected recently; before the first beat it reads the assumed
    // resting rate.
    float getFilteredHR() const;

    // Returns the largest motion magnitude seen since the previous call, in g with gravity removed,
    // and resets the peak.
    //
    // Reports a peak rather than an instantaneous value because motion is sampled at 50Hz but
    // reported at 1Hz, and the instantaneous value decays toward zero between samples. Sampling it
    // once per second would miss almost every movement that did not land just before a notification.
    float getMotionScore();

    // Returns the mean temperature in degrees Celsius over the last completed epoch. See
    // MPU6050Driver::getEpochTemperatureC() for the epoch rules.
    float getEpochTemperatureC();

private:
    TaskHandle_t taskHandle;

    // FreeRTOS entry points take a plain function pointer, so these static trampolines forward into
    // the instance through pvParameters and the globalMonitor pointer respectively.
    static void taskTrampoline(void *pvParameters);
    static void IRAM_ATTR isrTrampoline();
    void IRAM_ATTR handleISR();

    // Body of the sampling task. Never returns.
    void runLoop();

    // Kalman filter state, owned by the sampling task. The state vector is [heart rate, rate of
    // change]^T.
    Matrix<float, 2, 1> x;           // State estimate: [BPM, BPM/s]^T.
    Matrix<float, 2, 2> P;           // Estimate covariance: the filter's uncertainty about x.
    Matrix<float, 2, 2> Q_adaptive;  // Process noise, retuned at runtime by adaptProcessNoise().

    // Preallocated working matrices, held as members so the update step allocates nothing.
    Matrix<float, 2, 2> Q_scratch;   // Q_adaptive scaled by current motion.
    Matrix<float, 2, 1> K;           // Kalman gain.
    Matrix<float, 1, 1> S;           // Innovation covariance.
    Matrix<float, 1, 1> y;           // Innovation: measured minus predicted BPM.

    // Fixed system model, shared by all instances. Defined in BioMonitor.cpp.
    static const Matrix<float, 2, 2> F;  // State transition over one sample.
    static const Matrix<float, 1, 2> H;  // Maps state to measurement space.
    static const Matrix<float, 2, 2> Q;  // Baseline process noise.
    static const Matrix<float, 1, 1> R;  // Measurement noise.
    static const Matrix<float, 2, 2> I;  // Identity.

    // Set by the motion ISR, cleared by the sampling task. volatile because the interrupt can store
    // to it between any two instructions of the task.
    volatile bool motionDetected;

    float lastAccelMag;  // Motion magnitude in g, decayed toward zero between interrupts.
    float motionPeak;    // Largest lastAccelMag since the last BLE report. Guarded by motionMux.

    // Returns the current motion magnitude, reading the IMU only if the interrupt has fired since the
    // last call and decaying the previous value otherwise.
    float readAccelIfMotion();

    // Guards motionPeak, written by the sampling task and drained by the BLE timer task.
    portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;

    // Advances the estimate one sample into the future, widening uncertainty in proportion to
    // accelMag so that movement loosens the filter's confidence in what it is about to measure.
    void predictKalman(float accelMag);

    // Corrects the prediction with an observed heart rate in BPM.
    void updateKalman(float measurement);

    // Retunes process noise from the size of the innovation, so a run of surprising measurements
    // leaves the filter more willing to move.
    void adaptProcessNoise(float innovation);

    // Formats the current readings for the BLE characteristic. Runs on the timer service task;
    // context is the BioMonitor instance.
    static String bleNotifyCallback(void* context);

    MAX30102Driver ppg;
    MPU6050Driver imu;
    BLEDriver ble;
};

#endif