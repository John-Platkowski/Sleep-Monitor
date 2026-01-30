#ifndef BIOMONITOR_H
#define BIOMONITOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Matrix.h"
#include <Wire.h>
#include "drivers/MAX30102Driver.h"
#include "drivers/MPU6050Driver.h"
#include "drivers/BLEDriver.h"

// Default BLE notification period in milliseconds
#define BLE_NOTIFY_PERIOD_MS 1000

class BioMonitor 
{
public:
    BioMonitor();
    void begin();
    
    // Public accessors for BLE callback
    float getFilteredHR() const;
    float getMotionScore() const;

private:
    TaskHandle_t taskHandle;

    static void taskTrampoline(void *pvParameters);
    static void isrTrampoline();
    void handleISR();

    void runLoop();

    // Kalman Filter for Heart Rate
    // State: [HR, HR_velocity]^T - tracks heart rate and its rate of change
    Matrix<float, 2, 1> x;      // State estimate
    Matrix<float, 2, 2> P;      // Estimate covariance
    Matrix<float, 2, 2> Q_adaptive;  // Adaptive process noise (runtime)
    Matrix<float, 1, 1> R_adaptive; // Adaptive measurement noise (runtime)
    
    // System model (constant, defined in cpp)
    static const Matrix<float, 2, 2> F;  // State transition
    static const Matrix<float, 2, 1> B;  // Control matrix (maps accel to state)
    static const Matrix<float, 1, 2> H;  // Measurement matrix
    static const Matrix<float, 2, 2> Q;  // Base process noise covariance
    static const Matrix<float, 1, 1> R;  // Base measurement noise covariance

    // Motion detection via IMU interrupt (no polling)
    volatile bool motionDetected;
    float lastAccelMag;
    float readAccelIfMotion();

    void predictKalman(float controlInput);
    void updateKalman(float measurement);
    void adaptProcessNoise(float innovation);

    // BLE notification callback (static for function pointer compatibility)
    static String bleNotifyCallback(void* context);

    MAX30102Driver ppg;
    MPU6050Driver imu;
    BLEDriver ble;
};

#endif