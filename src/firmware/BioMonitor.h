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
    float getEpochTemperatureC();

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
    
    // Scratch matrices
    Matrix<float, 2, 2> Q_scratch;  // Motion-scaled Q for predict step
    Matrix<float, 2, 1> K;    // Kalman gain for update step
    Matrix<float, 1, 1> S;    // Innovation covariance
    Matrix<float, 1, 1> y;    // Innovation (measurement residual)
    
    // System model
    static const Matrix<float, 2, 2> F;  // State transition
    static const Matrix<float, 1, 2> H;  // Measurement matrix
    static const Matrix<float, 2, 2> Q;  // Base process noise covariance
    static const Matrix<float, 1, 1> R;  // Base measurement noise covariance
    static const Matrix<float, 2, 2> I;  // Identity matrix

    // Motion detection via IMU interrupt
    volatile bool motionDetected;
    float lastAccelMag; // Kalman
    float currentAccelMagUI; // UI
    float readAccelIfMotion();

    void predictKalman(float controlInput);
    void updateKalman(float measurement);
    void adaptProcessNoise(float innovation);

    // BLE notification callback
    static String bleNotifyCallback(void* context);

    MAX30102Driver ppg;
    MPU6050Driver imu;
    BLEDriver ble;
};

#endif